"""Offline PPO weight search over an uncertain sagittal hip damping model."""
import argparse
import csv
from pathlib import Path
import math
from types import SimpleNamespace

import gymnasium as gym
import numpy as np
from scipy.linalg import solve_discrete_are
from stable_baselines3 import PPO
import torch

import run_pso as files

OUT = files.HERE / "build/hip_ppo"
AXES = ((0,), (1,), (4, 6), (5, 7), (8,), (9,), (14, 15), (16, 17))
DAMP = np.zeros((2, 14))
DAMP[0, 5] = 1
DAMP[1, 7] = 1
DAMP[:, 9] = -1


def audit_vofa(out):
    folder = files.ROOT / "Wheel_Leg_Code/diagnostics/height_roll_capture_20261119"
    result = {"hip_damping_identified": False, "files": []}
    for path in sorted(folder.glob("vofa_*.csv")):
        with path.open(encoding="utf-8-sig") as f:
            channels = len(next(csv.reader(f)))
        result["files"].append({"name": path.name, "sha256": files.digest(path), "channels": channels,
                                "reason": "Height/roll layout has no measured sagittal hip torque."})
    result["training_prior_nm_s_rad"] = [0, .25, .75, 1.5]
    result["prior_source"] = "Existing sensitivity grid, not a VOFA fit."
    files.write_json(out/"identification.json", result)


class HipTuning(gym.Env):
    """One-step episode: a policy action is a fixed Q/R multiplier vector."""
    observation_space = gym.spaces.Box(-1, 1, shape=(1,), dtype=np.float32)
    action_space = gym.spaces.Box(-1, 1, shape=(len(AXES),), dtype=np.float32)

    def __init__(self, data, out):
        super().__init__()
        self.data, self.out = data, out
        self.baseline = np.r_[data["q"], data["r"]]
        self.models = [(np.array(p["A"]), np.array(p["B"]), np.array(p["xb"]), np.array(p["u0"]))
                       for p in data["models"]]
        self.dt = data["dt"]
        self.t = np.arange(0, 3, self.dt)
        self.ramp = np.clip(np.minimum((self.t-.15)/.30, (1.67-self.t)/.30), 0, 1)
        self.baseref = np.zeros(14)
        self.training = [(0, .25, .33, .09, 1), (1, .75, -.42, -.12, 1),
                         (2, 1.5, .35, .10, 1), (1, 0, -.35, .10, 1)]
        self.validation = [(0, .48, -.37, -.11, .95), (2, 1.12, .42, .08, .92),
                           (1, 0, .30, -.13, .96), (1, 1.5, -.40, .12, .93)]
        self.train_base = self.metrics(self.baseline, self.training)
        self.test_base = self.metrics(self.baseline, self.validation)
        self.rows = []

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        return np.zeros(1, dtype=np.float32), {}

    def weights(self, action):
        w = self.baseline.copy()
        for z, axes in zip(action, AXES, strict=True):
            w[list(axes)] *= math.exp(math.log(4)*float(z))
        return w

    def metrics(self, w, suite):
        result = []
        cache = {}
        for index, damping, speed, parked_offset, wheel_gain in suite:
            A, B, _, u0 = self.models[index]
            if index not in cache:
                Q, R = np.diag(w[:14]+self.data["rho"]), np.diag(w[14:]+self.data["rho"])
                P = solve_discrete_are(A, B, Q, R)
                cache[index] = np.linalg.solve(R+B.T@P@B, B.T@P@A)
            K = cache[index]
            nominal = B.copy()
            nominal[:, :2] *= wheel_gain
            plant = A + damping*nominal[:, 2:4]@DAMP
            for parked in (True, False):
                result.append(self.rollout(plant, nominal, K, u0, speed, parked_offset, parked))
        return result

    def rollout(self, A, B, K, u0, speed, offset, parked):
        x = np.zeros(14)
        x[0] = offset if parked else 0
        ref = self.baseref.copy()
        obs = x.copy()
        states = np.empty((len(self.t), 3))
        peak = 0.
        lo = np.asarray(self.data["umin"])-u0
        hi = np.asarray(self.data["umax"])-u0
        rate = 0
        for k, ramp in enumerate(self.ramp):
            if k == int(.6/self.dt):
                x[9] += .13*np.sign(offset)
            obs[:] = x
            obs[0] = .5*x[0] if parked else 0.
            ref[1] = 0 if parked else speed*ramp
            u = -K@(obs-ref)
            rate += np.any((u < lo) | (u > hi))
            x = A@x+B@np.clip(u,lo,hi)
            peak = max(peak, np.max(np.abs(x[[4,6,8,10]])))
            if not np.isfinite(x).all() or peak > .4 or abs(x[0])>2:
                return [10, 10, 3, max(peak,.4), 1.]
            states[k] = x[0], x[1], x[1]-ref[1]
        if parked:
            outside = (abs(states[:,0])>.015) | (abs(states[:,1])>.06)
            start = 0.
        else:
            outside = (self.t>=1.67) & (abs(states[:,1])>.06)
            start = 1.67
        settle = min(3.,self.t[np.flatnonzero(outside)[-1]]+self.dt)-start if outside.any() else 0.
        e = np.sqrt(np.mean(states[:,2]**2))
        pos = np.sqrt(np.mean(states[:,0]**2)) if parked else 0.
        return [e,pos,settle,peak,rate/len(self.t)]

    def score(self, metrics, nominal):
        a,b = np.asarray(metrics),np.asarray(nominal)
        regress = np.maximum(0,(a[:,3]-1.05*b[:,3]-.003)/.03)
        regress += np.maximum(0,(a[:,4]-b[:,4]-.05)/.10)
        raw = a @ np.array([4, 7, .38, 7, 2])
        return float((.6*raw.mean()+.4*raw.max())*(1+regress.max())**2)

    def step(self, action):
        w = self.weights(action)
        try:
            value = self.score(self.metrics(w,self.training),self.train_base)
        except (ValueError,np.linalg.LinAlgError):
            value = 1e4
        self.rows.append((value,w.tolist()))
        if len(self.rows)%32 == 0:
            print(f"trials={len(self.rows)} best_training={min(v for v,_ in self.rows):.6f}",flush=True)
        return np.zeros(1,dtype=np.float32), -min(value,50), True, False, {}


def read_model():
    path=OUT/"model.json"
    if not path.is_file():
        OUT.mkdir(parents=True,exist_ok=True)
        template=files.HERE/"matlab/export_ppo.m.txt"
        (OUT/"export_ppo.m").write_bytes(template.read_bytes())
        mix=files.HERE/"build/hip_parking_old/src/Matlab/mixmpc_pre"
        cmd=f"mixmpc_paths();addpath({files.mq(OUT)});export_ppo({files.mq(OUT)});"
        code=files.matlab(SimpleNamespace(matlab="matlab"),mix,cmd,OUT/"export.log")
        if code:
            raise RuntimeError(f"Model export returned {code}")
    return files.read_json(path)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps",type=int,default=192)
    args=parser.parse_args()
    torch.set_num_threads(1)
    data=read_model()
    audit_vofa(OUT)
    env=HipTuning(data,OUT)
    agent=PPO("MlpPolicy",env,n_steps=32,batch_size=32,n_epochs=8,
              learning_rate=.0015,gamma=0.,seed=19,device="cpu",
              policy_kwargs={"net_arch":[32,32],"log_std_init":-.9})
    agent.learn(total_timesteps=args.steps)
    agent.save(OUT/"policy")
    deterministic=env.weights(agent.predict(np.zeros(1,dtype=np.float32),deterministic=True)[0])
    trials=sorted(env.rows,key=lambda row:row[0])
    candidates=[("baseline",env.baseline),("ppo_mean",deterministic)]
    candidates += [(f"ppo_trial{k:02d}",np.array(w)) for k,(_,w) in enumerate(trials[:8],1)]
    rows=[]
    for name,w in candidates:
        train=env.metrics(w,env.training)
        test=env.metrics(w,env.validation)
        rows.append({"id":name,"Q_DIAG":w[:14].tolist(),"R_DIAG":w[14:].tolist(),
                     "train":env.score(train,env.train_base),"heldout":env.score(test,env.test_base),
                     "train_cases":train,"heldout_cases":test})
    best=min(rows[1:],key=lambda row:row["heldout"]+row["train"])
    files.write_json(OUT/"results.json",{"steps":len(trials),"rows":rows,"screened":best["id"]})
    files.write_json(OUT/"ppo_candidate.json",best)
    print(f"screened {best['id']}, train={best['train']:.6f}, holdout={best['heldout']:.6f}",flush=True)


if __name__=="__main__":
    main()

