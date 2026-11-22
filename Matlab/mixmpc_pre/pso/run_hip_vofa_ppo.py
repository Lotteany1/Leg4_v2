"""Run robust PPO Q/R screening on the timestamped hip VOFA experiment."""
import argparse
import math
import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import gymnasium as gym
import numpy as np
from scipy.linalg import solve_discrete_are
from stable_baselines3 import PPO
import torch

from fit_hip_vofa import OUT
import run_pso as files


AXES = ((0,), (1,), (2,), (3,), (4, 6), (5, 7), (8,), (9,),
        (10,), (11,), (12,), (13,), (14, 15), (16, 17), (18, 19))
METRICS = ("speed_rms", "rise_s", "stop_s", "late_speed_rms", "park_span_m",
           "attitude_rad", "saturation", "diverged")
SCALE = np.array([.25, .30, .65, .10, .06, .15, .10, 1.])
COST = np.array([.20, .13, .24, .19, .12, .10, .03, 8.])
MID_MARGIN = np.array([.005, .020, .025, .008, .008, .004, .020, 0.])
DAMP = np.zeros((2, 14))
DAMP[0, 5] = DAMP[1, 7] = 1.
DAMP[:, 9] = -1.


def scenarios(holdout):
    if holdout:
        low = [(.17, 0., -.51, 0., 2), (.18, .70, .54, 0., 5),
               (.19, 2.20, -.81, 0., 3), (.17, 5.25, .69, 0., 6),
               (.18, 1.82, 0., -.14, 4), (.19, 4.60, 0., .11, 5)]
        mid = [(.24, 0., .52, 0., 2), (.25, 1.70, -.74, 0., 4),
               (.26, 4.50, .67, 0., 5), (.25, .80, 0., -.12, 4)]
    else:
        low = [(.17, 0., .62, 0., 1), (.18, 1.45, -.66, 0., 3),
               (.19, 3.20, .82, 0., 4), (.17, 5.50, -.78, 0., 5),
               (.18, .75, 0., .12, 2), (.19, 2.0, 0., -.09, 4)]
        mid = [(.24, .60, -.62, 0., 2), (.25, 2.0, .72, 0., 3),
               (.26, 5.0, -.77, 0., 5), (.24, 0., 0., .09, 3)]
    return [dict(zip(("height", "damping", "reference", "offset", "delay_steps"), row, strict=True))
            for row in low + mid]


class Suite:
    def __init__(self, data, cases):
        self.cases = cases
        model = {m["height"]: m for m in data["models"]}
        self.a = np.array([model[c["height"]]["A"] for c in cases])
        self.b = np.array([model[c["height"]]["B"] for c in cases])
        self.u0 = np.array([model[c["height"]]["u0"] for c in cases])
        self.dt = data["dt"]
        self.rho = data["rho"]
        self.plant = self.a - np.array([c["damping"] for c in cases])[:, None, None] * (
            self.b[:, :, 2:4] @ DAMP)
        self.speed_ref = np.array([c["reference"] for c in cases])
        self.offset = np.array([c["offset"] for c in cases])
        self.delay = np.array([c["delay_steps"] for c in cases])
        self.lo, self.hi = np.asarray(data["umin"]) - self.u0, np.asarray(data["umax"]) - self.u0
        self.mid = np.array([c["height"] >= .24 for c in cases])

    def evaluate(self, w):
        # Group two 2 ms model steps so the Riccati screen acts on a 4 ms
        # controller update, independent of the nonlinear MPC verification.
        a, b = self.a @ self.a, self.a @ self.b + self.b
        plant, bp = self.plant @ self.plant, self.plant @ self.b + self.b
        dt = 2 * self.dt
        q, r = np.diag(w[:14] + self.rho), np.diag(w[14:] + self.rho)
        gains = {}
        for i, case in enumerate(self.cases):
            h = case["height"]
            if h not in gains:
                p = solve_discrete_are(a[i], b[i], q, r)
                gains[h] = np.linalg.solve(r + b[i].T @ p @ b[i], b[i].T @ p @ a[i])
        k_gain = np.stack([gains[c["height"]] for c in self.cases])
        n = len(self.cases)
        x = np.zeros((n, 14))
        x[:, 0] = self.offset
        drive = self.speed_ref != 0
        lock = ~drive
        park_pos = np.zeros(n)
        weight = np.where(lock, .5, 0.)
        requested = np.zeros_like(x)
        queue = np.zeros((self.delay.max() + 1, n, 6))
        speed_errors = []
        late_speed = []
        late_pos = []
        rise = np.full(n, 1.4)
        stop = np.zeros(n)
        attitude = np.zeros(n)
        clipped = np.zeros(n)
        failed = np.zeros(n, dtype=bool)
        t = np.arange(0, 3.2, dt)
        for step, seconds in enumerate(t):
            ramp = np.clip(min((seconds - .16) / .28, (1.52 - seconds) / .28), 0., 1.)
            vref = self.speed_ref * ramp
            if step == round(.60 / dt):
                x[:, 9] += .10 * np.sign(np.where(drive, self.speed_ref, self.offset))
            if seconds >= 1.52:
                capture = drive & ~lock & (abs(x[:, 1]) <= .10) & (abs(x[:, [4, 6]]).max(axis=1) < .30)
                park_pos[capture] = x[capture, 0]
                lock |= capture
                weight = np.where(lock, np.minimum(.5, weight + dt * .5 / .25), 0.)
            requested[:] = x
            requested[:, 0] = weight * (x[:, 0] - park_pos)
            requested[:, 1] -= vref
            u = -(k_gain @ requested[:, :, None])[:, :, 0]
            clipped += np.any((u < self.lo) | (u > self.hi), axis=1)
            queue[step % len(queue)] = np.clip(u, self.lo, self.hi)
            applied = queue[(step - self.delay) % len(queue), np.arange(n)]
            x = (plant @ x[:, :, None] + bp @ applied[:, :, None])[:, :, 0]
            peak = abs(x[:, [4, 6, 8, 10]]).max(axis=1)
            failed |= (~np.isfinite(x).all(axis=1)) | (peak > .45) | (abs(x[:, 0]) > 3.)
            x[failed] = 0.
            attitude = np.maximum(attitude, peak)
            speed_errors.append(x[:, 1] - vref)
            cross = drive & (rise == 1.4) & (x[:, 1] * np.sign(self.speed_ref) >= .5 * abs(self.speed_ref))
            rise[cross] = max(0., seconds - .30)
            if seconds >= 1.52:
                bad = (abs(x[:, 1]) > .06) | ~lock
                stop[bad] = seconds - 1.52 + dt
            if seconds >= 2.65:
                late_speed.append(x[:, 1].copy())
                late_pos.append(x[:, 0].copy())
        rms = lambda x: np.sqrt(np.mean(np.square(x), axis=0))
        values = np.column_stack((rms(speed_errors), rise * drive, stop, rms(late_speed),
                                  np.ptp(late_pos, axis=0), attitude, clipped / len(t), failed))
        values[failed, :7] = [4., 1.4, 1.68, 4., 3., .45, 1.]
        return values

    def score(self, metrics, base):
        difference = metrics - base
        low = metrics[~self.mid] / SCALE @ COST
        mid_penalty = np.maximum((difference[self.mid] - MID_MARGIN) / SCALE, 0)
        attitude_penalty = np.maximum((difference[:, 5] - .006) / .04, 0).max()
        return float((.65 * low.mean() + .35 * low.max())
                     * (1 + 4 * mid_penalty.max() + 4 * attitude_penalty) ** 2)


class Tuning(gym.Env):
    observation_space = gym.spaces.Box(-1, 1, shape=(1,), dtype=np.float32)
    action_space = gym.spaces.Box(-1, 1, shape=(len(AXES),), dtype=np.float32)

    def __init__(self, suite, weights, radius):
        self.suite = suite
        self.base = weights
        self.log_radius = math.log(radius)
        self.metrics = suite.evaluate(weights)
        self.trials = []

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        return np.zeros(1, dtype=np.float32), {}

    def weights(self, action):
        w = self.base.copy()
        for axes, move in zip(AXES, action, strict=True):
            w[list(axes)] *= math.exp(self.log_radius * float(move))
        return w

    def step(self, action):
        w = self.weights(action)
        try:
            score = self.suite.score(self.suite.evaluate(w), self.metrics)
        except (ValueError, np.linalg.LinAlgError):
            score = 1e5
        self.trials.append((score, w))
        return np.zeros(1, dtype=np.float32), -min(score, 60.), True, False, {}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=256, help="PPO trials per seed")
    parser.add_argument("--radius", type=float, default=2.25, help="Largest multiplier of a Q/R axis")
    parser.add_argument("--tag", choices=("wide", "local"), default="wide")
    args = parser.parse_args()
    data = files.read_json(OUT / "model.json")
    train, test = (Suite(data, scenarios(held)) for held in (False, True))
    base = np.r_[data["q"], data["r"]]
    nominal = [s.evaluate(base) for s in (train, test)]
    weights = [("baseline", base)]
    torch.set_num_threads(1)
    for seed in (19, 59, 117):
        env = Tuning(train, base, args.radius)
        agent = PPO("MlpPolicy", env, n_steps=32, batch_size=32, n_epochs=6,
                    learning_rate=.002, gamma=0., seed=seed, device="cpu",
                    policy_kwargs={"net_arch": [32, 32], "log_std_init": -1.4})
        agent.learn(total_timesteps=args.steps)
        top = sorted(env.trials, key=lambda row: row[0])[:8]
        weights += [(f"s{seed}_trial{i:02d}", w) for i, (_, w) in enumerate(top, 1)]
        mean = agent.predict(np.zeros(1, dtype=np.float32), deterministic=True)[0]
        weights.append((f"s{seed}_mean", env.weights(mean)))
        agent.save(OUT / f"policy_{args.tag}_{seed}")
        print(f"seed={seed} trials={len(env.trials)} best_train={top[0][0]:.6f}", flush=True)
    rows = []
    for name, w in weights:
        results = [s.evaluate(w) for s in (train, test)]
        if args.tag == "local" and name != "baseline":
            name = "local_" + name
        rows.append({"id": name, "Q_DIAG": w[:14].tolist(), "R_DIAG": w[14:].tolist(),
                     "train": train.score(results[0], nominal[0]), "heldout": test.score(results[1], nominal[1]),
                     "train_mid_guard": bool(np.all((results[0] - nominal[0])[train.mid] <= MID_MARGIN)),
                     "heldout_mid_guard": bool(np.all((results[1] - nominal[1])[test.mid] <= MID_MARGIN)),
                     "train_cases": results[0].tolist(), "heldout_cases": results[1].tolist()})
    output = "ppo_results.json" if args.tag == "wide" else "ppo_local_results.json"
    files.write_json(OUT / output, {"metrics": METRICS, "mid_margins": MID_MARGIN.tolist(),
                     "train_cases": train.cases, "heldout_cases": test.cases, "rows": rows})


if __name__ == "__main__":
    main()
