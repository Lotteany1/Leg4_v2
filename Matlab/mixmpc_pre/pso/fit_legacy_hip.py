"""Attempt aggregate sagittal damping identification from legacy VOFA captures."""
import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.linalg import logm, null_space
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

import run_pso as r

OUT = r.HERE/"build/hip_ppo"
SOURCES = {
    "capture_20261029_new.csv": "2901023a",
    "capture_16d00271.csv": "16d00271",
    "capture_26262de3.csv": "26262de3",
    "capture_ac2efca4.csv": "ac2efca4",
    "capture_dcb48d69_segment2.csv": "e6af2612",
}


def matrices(data):
    point = data["models"][0]
    aug = np.eye(20)
    aug[:14, :14], aug[:14, 14:] = point["A"], point["B"]
    continuous = logm(aug).real/data["dt"]
    ix = [1, 5, 7, 9]
    A = continuous[ix, :14]
    B = continuous[ix, 14:]
    projection = null_space(B[:, :2].T).T
    assert np.linalg.norm(projection @ B[:, :2]) < 1e-10
    return projection @ A, projection @ B, np.asarray(point["xb"]), np.asarray(point["u0"]), projection


def observations(path, model):
    A, B, xb, u0, projection = model
    s = np.genfromtxt(path, delimiter=",", skip_header=1, invalid_raise=False)
    s = s[np.isfinite(s).all(axis=1) & (abs(s).max(axis=1)<1e8)]
    edges = np.where((np.diff(s[:,0])>20) | (np.diff(s[:,0])<=0))[0]+1
    x_all, y_all = [], []
    for chunk in np.split(s,edges):
        if len(chunk)<35:
            continue
        t = np.arange(chunk[0,0],chunk[-1,0],10.)*.001
        arr = np.column_stack([np.interp(t,chunk[:,0]*.001,chunk[:,i]) for i in range(52)])
        for smooth in (9,):
            x = np.zeros((len(t),14))
            x[:,[1,3,4,6,8,10,11,12,13]] = arr[:,[47,11,36,37,35,7,8,3,6]]
            x[:,[5,7,9]] = savgol_filter(x[:,[4,6,8]],smooth,3,deriv=1,delta=.01,axis=0)
            xt = savgol_filter(x,smooth,3,axis=0)
            u = np.zeros((len(t),6))
            u[:,2:] = savgol_filter(arr[:,[45,46,18,19]],smooth,3,axis=0)
            residual = (xt-xb)@A.T+(u-u0)[:,2:]@B[:,2:].T
            omega = x[:,[5,7]]-x[:,[9]]
            reg = omega@B[:,2:4].T
            # Separate pose-dependent gravity-compensation mismatch and offsets
            # from the rate-dependent residual in each projected coordinate.
            nuisance = np.stack([xt[:,4,None]*B[:,2],xt[:,6,None]*B[:,3],
                                  np.ones_like(reg)*[1,0],np.ones_like(reg)*[0,1]],axis=2)
            obs = xt[:,[1,5,7,9]] @ projection.T
            integ = cumulative_trapezoid(np.concatenate([residual[:,:,None],reg[:,:,None],nuisance],axis=2),
                                         dx=.01,axis=0,initial=0)
            for j in range(8,len(t)-20,12):
                stop=j+12
                win=arr[j:stop+1]
                if not (np.all(abs(win[:,29:31]-.21)<.045) and np.max(abs(win[:,29]-win[:,30]))<.035 and
                        np.max(abs(win[:,7]))<.14 and np.max(abs(win[:,11]))<.55 and np.max(abs(omega[j:stop]))>.15):
                    continue
                a=integ[stop]-integ[j]
                x_all.append(a[:,1:])
                y_all.append(obs[stop]-obs[j]-a[:,0])
    return np.concatenate(x_all) if x_all else np.empty((0,5)), np.concatenate(y_all) if y_all else np.empty(0)


def fit(x,y,enabled=True):
    return least_squares(lambda a:x@a-y,np.zeros(x.shape[1]),loss="soft_l1",f_scale=.08,
                         bounds=([0,-20,-20,-10,-10],[10,20,20,10,10])).x if enabled else np.r_[0,fit_nuisance(x[:,1:],y)]


def fit_nuisance(x,y):
    return least_squares(lambda a:x@a-y,np.zeros(x.shape[1]),loss="soft_l1",f_scale=.08,
                         bounds=([-20,-20,-10,-10],[20,20,10,10])).x


def main():
    model=matrices(r.read_json(OUT/"model.json"))
    source=r.ROOT/"Wheel_Leg_Code/diagnostics"
    data={}
    for name,expected in SOURCES.items():
        path=source/name
        assert r.digest(path).startswith(expected),name
        x,y=observations(path,model)
        if len(y)>=30:
            data[name]=(x,y)
    results={}
    for name,(x,y) in data.items():
        train=[v for k,v in data.items() if k!=name]
        if not train:
            break
        X,Y=map(np.concatenate,zip(*train,strict=True))
        p=fit(X,Y); zero=fit(X,Y,False)
        null=float(np.sqrt(np.mean((x@zero-y)**2)))
        pred=float(np.sqrt(np.mean((x@p-y)**2)))
        results[name]={"windows":len(y)//2,"estimated_nm_s_rad":float(p[0]),
                       "zero_rmse":null,"fit_rmse":pred}
    recommend=bool(len(results)>=3 and all(q["fit_rmse"]<.95*q["zero_rmse"] for q in results.values()) and
                   min(q["estimated_nm_s_rad"] for q in results.values())>.02 and
                   max(q["estimated_nm_s_rad"] for q in results.values())<5)
    r.write_json(OUT/"legacy_fit.json",{"legacy_estimate_pass":recommend,"leave_one_record_out":results,
                  "source_note":"2026-10-29 actual traces; wheel input projected out; model and source revision differ."})
    print(results, "PASS" if recommend else "UNRELIABLE",flush=True)


if __name__=="__main__":
    main()

