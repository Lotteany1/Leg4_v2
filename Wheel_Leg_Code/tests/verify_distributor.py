"""Compare production C allocator and VMC with independent float64 constrained optimization."""
import sys
sys.dont_write_bytecode = True
import ctypes
import json
import re
from pathlib import Path
import subprocess
import sys
import numpy as np
from scipy.optimize import minimize, LinearConstraint, nnls
from run_distributor import ROOT, OUT, CC, environment
from distributor_evidence import begin_evidence, finish_evidence
sys.path.insert(0, str(ROOT.parent / "research/actuator_coordination_20261101_01"))
from mapping_model import ik, sent_map

def main():
    dll = OUT / "distributor_reference.dll"
    command = [CC, "-std=c99", "-O3", "-Wall", "-Wextra", "-Werror", "-shared", "-include", "tests/stubs/ac5_host.h", "-Itests/distributor_stubs", "-Itests/stubs",
        *["-I" + str(p) for p in (ROOT / "Code").iterdir() if p.is_dir()],
        "-Wno-unused-parameter", "-Wno-tautological-constant-out-of-range-compare",
        "tests/distributor_reference.c", "tests/distributor_runtime.c", "Code/Control/distributor.c",
        "Code/Driver/dm_motor.c", "Code/Driver/dji_motor.c", "Code/Math/Leg_Trq.c", "-o", str(dll),
        "-Wl,/EXPORT:Test_Distributor", "-Wl,/EXPORT:Test_Distributor_Parameters", "-Wl,/EXPORT:Leg_Trq"]
    evidence = begin_evidence(command, extra_inputs=[ROOT.parent / 'research/actuator_coordination_20261101_01/mapping_model.py'])
    subprocess.run(command, cwd=ROOT, env=environment(), check=True)
    lib = ctypes.CDLL(str(dll))
    ptr = ctypes.POINTER(ctypes.c_float)
    lib.Test_Distributor.argtypes = [ptr, ctypes.c_int, ptr]
    lib.Leg_Trq.argtypes = [ctypes.c_float]*4 + [ptr]
    parameters = np.zeros(7, dtype=np.float32)
    lib.Test_Distributor_Parameters.argtypes = [ptr]
    lib.Test_Distributor_Parameters(parameters.ctypes.data_as(ptr))
    scale, weights = parameters[:2].astype(float), parameters[2:4].astype(float)
    support_min, support_max, reserve = parameters[4:].astype(float)
    force_header = (ROOT / 'Code/Model/tinympc14_data.h').read_text(encoding='utf-8')
    force_box = [float(re.search(r'#define TINYMPC14_F_' + name + r'\s+([\d.eE+-]+)f', force_header)[1])
                 for name in ('MIN', 'MAX')]
    assert force_box[0] <= support_min <= support_max <= force_box[1], 'NORMAL soft support range exceeds planner box'
    reserve = np.clip(reserve, support_min, support_max)
    def call(data, strategy):
        data = np.asarray(data, dtype=np.float32); out = np.zeros(8, dtype=np.float32)
        lib.Test_Distributor(data.ctypes.data_as(ptr), strategy, out.ctypes.data_as(ptr))
        return out
    rng = np.random.default_rng(20261101)
    maximum = mapping = production_kkt = 0.0
    statuses = {}
    boundary_counts = {0:0, 1:0, 2:0}
    support_active = support_relaxed = comparison_feasible = 0
    for n in range(1200):
        length = rng.uniform(.16,.34); q = ik(length, rng.uniform(1.2,1.9)); right = n % 2 != 0
        a = sent_map(q,right)
        generated = []
        for f,p in ((1,0),(0,-1 if right else 1)):
            out = np.zeros(2,dtype=np.float32)
            lib.Leg_Trq(f,p,*q,out.ctypes.data_as(ptr))
            generated.append(out[::-1] if right else out)
        actual = np.column_stack(generated)
        mapping = max(mapping,float(np.max(np.abs(actual-a))))
        assert np.allclose(actual,a,rtol=2e-5,atol=2e-6)
        request = rng.uniform([-300,-180],[1000,180]); spring = rng.uniform(0,200); cp = rng.uniform(-20,20)
        lower = rng.uniform(-54,-.1,2); upper = rng.uniform(.1,54,2)
        # The reference uses precisely the float32 problem submitted to C.
        data = np.float32([*actual.ravel(),*request,spring,cp,*lower,*upper])
        a = data[:4].astype(float).reshape(2,2); request=data[4:6].astype(float)
        offset=np.array([-data[6],data[7]],float); lower=data[8:10].astype(float); upper=data[10:12].astype(float)
        # Independently enumerate all four motor-box vertices to find reachable support.
        vertices = np.array([np.linalg.solve(a, [t0, t1])-offset
                             for t0 in (lower[0], upper[0]) for t1 in (lower[1], upper[1])])
        support_lo, support_hi = np.clip([min(request[0], reserve), max(request[0], support_max)],
                                         vertices[:,0].min(), vertices[:,0].max())
        support_relaxed += int(min(request[0], reserve) > vertices[:,0].max() or
                               max(request[0], support_max) < vertices[:,0].min())
        g=np.vstack((a*scale, [scale[0], 0.])); bias=a@offset; target=request/scale
        constraint_lo=np.r_[lower-bias, support_lo]
        constraint_hi=np.r_[upper-bias, support_hi]
        clipped=np.clip(a@(request+offset),lower,upper); start=np.linalg.solve(a,clipped)/scale-offset/scale
        constraints=[LinearConstraint(g[:2],constraint_lo[:2],constraint_hi[:2]),
                     LinearConstraint(g[2:],constraint_lo[2:],constraint_hi[2:])]
        ref=minimize(lambda x:.5*np.sum(weights*(x-target)**2),start,jac=lambda x:weights*(x-target),
            constraints=constraints,method="SLSQP",options={"ftol":1e-12,"maxiter":100})
        # SLSQP sometimes reports a line-search limit at an already optimal vertex.
        # Check primal feasibility and independent convex KKT conditions instead.
        residual = g@ref.x
        assert np.max(np.maximum(constraint_lo-residual,residual-constraint_hi)) < 1e-6
        active = np.vstack((g[np.abs(residual-constraint_hi)<1e-6],-g[np.abs(residual-constraint_lo)<1e-6]))
        gradient = weights*(ref.x-target)
        if len(active):
            _, kkt = nnls(active.T,-gradient)
            assert kkt < 1e-6,(ref.message,kkt)
        else: assert np.linalg.norm(gradient) < 1e-6
        out=call(data,0); statuses[int(out[1])]=statuses.get(int(out[1]),0)+1
        assert out[0] == 1 and out[1] != 4 and out[7] == 0
        active = int(np.count_nonzero(np.minimum(abs(out[4:6]-lower),abs(out[4:6]-upper))<1e-4))
        boundary_counts[active] += 1
        error=abs(float(out[6])-ref.fun); maximum=max(maximum,error)
        assert error <= 5e-5*(1+ref.fun),(n,out,ref.fun)
        assert np.all(out[4:6]>=lower) and np.all(out[4:6]<=upper)
        assert support_lo-2e-4 <= out[2] <= support_hi+2e-4, (n, out, support_lo, support_hi)
        support_active += int(min(abs(out[2]-support_lo), abs(out[2]-support_hi)) < 2e-4)
        # Check stationarity at the production point as well as the SLSQP oracle.
        production_gradient = weights*(out[2:4].astype(float) / scale - target)
        production_residual = np.r_[out[4:6].astype(float)-bias, float(out[2])]
        active_tolerance = np.array([1e-4, 1e-4, 2e-4])
        production_active = np.vstack((g[np.abs(production_residual-constraint_hi)<active_tolerance],
                                       -g[np.abs(production_residual-constraint_lo)<active_tolerance]))
        if len(production_active):
            _, residual_kkt = nnls(production_active.T, -production_gradient)
        else:
            residual_kkt = np.linalg.norm(production_gradient)
        production_kkt = max(production_kkt, float(residual_kkt))
        assert residual_kkt <= 5e-5*(1+np.linalg.norm(production_gradient)), (n, residual_kkt)
        for strategy in (1,2):
            alt=call(data,strategy)
            assert alt[0]==1
            # The alternative is a valid objective upper bound only inside the new support domain.
            if support_lo-2e-4 <= alt[2] <= support_hi+2e-4:
                comparison_feasible += 1
                assert out[6] <= alt[6]+5e-5*(1+alt[6])
    invalid=np.zeros(12,dtype=np.float32); assert call(invalid,0)[0] == 0
    assert support_active > 0 and support_relaxed > 0 and comparison_feasible > 0
    report={"cases":1200,"max_objective_error":maximum,"max_mapping_error":mapping,"max_production_kkt_residual":production_kkt,"statuses":statuses,"active_boundary_counts":boundary_counts,
            "support_active":support_active,"support_relaxed":support_relaxed,"feasible_alternative_comparisons":comparison_feasible}
    report['seed'] = 20261101
    report['compiled_parameters'] = dict(scales=scale.tolist(), weights=weights.tolist(),
                                         support_min=support_min, support_max=support_max, support_reserve=reserve)
    report['evidence'] = finish_evidence(evidence, [dll])
    (OUT/"allocator_reference.json").write_text(json.dumps(report,indent=2))
    print("PASS: independent float64 SLSQP, mirrors, geometry, asymmetric limits, compensation, clip/scale comparison",
          {key: value for key, value in report.items() if key != 'evidence'})
if __name__ == "__main__": main()
