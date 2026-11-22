"""Apply the existing numerical and firmware gates to the full Q/R search."""
import run_pso as r
from recheck_firmware import sweep
from select_rechecked import comparable_source


def response_times(result):
    cases=result["scenarios"]
    parked=[m for m in cases if m["conditions"][10] and not any(m["conditions"][7:9])]
    moving=[m for m in cases if m["conditions"][7]]
    return [sum(m[k] for m in group)/len(group) for k,group in
            (("position_settle_s",parked),("speed_rise_lag_s",moving),("speed_stop_s",moving))]


def response_ratio(candidate,base):
    times=response_times(candidate)
    initial=response_times(base)
    return [x/max(b,.02) for x,b in zip(times,initial,strict=True)]


def improves(candidate,base):
    if not candidate["feasible"] or candidate["score"] > 1.05*base["score"]:
        return False
    if any(m["attitude_peak"] > 1.05*b["attitude_peak"]+.003
           or m["roll_peak"] > 1.05*b["roll_peak"]+.003
           or m["fallback"] > b["fallback"]+.05
           for m,b in zip(candidate["scenarios"],base["scenarios"],strict=True)):
        return False
    times=response_times(candidate)
    initial=response_times(base)
    ratios=response_ratio(candidate,base)
    return (sum(ratios)/3 < 1 and rank(candidate,base)<1 and
            all(x<=1.05*b+.02 for x,b in zip(times,initial,strict=True)))


def rank(candidate,base):
    if not candidate["feasible"]:
        return float("inf")
    return .35*candidate["score"]/base["score"]+.65*sum(response_ratio(candidate,base))/3

def holdout(args,out,mix,batch,index):
    picks=out/"shortlist.json"
    r.write_json(picks,batch)
    cmd=f"mixmpc_paths();addpath({r.mq(out/'matlab')});pso_full({r.mq(out)},'holdout',{r.mq(picks)});"
    code=r.matlab(args,mix,cmd,out/f"logs/holdout_{index:02d}.log")
    if code:
        raise RuntimeError(f"Holdout returned {code}")
    return r.read_json(out/"holdout/baseline.json")


def check_candidate(args,out,mix,firmware,item,row):
    name=item["id"]
    pkg=out/"packages"/name
    logs=out/"logs"
    config=mix/"tinympc14_config.m"
    original=(out/"baseline.m.txt").read_bytes()
    r.write_config(original.decode("utf-8-sig"),config,item)
    row["fast"]=r.matlab(args,mix,f"build_mixmpc_pre_fast({r.mq(pkg/'fast')});",logs/f"{name}_fast.log")
    if row["fast"]:
        raw=(logs/f"{name}_fast.log").read_bytes()
        if not any(marker in raw for marker in (
            b"qr_build:CacheStale",b"qr_build:CacheMissing",b"qr_build:CacheCorrupt")):
            return
        config.write_bytes(original)
        base=out/"packages/baseline_cache_rebuild"
        row["cache_rebuild"]=r.matlab(args,mix,f"build_mixmpc_pre({r.mq(base)});",logs/"cache_rebuild.log")
        if row["cache_rebuild"]:
            return
        r.write_config(original.decode("utf-8-sig"),config,item)
        row["fast"]=r.matlab(args,mix,f"build_mixmpc_pre_fast({r.mq(pkg/'fast')});",logs/f"{name}_fast.log")
        if row["fast"]:
            return
    r.install_package(pkg/"fast",firmware)
    row["closed"]=r.validate_c(firmware,out,name)
    if row["closed"]:
        return
    if not check_sweep(out,pkg/"fast",name,row):
        return
    row["full"]=r.matlab(args,mix,f"build_mixmpc_pre({r.mq(pkg/'full')});",logs/f"{name}_full.log")
    if row["full"]:
        return
    r.install_package(pkg/"full",firmware)
    if any(comparable_source(pkg/"fast"/f)!=comparable_source(pkg/"full"/f) for f in r.MODEL_FILES):
        row["closed_full"]=r.validate_c(firmware,out,name+"_full")
        if row["closed_full"] or not check_sweep(out,pkg/"full",name+"_full",row):
            return
    row["arm"]=r.validate_arm(args,firmware,out,name)


def check_sweep(out,package,name,row):
    row["sweep"]=sweep(out,out/"c_grid",package,name)
    raw=(out/"logs"/f"{name}_sweep.log").read_bytes()
    row["sweep_passes"]=raw.count(b"LIMITS PASS")
    row["sweep_failures"]=raw.count(b"LIMITS FAIL")
    return row["sweep"]==0 and row["sweep_passes"]==54 and row["sweep_failures"]==0


def validate(args,out):
    mix=out/"src/Matlab/mixmpc_pre"
    firmware=out/"src/Wheel_Leg_Code"
    config=mix/"tinympc14_config.m"
    original=(out/"baseline.m.txt").read_bytes()
    points=sorted((r.read_json(p) for p in (out/"candidates").glob("*.json")),
                  key=lambda p:(p["score"] if p["score"] is not None else float("inf"),p["id"]))
    base=next(p for p in points if p["id"]=="baseline")
    unique={}
    for item in points:
        if improves(item,base):
            unique.setdefault(tuple(item["Q_DIAG"]+item["R_DIAG"]),item)
    candidates=list(unique.values())
    r.write_json(out/"search_rank.json",[
        {"id":p["id"],"score":p["score"],"eligible":improves(p,base)} for p in points])
    grid=out/"c_grid"
    if not grid.exists():
        for sub in ("Code","tests"):
            r.copy_tree(firmware/sub,grid/sub)
    rows=[]
    state={"selected":None,"search_positions":len(points),"search_eligible":len(candidates),
           "results":rows,"error":None}
    try:
        # Evaluate independent holdout for every training-eligible point so both
        # suites participate in a single global ranking of tested candidates.
        for start in range(0,max(1,len(candidates)),20):
            batch=candidates[start:start+20]
            config.write_bytes(original)
            b=holdout(args,out,mix,batch,start//20+1)
            if not b["feasible"]:
                raise RuntimeError("Current production baseline failed the holdout plant")
            for item in batch:
                held=r.read_json(out/"holdout"/f"{item['id']}.json")
                rows.append({"id":item["id"],"train":item["score"],"holdout":held["score"],
                             "holdout_improved":improves(held,b),
                             "rank":.5*(rank(item,base)+rank(held,b))})
            r.write_json(out/"validation.json",state)
        rows.sort(key=lambda x:x["rank"])
        lookup={p["id"]:p for p in candidates}
        for row in rows:
            if not row["holdout_improved"]:
                continue
            item=lookup[row["id"]]
            try:
                check_candidate(args,out,mix,firmware,item,row)
            except (OSError,ValueError) as exc:
                row["error"]=str(exc)
            if row.get("arm")==0:
                state["selected"]=row["id"]
                return item
            r.write_json(out/"validation.json",state)
    except Exception as exc:
        state["error"]=str(exc)
        raise
    finally:
        config.write_bytes(original)
        r.write_json(out/"validation.json",state)
    return None



