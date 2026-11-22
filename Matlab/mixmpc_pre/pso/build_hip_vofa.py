"""Build an isolated image that captures parking and hip torque in one VOFA frame."""
from types import SimpleNamespace

import run_pso as r


def main():
    archive = r.HERE / "build/full20"
    work = r.HERE / "build/hip_vofa_old_id"
    firmware = work / "src/Wheel_Leg_Code"
    if not firmware.exists():
        for part in r.FIRMWARE_PARTS:
            r.copy_tree(archive / "src/Wheel_Leg_Code" / part, firmware / part)
    r.install_package(archive / "packages/response_r1_p07/full", firmware)
    task = firmware / "Code/Task/vofa_task.c"
    template = (r.HERE / "hip_vofa.c.txt").read_text(encoding="utf-8-sig")
    task.write_text(template, encoding="utf-8")
    code = r.validate_arm(SimpleNamespace(armcc=r.ARMCC), firmware, work, "hip_vofa")
    if code:
        return code
    release = work / "packages/hip_vofa/arm"
    old = r.read_json(archive / "packages/response_r1_p07/arm/build_manifest.json")
    new = r.read_json(release / "build_manifest.json")
    before = old["original_code_sha256"]
    after = new["original_code_sha256"]
    changed = sorted(name for name in before.keys() | after.keys() if before.get(name) != after.get(name))
    if changed != ["Code/Task/vofa_task.c"]:
        raise ValueError(f"Unexpected differences from the archived firmware: {changed}")
    r.write_json(work / "verification.json", {
        "baseline": "response_r1_p07", "changed_sources": changed, "frame": 5402,
        "channels": 54, "hex_sha256": r.digest(release / "Wheel_Leg_Code.hex"),
    })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


