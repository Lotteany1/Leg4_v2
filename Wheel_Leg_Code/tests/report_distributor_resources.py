"""Extract linked AC5 size and known static task-stack figures for this integration."""
import json
from pathlib import Path
import re
ROOT = Path(__file__).resolve().parents[1]

def collect(directory):
    base = ROOT / "build" / directory
    data = (base / "project.map").read_text(errors="replace")
    graph = (base / "Wheel_Leg_Code.htm").read_text(errors="replace")
    sizes = {}
    for label in ("RO", "RW", "ROM"):
        sizes[label] = int(re.search(r"Total " + label + r"\s+Size[^\n]*?\)\s+(\d+)", data)[1])
    for name in ("StartControlTask", "StartTinympc14Task", "StartVofaTask"):
        match = re.search(re.escape(name)+r"</STRONG>.*?Max Depth = (\d+)", graph, re.S)
        if match: sizes[name + "_known_stack"] = int(match[1])
    history = re.search(r"execution_history\s+0x[0-9a-f]+\s+Data\s+(\d+)",data)
    if history: sizes["execution_history_bytes"] = int(history[1])
    return sizes

if __name__ == "__main__":
    profiles = {name: collect(directory) for name, directory in (("baseline", "project_check_baseline"),
        ("default", "project_check"), ("diagnostics", "project_check_diagnostics"))}
    profiles["delta_default"] = {key: profiles["default"][key]-profiles["baseline"][key] for key in ("RO", "RW", "ROM")}
    output = ROOT / "build/distributor_check/resources.json"
    output.write_text(json.dumps(profiles,indent=2))
    print(json.dumps(profiles,indent=2))
