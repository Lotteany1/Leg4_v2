"""Verify the source template and the layout of the deployed TinyMPC14 package."""
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    model = root / "Code/Model"
    template = root.parent / "Matlab/mixmpc_pre/deploy/solver"
    files = {"tinympc14.c", "tinympc14.h", "tinympc14_data.c", "tinympc14_data.h"}
    assert {p.relative_to(model).as_posix() for p in model.rglob('*') if p.suffix in ('.c', '.h')} == files
    assert (model / "tinympc14.h").read_bytes() == (template / "tinympc14.h").read_bytes()
    deployed = (model / "tinympc14.c").read_text(encoding="utf-8")
    assert deployed.startswith((template / "tinympc14.c").read_text(encoding="utf-8"))
    assert "void tinympc14_online_model(" in deployed
    print("PASS: deployed TinyMPC14 solver and data pairs")


if __name__ == "__main__":
    main()
