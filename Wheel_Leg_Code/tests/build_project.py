"""Build and link every EIDE C source with the real AC5 toolchain."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
from distributor_evidence import begin_evidence, finish_evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--armcc', required=True)
    parser.add_argument('--output', type=Path, help='Isolated output directory relative to the firmware root')
    parser.add_argument('--baseline', action='store_true', help='Build HEAD C/header snapshot for a comparable resource baseline')
    parser.add_argument('--diagnostics', action='store_true', help='Enable allocation comparison in an isolated profile; select VOFA channels manually')
    parser.add_argument('--allocator', choices=('clip', 'project'), help='Select the NORMAL allocation strategy in an isolated profile')
    parser.add_argument('--step-check', choices=('disabled', 'enabled'), help='Override STEP_ENABLE in the isolated build')
    parser.add_argument('--spring-bench', choices=('right', 'off'), help='Isolate the right-leg calibration HEX or verify ordinary firmware')
    args = parser.parse_args()
    if args.step_check and args.baseline:
        parser.error('The single-side profiles require current sources')
    if args.spring_bench and args.baseline:
        parser.error('Spring bench firmware requires the current calibration sources')
    root = Path(__file__).resolve().parents[1]
    output = root / 'build/project_check'
    if args.output: output = root / args.output
    if args.baseline: output = output.with_name(output.name + '_baseline')
    if args.allocator: output = output.with_name(output.name + '_' + args.allocator)
    if args.diagnostics: output = output.with_name(output.name + '_diagnostics')
    if args.step_check: output = output.with_name(output.name + '_step_' + args.step_check)
    if args.spring_bench:
        output = (output if args.output else root / 'build') / ('spring_bench_' + args.spring_bench)
    output.mkdir(parents=True, exist_ok=True)
    tool = Path(args.armcc).parent
    config = (root / '.eide/eide.yml').read_text(encoding='utf-8')
    block = config.split('      incList:\n', 1)[1].split('      libList:', 1)[0]
    includes = ['-I' + line.strip()[2:] for line in block.splitlines() if line.strip().startswith('- ')]
    # Compile copied Code files so quoted includes cannot escape into a live Config directory.
    snapshot = output / ('snapshot_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    if args.baseline:
        tracked = subprocess.check_output(['git', 'ls-files', 'Code'], cwd=root, text=True).splitlines()
        code = {relative: subprocess.run(['git', 'show', 'HEAD:' + root.name + '/' + relative], cwd=root,
                                        capture_output=True, check=True).stdout
                for relative in tracked if relative.endswith(('.c', '.h'))}
    else:
        code = {str(path.relative_to(root)): path.read_bytes() for path in sorted((root / 'Code').rglob('*'))
                if path.suffix in ('.c', '.h')}
    for relative, original in code.items():
        target = snapshot / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(original)
    includes = ['-I' + str(snapshot / flag[2:]) if flag.startswith('-ICode') else flag for flag in includes]
    feature_path = snapshot / 'Code/Config/system_key.h'
    # HEAD may precede switch centralization when comparing the old-car baseline.
    config_paths = [feature_path]
    if args.baseline:
        config_paths += [path for path in sorted((snapshot / 'Code/Config').glob('*.h')) if path != feature_path]
        config_paths.append(snapshot / 'Code/Control/turn_limit.h')
    configs = {path: path.read_text(encoding='utf-8') for path in config_paths}

    def set_switch(key, value):
        matches = 0
        for path, text in configs.items():
            configs[path], count = re.subn(r'(^#define\s+' + key + r'\s+)\d+',
                                          r'\g<1>' + str(value), text, flags=re.M)
            matches += count
        if matches != 1:
            raise ValueError(f'{key}: expected one switch definition, found {matches}')

    if args.diagnostics:
        set_switch('BALANCE_DISTRIBUTOR_COMPARE', 1)
    if args.allocator:
        value = '1' if args.allocator == 'project' else '0'
        set_switch('BALANCE_USE_DISTRIBUTOR', value)
    if args.step_check:
        set_switch('STEP_ENABLE', int(args.step_check == 'enabled'))
    if args.spring_bench:
        set_switch('SPRING_CAL_BENCH_ENABLE', int(args.spring_bench == 'right'))
    for path, text in configs.items():
        path.write_text(text, encoding='utf-8')
    external = {root / path for path in re.findall(r'path: (\S+\.c)\b', config) if not path.startswith('Code/')}
    # An open EIDE window can republish its old list after CubeMX removes a peripheral.
    # Confirm these obsolete entries against the generated project before building.
    mdk = root / 'MDK-ARM'
    generated_project = mdk / 'Wheel_Leg_Code.uvprojx'
    generated = {(mdk / node.text).resolve() for node in ET.parse(generated_project).iter('FilePath')}
    obsolete = {root / 'Core/Src' / name for name in ('spi.c', 'tim.c')}
    stale = {path for path in external & obsolete if not path.exists() and path not in generated}
    for path in sorted(stale):
        print(f'EIDE lists obsolete CubeMX source: {path.relative_to(root)}', flush=True)
    sources = sorted(set(snapshot.glob('Code/**/*.c')) | (external - stale))
    common = [args.armcc, '-c', '--cpu', 'Cortex-M7.fp.dp', '--li', '--c99', '--gnu', '-O3',
              '--split_sections', '-g', '--diag_suppress=1', '--diag_suppress=1295',
              '-DSTM32H723xx', '-DUSE_HAL_DRIVER', '-DUSE_PWR_EXTERNAL_SOURCE_SUPPLY', *includes]

    commands = []
    objects = []
    for source in sources:
        relative = source.relative_to(snapshot) if source.is_relative_to(snapshot) else source.relative_to(root)
        obj = output / (str(relative).replace('\\', '_').replace('/', '_') + '.o')
        # The production scatter file selects this EIDE object by basename.
        if relative.as_posix() == 'Code/Model/tinympc14_cost.c':
            obj = output / 'tinympc14_cost.o'
        objects.append(str(obj))
        commands.append([*common, '-o', str(obj), str(source)])
    startup = output / 'startup.o'
    assembly = [str(tool / 'armasm.exe'), '--cpu', 'Cortex-M7', '-o', str(startup), 'MDK-ARM/startup_stm32h723xx.s']
    libraries = [str(root / path) for path in re.findall(r'path: (\S+\.lib)\b', config)]
    link = [str(tool / 'armlink.exe'), '--scatter', 'MDK-ARM/wheel_leg.sct', '--entry', 'Reset_Handler',
            '--map', '--symbols', '--callgraph', '--info', 'sizes,totals,unused,veneers', '--list', str(output / 'project.map'),
            '-o', str(output / 'Wheel_Leg_Code.axf'), str(startup), *objects, *libraries]
    binary = [str(tool / 'fromelf.exe'), '--bincombined', '--output', str(output / 'Wheel_Leg_Code.bin'), str(output / 'Wheel_Leg_Code.axf')]
    conversions = [binary]
    if args.spring_bench == 'right':
        hex_path = output / ('spring_bench_' + args.spring_bench + '.hex')
        conversions.append([str(tool / 'fromelf.exe'), '--i32combined', '--output', str(hex_path), str(output / 'Wheel_Leg_Code.axf')])
    inputs = set(snapshot.rglob('*.c')) | set(snapshot.rglob('*.h')) | set(sources)
    for flag in includes:
        inputs.update((root / flag[2:]).rglob('*.h'))
    inputs.update(root / name for name in ('.eide/eide.yml', 'MDK-ARM/startup_stm32h723xx.s', 'MDK-ARM/wheel_leg.sct'))
    inputs.add(generated_project)
    inputs.update(Path(path) for path in libraries)
    inputs.update((Path(__file__), root / 'tests/distributor_evidence.py'))
    evidence = begin_evidence([*commands, assembly, link, *conversions], inputs=sorted(inputs))
    evidence['evidence_class'] = 'AC5 compilation and link; no hardware execution or actuator capability validation'
    evidence['profile'] = {key: getattr(args, key) for key in ('baseline', 'allocator', 'diagnostics', 'step_check')}
    if args.spring_bench:
        evidence['profile']['spring_bench'] = args.spring_bench
    evidence['obsolete_eide_entries'] = [path.relative_to(root).as_posix() for path in sorted(stale)]
    evidence['snapshot'] = str(snapshot.relative_to(root))
    evidence['original_code_sha256'] = {Path(name).as_posix(): hashlib.sha256(data).hexdigest() for name, data in code.items()}
    evidence['compiled_config'] = {key: int(value) for text in configs.values()
        for key, value in re.findall(
            r'^#define\s+((?:BALANCE_USE_|VOFA_USE_|STEP_)\w+|IMU_USE_HI229|LEGLEN_TURN_ENABLE|BALANCE_DISTRIBUTOR_COMPARE|APP_USE_MUSIC|SPRING_CAL_BENCH_ENABLE)\s+(\d+)u?\s',
            text, re.M)}
    manifest = output / 'build_manifest.json'
    evidence['build_status'] = 'started'
    manifest.write_text(json.dumps(evidence, indent=2, ensure_ascii=False), encoding='utf-8')

    def compile_one(item):
        source, command = item
        result = subprocess.run(command, cwd=root,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        log = result.stdout.decode('utf-8', errors='replace')
        return source.relative_to(root), result.returncode, log

    with ThreadPoolExecutor(max_workers=4) as executor:
        results = list(executor.map(compile_one, zip(sources, commands)))
    logs = []
    for source, code, log in results:
        logs.append(f'{source}: {code}\n{log}')
        if code:
            print(f'FAIL {source}\n{log}', flush=True)
    (output / 'compile.txt').write_text('\n'.join(logs), encoding='utf-8')
    failed = sum(bool(item[1]) for item in results)
    print(f'AC5 compile: {len(sources)} sources, {failed} failures', flush=True)
    if failed:
        evidence['build_status'] = 'compile_failed'
        manifest.write_text(json.dumps(finish_evidence(evidence), indent=2, ensure_ascii=False), encoding='utf-8')
        raise SystemExit(1)
    subprocess.run(assembly, cwd=root, check=True)
    result = subprocess.run(link, cwd=root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log = result.stdout.decode('utf-8', errors='replace')
    (output / 'link.txt').write_text(log, encoding='utf-8')
    print(log)
    if result.returncode:
        evidence['build_status'] = 'link_failed'
        manifest.write_text(json.dumps(finish_evidence(evidence), indent=2, ensure_ascii=False), encoding='utf-8')
        raise SystemExit(result.returncode)
    for conversion in conversions:
        subprocess.run(conversion, cwd=root, check=True)
    evidence['build_status'] = 'linked'
    artifacts = [output / name for name in ('Wheel_Leg_Code.axf', 'Wheel_Leg_Code.bin', 'project.map', 'compile.txt', 'link.txt')]
    if args.spring_bench == 'right':
        artifacts.append(hex_path)
    manifest.write_text(json.dumps(finish_evidence(evidence, artifacts=artifacts), indent=2, ensure_ascii=False), encoding='utf-8')
    print('AC5 full link PASS', flush=True)


if __name__ == '__main__':
    main()
