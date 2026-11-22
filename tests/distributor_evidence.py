"""Bind host verification results to the source, configuration and artifact tested."""
from datetime import datetime, timezone
import hashlib
from importlib import metadata
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = ROOT.parent


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _name(path):
    try:
        return path.relative_to(REPOSITORY).as_posix()
    except ValueError:
        return path.as_posix()


def begin_evidence(command, inputs=None, extra_inputs=()):
    """Freeze hashes before compiling; callers can supply isolated snapshot inputs."""
    if inputs is None:
        inputs = [path for folder in (ROOT / 'Code', ROOT / 'tests')
                  for path in folder.rglob('*') if path.suffix in ('.c', '.h')]
        inputs += [Path(__file__), ROOT / 'tests/run_distributor.py', Path(sys.argv[0])]
    paths = sorted({Path(path).resolve() for path in [*inputs, *extra_inputs]})
    source = {_name(path): _sha256(path) for path in paths}
    head = None
    if any((parent / '.git').exists() for parent in (REPOSITORY, *REPOSITORY.parents)):
        head = subprocess.run(['git', '--no-optional-locks', 'rev-parse', 'HEAD'], cwd=REPOSITORY,
                              capture_output=True, text=True, check=True).stdout.strip()
    return {
        'schema': 'distributor_host_evidence_v1',
        'evidence_class': 'host validation; no hardware execution or actuator capability validation',
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'git_head': head,
        'command': command,
        'source_sha256': source,
        '_input_paths': {name: str(path) for name, path in zip(source, paths)},
    }


def finish_evidence(evidence, artifacts=()):
    """Reject source changes during a run and attach compiler and output identity."""
    paths = evidence.pop('_input_paths')
    changed = [name for name, path in paths.items()
               if _sha256(Path(path)) != evidence['source_sha256'][name]]
    if changed:
        raise RuntimeError('Verification inputs changed during the run: ' + ', '.join(changed))
    command = evidence['command']
    compiler = command[0][0] if isinstance(command[0], list) else command[0]
    version = subprocess.run([compiler, '--version'], capture_output=True, text=True,
                             errors='replace', check=False)
    if version.returncode:
        version = subprocess.run([compiler, '--vsn'], capture_output=True, text=True,
                                 errors='replace', check=False)
    evidence['compiler_version'] = (version.stdout + version.stderr).strip()
    evidence['compiler_version_returncode'] = version.returncode
    evidence['compiler_sha256'] = _sha256(Path(compiler))
    commands = command if isinstance(command[0], list) else [command]
    tools = sorted({Path(item[0]).resolve() for item in commands if Path(item[0]).is_file()})
    evidence['tool_sha256'] = {str(path): _sha256(path) for path in tools}
    evidence['python_version'] = platform.python_version()
    evidence['packages'] = {}
    for package in ('numpy', 'scipy'):
        try:
            evidence['packages'][package] = metadata.version(package)
        except metadata.PackageNotFoundError:
            pass
    evidence['artifact_sha256'] = {_name(Path(path).resolve()): _sha256(Path(path))
                                  for path in artifacts}
    evidence['completed_utc'] = datetime.now(timezone.utc).isoformat()
    return evidence
