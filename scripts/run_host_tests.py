"""Run the existing host C tests offline; never access target hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).absolute().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'gcc'),
                        help='Compiler command; defaults to CC or gcc')
    options = parser.parse_args()
    cc = shlex.split(options.cc, posix=os.name != 'nt')
    failures = []
    passed = 0

    def check(name, args, expected=0):
        nonlocal passed
        result = subprocess.run(args, cwd=ROOT, capture_output=True, text=True)
        ok = result.returncode == 0 if expected == 0 else result.returncode != 0
        print(('PASS: ' if ok else 'FAIL: ') + name, flush=True)
        if ok:
            passed += 1
        else:
            failures.append(name)
            print(result.stdout + result.stderr)
        return ok

    try:
        identity = subprocess.run(cc + ['--version'], check=True,
                                  capture_output=True, text=True)
        print(identity.stdout.strip(), flush=True)
        recipes = json.loads((ROOT / 'scripts/host_test_manifest.json').read_text())
        discovered = {p.stem for p in ROOT.glob('Tests/Host/*/test_*.c')}
        if discovered != {r['name'] for r in recipes}:
            raise ValueError('Manifest does not match every existing host test source')
        with tempfile.TemporaryDirectory(prefix='g431-host-') as temp:
            build = Path(temp)
            for recipe in recipes:
                exe = build / (recipe['name'] + ('.exe' if os.name == 'nt' else ''))
                if check(recipe['name'] + ' compile',
                         cc + recipe['args'] + ['-o', str(exe)]):
                    check(recipe['name'] + ' execute', [str(exe)])
            source = (ROOT / 'Software/Bsw/ComplexDrivers/SerialCommunication/Src/SerialCommunicationCdd.c').read_text()
            forbidden = r'MotorControlCdd_|StateManager_|\bOs_|\bStartOS\b|\bWaitEvent\b|\bSetEvent\b'
            if re.search(forbidden, source):
                failures.append('serial source boundary')
                print('FAIL: serial source boundary')
            else:
                passed += 1
                print('PASS: serial source boundary')
            source = (ROOT / 'Software/Asw/UserCommand/Src/User_Command.c').read_text()
            profiles = {
                'valid_enabled_profile': ('1U', '300', '5000U', '100U'),
                'invalid_enabled': ('2U', '0', '0U', '0U'),
                'disabled_latent_current': ('0U', '1', '0U', '0U'),
                'invalid_current_low': ('1U', '-301', '5000U', '100U'),
                'invalid_current_high': ('1U', '301', '5000U', '100U'),
                'invalid_duration_low': ('1U', '300', '0U', '100U'),
                'invalid_duration_high': ('1U', '300', '5001U', '100U'),
                'invalid_lease_low': ('1U', '300', '5000U', '0U'),
                'invalid_lease_high': ('1U', '300', '5000U', '101U'),
            }
            old = ['#define USER_COMMAND_ENABLED          0U',
                   '#define USER_COMMAND_CURRENT_MA       0',
                   '#define USER_COMMAND_RUN_DURATION_MS  0U',
                   '#define USER_COMMAND_LEASE_MS         0U']
            flags = ['-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O0',
                     '-I', 'Software/Bsw/Common/Inc', '-I', 'Software/Asw/UserCommand/Inc']
            for name, values in profiles.items():
                text = source
                for original, value in zip(old, values):
                    if text.count(original) != 1:
                        raise ValueError('UserCommand profile substitution no longer matches source')
                    text = text.replace(original, original.rsplit(' ', 1)[0] + ' ' + value)
                probe = build / (name + '.c')
                probe.write_text(text)
                check(name, cc + flags + ['-c', str(probe), '-o', str(build / (name + '.o'))],
                      expected=0 if name == 'valid_enabled_profile' else 1)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        failures.append(str(error))
        print('FAIL: ' + str(error))
    print(f'Summary: {passed} checks passed; {len(failures)} failed')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

