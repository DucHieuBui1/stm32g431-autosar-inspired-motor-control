# Verification scope and results

## Prerequisites and commands

Python 3 and an existing GCC-compatible compiler are required. The runner uses only Python's standard library and builds in a temporary directory. From the repository root:

```sh
python3 scripts/run_host_tests.py --cc gcc
# Alternatively: CC=gcc python3 scripts/run_host_tests.py
```

The Windows PowerShell runners under `Tests/Host/` pin MinGW GCC 6.3.0 at `C:\MinGW\bin\gcc.exe`. The portable runner accepts a compiler command through `--cc` or `CC` and reports its version. Required compile, execution or source-check failures produce a nonzero exit status.

## Latest host result

| Field | Recorded value |
| --- | --- |
| Date | 2026-10-11 |
| Platform | Windows 11, build 26200 |
| Compiler | `gcc.exe (MinGW.org GCC-6.3.0-1) 6.3.0` |
| Input manifest SHA-256 | `6ee03e0e79b1765dfc59b2a5875034424cfd971c9c759d7edd04c3613ac95a9f` |
| Command | `python U:/scripts/run_host_tests.py --cc C:/MinGW/bin/gcc.exe` |
| Executable compilation | 16 passed, 0 failed |
| Executable execution | 16 passed, 0 failed |
| Compile probes | 9 passed, 0 failed |
| Serial source-boundary check | 1 passed, 0 failed |
| Total / exit status | 42 checks passed, 0 failed / 0 |

[Latest raw output](host-test-2026-10-11.txt) records this execution. The count describes checks, not test cases or coverage. The manifest identifies exact input bytes, including firmware, tests, runners, IDE inputs and dependency files. This is host evidence only; target compilation, electrical behavior and motor performance remain unverified.

[Earlier raw output](host-test-result.txt) is retained as a separate historical capture with the same Windows compiler and 42-check summary. It is not the latest execution. A previously reported Linux GCC 13.3 result has no raw artifacts in this repository and is not treated as reproduced evidence. Native Linux execution remains unverified.

## Windows include-path troubleshooting

Older MinGW GCC can fail to open a deeply nested production-source include even when the file exists. The recorded failure affected `test_serial_communication_cdd.c` at its `../../../Software/.../SerialCommunicationCdd.c` include. Use a short checkout path or an unused drive alias:

```powershell
subst U: '<absolute-project-directory>'
python U:/scripts/run_host_tests.py --cc C:/MinGW/bin/gcc.exe
subst U: /D
```

The runner preserves the alias instead of resolving it to the long physical path. The latest run used this workaround. See the [technical incident log](VERIFICATION_INCIDENT_LOG.md).

## Input integrity

From the repository root, check every path in [source_manifest.json](source_manifest.json):

```sh
python3 -c "import hashlib,json,pathlib; entries=json.loads(pathlib.Path('docs/source_manifest.json').read_text()); bad=[e['path'] for e in entries if hashlib.sha256(pathlib.Path(e['path']).read_bytes()).hexdigest()!=e['sha256']]; print('PASS' if not bad else 'FAIL: '+', '.join(bad)); raise SystemExit(bool(bad))"
```

This verifies byte identity, not runtime correctness. Dependency notices and the firmware/test inputs are included in the manifest.

## Module-to-test mapping

Test sources are linked below. Exact per-executable compiler recipes are in [host_test_manifest.json](../scripts/host_test_manifest.json) and correspond to the PowerShell runners. Tests generally include production C sources directly with fake registers, hooks or dependency stubs as indicated.

| Module | Test source | Test type and boundary/fault examples |
| --- | --- | --- |
| Mcu ADC12 / ADC2 | [test_adc.c](../Tests/Host/Adc/test_adc.c) | Production source, fake registers: clock/reset, readback, bounded waits, conversion validity, sequence wrap/corruption |
| EcuM | [test_ecum_adc_integration.c](../Tests/Host/Adc/test_ecum_adc_integration.c) | Production EcuM, stub dependencies: boot order, ADC failures, safe output, one-way handoff and refresh denial |
| Motor CDD | [test_motor_control_cdd.c](../Tests/Host/MotorControl/test_motor_control_cdd.c) | Production CDD, fake registers/hooks: GPIO/TIM1/BKIN, calibration, queue/generation identity, commit cutoffs, drop/expiry, ADC/Break/Update faults and cleanup |
| Immutable configuration | [test_sc1_config.c](../Tests/Host/MotorControl/test_sc1_config.c) | Production config sources/hooks: approved/unapproved shapes, corrupt fields, identity/digest and cleared destinations |
| Fixed-point algorithms | [test_motor_control_algorithm.c](../Tests/Host/MotorControlAlgorithm/test_motor_control_algorithm.c) | Production pure core/reference math: transforms, invalid pointers/norms, PI saturation/anti-windup and SVPWM boundaries |
| Fast-loop | [test_motor_control_fast_loop.c](../Tests/Host/MotorControlFastLoop/test_motor_control_fast_loop.c) | Production composition/cores with synthetic trajectories and reference checks |
| Fast-loop injection | [test_motor_control_fast_loop_injection.c](../Tests/Host/MotorControlFastLoop/test_motor_control_fast_loop_injection.c) | Production composition, stub cores: staged failures, ordering and fail-closed publication |
| Observer | [test_motor_control_observer.c](../Tests/Host/MotorControlObserver/test_motor_control_observer.c) | Production pure core: both axes, rounding ties, angle wrap/half-turn, speed/slew, rolling oracle, reliability thresholds, metamorphic sequences and replay |
| Startup/handover | [test_motor_control_startup.c](../Tests/Host/MotorControlStartup/test_motor_control_startup.c) | Production pure core: exact phase lengths, current cap, direction/wrap, stop/fault precedence, observer loss, final bounds and corruption |
| OS C core | [test_os_step7a.c](../Tests/Host/Os/test_os_step7a.c) | Production C, fake MCU/port hooks: task priorities, events, guards and scheduler selection; no ARM assembly execution |
| OS configuration | [test_os_cfg_step7a.c](../Tests/Host/Os/test_os_cfg_step7a.c) | Production task configuration, service stubs: wait/clear before assigned service |
| Serial CDD | [test_serial_communication_cdd.c](../Tests/Host/SerialCommunication/test_serial_communication_cdd.c) | Production CDD, fake registers: CRC/headers, session/sequence/lease, signed current, RX errors/full/wrap, quarantine and rate boundaries |
| Mcu communication | [test_mcu_communication.c](../Tests/Host/SerialCommunication/test_mcu_communication.c) | Production setup, fake MCU registers: setup/readback |
| Supervision/state/watchdog | [test_supervision.c](../Tests/Host/Supervision/test_supervision.c) | Production services with hardware/dependency stubs: readiness interlocks, latching, simultaneous faults, drop/wrap, non-actuating responses, alive profiles and fatal handling |
| UserCommand | [test_user_command.c](../Tests/Host/UserCommand/test_user_command.c) | Production profile/validation: disabled defaults, enabled limits, adjacent invalid values and runtime rejection |
| UserCommand corruption | [test_user_command_corruption.c](../Tests/Host/UserCommand/test_user_command_corruption.c) | Production profile with corruption hook |

## Compiler flags and parity

Every recipe retains `-Wall -Wextra -Werror -pedantic`. ADC/EcuM use C99 and `-O0`. Algorithm, observer, startup and both fast-loop executables use C11 and `-O2`; other executables use C11 and `-O0`. Include order and `-lm` use/placement are retained per recipe.

The Python runner checks the serial dependency boundary and all UserCommand compile profiles: one valid enabled profile; invalid enabled value; disabled latent current; current -301/+301 mA; duration 0/5001 ms; lease 0/101 ms. Negative probes must fail compilation. Probe changes affect only temporary source copies, as in the PowerShell runner.

The executable inventory is checked against every current `test_*.c`. Windows runners retain their pinned compiler identity; Python intentionally accepts another compiler and prints its identity. The recipes and extra checks correspond to the PowerShell runners; the latest result exercises them on Windows. Native Linux execution parity is not established. Full-platform policy checks and target object-code audits are outside this runner.

## Target build

The recorded STM32CubeIDE 2.0.0 headless attempt exited 13 during project import, before compilation:

```text
No file system is defined for scheme: V
```

The import argument was `V:/`; Eclipse treated it as a URI scheme. Use a Windows-native absolute path as shown in the [build instructions](../README.md#build-instructions). That corrected command has no successful build result recorded here. Current-source compilation/linking is unverified; existing ELF files are not evidence for it. No target build was executed as part of the latest host run.

## Hardware evidence

Historical summaries refer to earlier firmware images:

- Gate G431-0: pre-ADC image programmed/byte-verified and bounded software/SWD regression on 2026-08-27, with motor/power-stage supply disconnected. Independent clock measurement and clock-failure injection remained open.
- `G431-ADC-TR-001`, prepared 2026-08-31: T1 PASS on original/corrected images; T2 attempts 001-005 FAIL/ABORTED; attempt 006 bounded functional PASS. T3/T4 unexecuted, target gate open. Recorded corrected-image SHA-256: `B56D73EF567F572A9A80E0604A6C613E1B2AE34D6D05B0DD142C885636F4B108`.

The underlying raw logs, methods, photos and exact binaries are not available in this repository, so these historical summaries cannot be independently checked here. They do not establish hardware behavior for the current source. Full motor operation, powered commissioning and closed-loop performance remain unverified.

Fake registers cannot reproduce asynchronous hardware, flag semantics, analog noise or preemption. Stubs cannot prove the integrated dependency. Host execution does not execute SVC/PendSV assembly or establish stack margin, WCET, watchdog expiry, trigger latency, electrical protection or motor parameters. Those remain separate target obligations.
