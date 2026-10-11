# Verification incident log

## PUB-001: Windows include-path length

- Date: 2026-10-11.
- Observation: MinGW GCC 6.3.0 could not open the serial CDD test's included production source under the long checkout path; the other 15 executables passed.
- Investigation: the included file existed. Resolving a short drive alias back to the long path reproduced the failure; preserving the alias allowed compilation.
- Root cause: older MinGW/Windows include-path length handling.
- Correction: preserve the script's absolute drive alias and execute through a short alias. Firmware and test logic are unaffected.
- Retest: the latest Windows run compiled/executed all 16 executables and passed nine compile probes plus the serial boundary check: 42 checks, zero failures.
- Residual limitation: this compiler requires a sufficiently short path; native Linux execution is unverified.

## PUB-002: CubeIDE project-import failure

- Date: 2026-10-11.
- Observation: headless build exited 13 before compilation.
- Investigation: the Eclipse workspace log locates the error in project import for `V:/`; compilation had not started.
- Root cause: Eclipse interpreted `-import V:/` as a URI scheme and reported `No file system is defined for scheme: V`.
- Proposed correction: pass a Windows-native absolute project path.
- Retest: no successful result is recorded for the corrected command.
- Residual limitation: current target compilation/linking remains unverified. Host results do not close this build check.
