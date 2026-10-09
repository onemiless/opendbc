# P1 acceptance

Base: `7b519bc3c79d31a8e1217a0e91a03a6586839019`; source: `3b9bf1895eca4983b2ed7cb620f318d362129112`.

- Tesla owned car, ARS408, DBC, generator, and safety paths restored.
- 12 existing shared files changed, including existing replay/timing fixture tables; Toyota source and safety mode unchanged.
- Tesla torque data already present in lean override.toml; no restoration necessary.
- Dynamic discovery already covers platform_list, fingerprints_ext, generator and car_helpers.
- Targeted Tesla/car/radar/defaults/Toyota: 1593 passed, 572 skipped, 1035 subtests.
- Complete core safety including release build and ELM327: 1454 passed, 579 skipped, 1035 subtests.
- Car interfaces/firmware/routes: 67 passed, 6 skipped, 2023 subtests.
- Ambient mutation: disabling initialization fails positive cases; unconditional enabling fails the prewritten param=0 negative case; restored 11 cases pass.
- MISRA: baseline and final both pass. Initial imported source warnings and corrections are documented in FAILURE_MODES.md.
- Broad CAN suite retains exactly the same 38 failed test IDs as lean baseline (removed-brand fixtures); no new failed subtests after Tesla fixtures restored.

## Source deviations

`opendbc/car/tesla/interface.py`: wrap function radar factory in staticmethod so existing instance callers do not inject `self`.
`opendbc/safety/modes/tesla.h`: explicit array zeros, signed intermediate for 12-bit torque, handoff rejection through the existing single return while skipping ordinary actuation checks.
`opendbc/safety/modes/defaults.h`: local static whitelist table instead of file scope.
Existing regression cases cover these corrections; no unit tests were authored after production edits.

## Repeat

Use Python 3.12 with pyproject dependencies (the recorded targeted runs used `/Users/mile/Desktop/mo-op/.venv/bin/python`).

```sh
python -m pytest -q opendbc/car/tesla opendbc/sunnypilot/car/tesla opendbc/sunnypilot/car/tests opendbc/safety/tests/test_tesla.py opendbc/safety/tests/test_toyota.py opendbc/safety/tests/test_defaults.py
python -m pytest -q opendbc/car/tests
python -m pytest -q opendbc/safety/tests/test_defaults.py opendbc/safety/tests/test_elm327.py opendbc/safety/tests/test_release_build.py opendbc/safety/tests/test_tesla.py opendbc/safety/tests/test_toyota.py
bash opendbc/safety/tests/misra/test_misra.sh
```

No push, physical CAN, firmware write, device test, or vehicle acceptance performed by P1.

Coverage shell wrapper remains unaccepted on this macOS host: llvm-cov/gcovr working-directory errors, see T1.4-test-sh.txt. Independent safety and MISRA pass; do not claim 100% coverage.
