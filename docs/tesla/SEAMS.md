# P1 shared seams (registered before edits)

| File | Purpose | Source |
|---|---|---|
| opendbc/car/values.py | Register Tesla | 3b9bf189 |
| opendbc/car/structs.py | Tesla state extension | 3b9bf189 |
| opendbc/sunnypilot/car/interfaces.py | Six Tesla initialization functions | 3b9bf189 |
| opendbc/sunnypilot/car/car_list.json | Tesla list entries only | 3b9bf189 |
| opendbc/can/dbc.py | Tesla checksum only | 3b9bf189 |
| opendbc/safety/safety.h | Validated RX observer | 3b9bf189 |
| opendbc/safety/declarations.h | Optional observer hook | 3b9bf189 |
| opendbc/safety/modes/defaults.h | Param 1 bounded ambient output | 3b9bf189 |
| opendbc/safety/tests/libsafety/safety.c | Tesla summon reset | 3b9bf189 |
| opendbc/safety/tests/test_defaults.py | Source tests and prewritten negative regression | 3b9bf189 + T1.5 |

Tesla safety mode is a Tesla-owned path restored with documented MISRA corrections. Generator, platform_list,
fingerprints_ext and car_helpers discover registered brands dynamically and need no seam. Tesla torque override entries already exist unchanged in lean.
| opendbc/car/tests/routes.py | Restore existing Tesla replay fixtures, no new tests | 3b9bf189 |
| opendbc/car/tests/test_fw_fingerprint.py | Restore Tesla 0.1s timing fixture, total 0.8s | 3b9bf189 |
