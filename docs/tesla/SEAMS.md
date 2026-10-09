# P1 shared seams (registered before edits)

| File | Purpose | Source |
|---|---|---|
| opendbc/car/values.py | Register Tesla | 3b9bf189 |
| opendbc/car/structs.py | Tesla state extension | 3b9bf189 |
| opendbc/car/torque_data/params.toml | Tesla torque data only | 3b9bf189 |
| opendbc/sunnypilot/car/interfaces.py | Six Tesla initialization functions | 3b9bf189 |
| opendbc/sunnypilot/car/car_list.json | Tesla list entries only | 3b9bf189 |
| opendbc/can/dbc.py | Tesla checksum only | 3b9bf189 |
| opendbc/safety/safety.h | Validated RX observer | 3b9bf189 |
| opendbc/safety/declarations.h | Optional observer hook | 3b9bf189 |
| opendbc/safety/modes/defaults.h | Param 1 bounded ambient output | 3b9bf189 |
| opendbc/safety/tests/libsafety/safety.c | Tesla summon reset | 3b9bf189 |
| opendbc/safety/tests/test_defaults.py | Source tests and prewritten negative regression | 3b9bf189 + T1.5 |

Tesla safety mode is a Tesla-owned path restored exactly. Generator, platform_list,
fingerprints_ext and car_helpers discover registered brands dynamically and need no seam.
