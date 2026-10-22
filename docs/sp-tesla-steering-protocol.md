# Tesla steering protocol preservation

The official source is merged with continuous history. The fork continues to support the registered legacy two-bit steering protocol as well as the official three-bit protocol.

| Fingerprint / firmware | CP flags | Safety parameter | DBC | Control byte |
| --- | --- | --- | --- | --- |
| No new-protocol marker | no steering flag | no steering flag | `tesla_model3_party_legacy_generated` | byte 2 bits 7:6 |
| Known legacy FSD 14 EPS firmware | `FSD_14=2` | `FSD_14=2` | legacy generated | byte 2 bits 7:6; preserve 1/2 mapping |
| `0x489` on bus 2, or `0x054` on bus 0 | `STEERING_3_BIT=8` | `STEERING_3_BIT=4` | `tesla_model3_party` | byte 2 bits 7:5; no legacy remapping |

`LONG_CONTROL=1` is independent. CP flag `MISSING_DAS_SETTINGS=4` is unchanged and is a different namespace from the safety parameter. A new-protocol marker takes precedence over the historical EPS list. Parser, CANDefine and controller packer select through `get_party_dbc(CP)`. The legacy DBC is generated from the official-width DBC rather than maintaining a second copy.

Types 1/2 remain subject to all existing angle, override, AP ownership, fault and forwarding checks. Type 3 is consumed only as the registered internal lateral handoff marker and must never be transmitted. New three-bit types 4..7 are rejected for host actuation. Firmware must include the matching safety parameter implementation before a new-protocol host is used; a Git parent update alone does not prove deployment.

## Upstream schema contract

- `CarParams.mass @17`, `steerRatioRear @21`, `lateralParams @48`, and `enableBsm @56` moved into `deprecated`.
- Wire ordinals 22, 23 and 24 now represent `rotationalInertiaPerMass`, `tireStiffnessFrontPerMass`, and `tireStiffnessRearPerMass`. Existing serialized pre-migration CarParams cannot be used directly as normalized dynamics; rebuild persistent/session CarParams or explicitly convert historical logs.
- BSM callers use the official `has_bsm(CP, CP_SP)` helper; Tesla continues to report BSM capability.
- The root integration owns cache invalidation and historical-log compatibility. This submodule change does not rewrite device Params or logs.

## Repeatable host evidence

Prepare the failure modes before implementation; preserve the S0 object `3b9bf1895eca4983b2ed7cb620f318d362129112` / tag `exp/20261022-sync/S0`. Official merge parent: `5be750714349624462da012f2f86ea9b686d8b27`.

From the submodule root:

```sh
PYTHONPATH=. python opendbc/car/tesla/tests/steering_protocol_e2e.py /tmp/steering-protocol.json
PYTHONPATH=. python -m pytest opendbc/car/tesla/tests opendbc/sunnypilot/car/tesla/ars408/tests opendbc/sunnypilot/car/tesla/tests opendbc/safety/tests/test_tesla.py opendbc/safety/tests/test_defaults.py opendbc/sunnypilot/car/tests/test_has_bsm.py -q
```

The E2E artifact records four real synthetic-CAN-to-controller-to-compiled-safety paths, exact steering bytes, source identity and script SHA-256. It proves host behavior; physical CAN, EPS behavior and deployed firmware require separate bench/vehicle verification.
