# P1 failure modes recorded before production edits
- Tesla platform/firmware/torque registration missing: imports and platform enumeration fail.
- Missing Tesla DBC/checksum/generator yields invalid CAN parsing or controller output.
- Restoring other brands or overwriting Toyota alters the lean baseline.
- CarStateSP fields missing/order changed breaks consumers.
- Ambient param=0 must reject 0x679 even after a fresh template; param=1 without fresh template must reject.
- Safety observer/config seams can allow unrelated CAN, stale frames, malformed payload, or excess TX.
- Existing Tesla, Toyota, defaults and interface tests will exercise the complete Python/C safety chains.

Baseline: lean 7b519bc3c79d31a8e1217a0e91a03a6586839019. Source: 3b9bf1895eca4983b2ed7cb620f318d362129112.
No physical CAN or device commands are used. Existing source tests copied before production code; ambient negative test authored first.
