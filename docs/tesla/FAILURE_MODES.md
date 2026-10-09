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

Broad existing interface replay exposed source bug before the correction: Tesla's
function-valued RadarInterface class attribute binds an extra `self` when called
through an instance (all three Tesla platforms fail). Minimal fix: staticmethod
on the Tesla-only assignment; no shared or Toyota behavior changes. This is the
sole deliberate difference from dev-sp-owned source; existing fuzzy interface
replay supplies pre-existing regression coverage.

MISRA baseline passes. Imported source exposes five violations: local-only ambient
TX table at file scope, partially explicit zero initializer, unsigned-to-signed
subtraction, and two early returns in handoff markers. Corrections preserve
rejection and state side effects: local static table, explicit eight zeros,
store the 12-bit torque in a signed intermediate before subtraction, and reject marker with an else
branch that skips all regular actuation checks. Existing handoff safety cases and
ambient mutation tests must remain passing. These are documented source deviations.
