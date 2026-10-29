# Coupled hinge motor and rotor package scope

Status: SOURCE CANDIDATE; built and packed qualification pending.
Owner: Codex, continuing the user-authorized Robot Lab physics work.
Integration target: `brickwright/part-pair-filter` at
`f488edd3c9945243331eed7e3d887a1e95868716` in this owned fork.

The existing bw.8 package exposes the optional native hinge rotor and retains
its bounded built/packed controls. Its adopting Lite consumer subsequently
failed unchanged full-turn endpoints. A diagnostic observed completion near
360.852 degrees followed by actual body and rotor reversal during zero-duty
braking to about 353.906 degrees. Preserve that failure and every old guard;
prior library qualification does not qualify this consumer.

The [native owning fork's PR 6](https://github.com/CrispStrobe/JoltPhysics/pull/6)
is qualified and normally merged at
`0ff27c94898c910cb024b4e74218818358ee4852`. All enabled native checks and
independent retained-observation audits passed with unchanged physics guards.
Earlier fixture failures and exact-float observer refusals remain preserved.
This native result does not qualify the WebAssembly package or Lite consumer.

## Bounded ownership

Own the Jolt native-source pin in `CMakeLists.txt` only after that native
gate; the root `package.json` and matching `package-lock.json` version metadata
for an explicit `1.1.0-bw.9` candidate; narrowly necessary matching package README
and generated types metadata; `brickwright/verify-rotor-inertia.mjs`; affected
built and packed scalar/SIMD verification and raw-observation retention steps
in `.github/workflows/brickwright-release.yml`; this scope and corresponding
`BRICKWRIGHT.md` status. Existing export paths, binding API and notices remain.
Generated declarations, if produced by the unchanged qualified build, must
match the actual package; no unrelated type, IDL or toolchain ownership follows.

The new verifier controls must exercise the high reflected-to-bare inertia
ratio and zero-duty brake transition in the actual WebAssembly engine. Admit
and read back the intended dynamic-body mass and inertia before constructing
the hinge, using the existing native inverse-inertia API where body creation's
small-inertia fallback would otherwise change the fixture. Record actual
before/after body poses, velocities, inertia, rotor scalar/impulses, motor
impulses, commands, constraints and iteration settings. No fabricated encoder,
JavaScript momentum correction, target reset to pass an endpoint, or substitute
physical response is allowed.

Keep the existing shipped motor iteration boundary (30 velocity / 5 position),
all old verifier cases and their budgets, neighboring-joint/anchor guards,
configuration and same-build rollback checks, zero-rotor controls, gear scenes,
rope-impulse/trajectory comparisons, non-gear regression hash and notices.
Native and WebAssembly precision contracts remain explicitly distinct; no
existing threshold is relaxed. New passivity checks use the actual implicit
motor impulse and solved rate at a source-admitted zero-bias brake boundary,
with any float observation budget justified and frozen before producer outputs.
Retain a meaningful previous-engine negative control and all first failures.

## Ordered qualification and release gates

No source implementation precedes merging this claim. After that release and
the separately accepted native merge proof, normally compose the authoritative
branch and update only the explicit native pin and package candidate metadata.
Do not silently adopt a native candidate, rebuild an already accepted original
for inspection, or advance any Lite pin from source review alone.

Use the existing hosted build and package workflow. Qualify built AND packed
scalar and SIMD consumers, retaining distinct trace identities and partial
first-failure observations. Bind actual logical and checkout source/tree,
native pin, compiler/runtime, verifier, module and archive bytes. Independently
audit the once-acquired official originals and all preserved old guards before
a normal exact-head merge with actual parent/tree verification. Only declared
skips may be accepted; queued or skipped physics is not evidence.

A `1.1.0-bw.9` source candidate is not a published release. Any tag, package
publication or application pin must use separately qualified exact bytes and
provenance within the user-authorized work. Affected Lite endpoint programmes, model guards and
rendered worker/save-reopen qualification remain in the consumer lane. This
claim does not establish full-model acceptance, hardware accuracy, arbitrary
sleeping behavior, contact calibration, deployment or an application release.

## Implementation checkpoint

The candidate pins that actual native merge and identifies package
`1.1.0-bw.9`; the root lockfile version metadata is made consistent without
changing dependency resolutions. Existing binding API and export/type paths
are unchanged; no IDL or generated type repair is required by this source.

The verifier's default invocation retains the eleven existing scenes and
1,215 steps. A separate `--controls=motor-block` invocation records three new
100-step scenes: both command signs and an active motor clamp. It explicitly
reads back the admitted small inertia, mass, native spring settings, target,
motor state and impulses. The limited scene uses 30 positive / 20 negative /
50 zero-duty steps; the others use 30 / 30 / 40, reversing signs in the reverse
scene. All require driven and moving-brake observations. New motor KKT and
rotor residual bounds are `1e-4`; existing point/transverse `2e-4` guards remain.

Softness and bias observations are explicitly source-derived from native float
readbacks using the pinned SpringPart operation sequence, not presented as
native cached values. Passivity requires an exactly zero effective bias and
uses implicit motor impulse times solved relative rate. Its predeclared float
observation budget is 32 binary32 epsilons times absolute motor impulse times
`max(1, abs(rate_before), abs(rate_after))`; this does not authorize arbitrary
solver error or fitted endpoint tolerances.

Built and packed scalar/SIMD controls emit four distinct new reports separately
from all four old scene reports. Two meaningful negative controls execute only
the new three scenes against the exact qualified bw.8 scalar/SIMD archive,
verified by SHA256, SHA512 integrity, package version and both module hashes.
Previous-engine identity is explicit. Reports preserve all observed steps and
violations; `complete:true` means a complete observation, not acceptance, and
physical violations produce exit status one. No old successful scene is rerun
in that previous engine just for inspection.

No local build or engine execution has occurred. All WebAssembly results,
package qualification, publication and consumer endpoint repairs remain pending.
The separate Lite clutch-bench physical 24:40 failure is not claimed repaired.
