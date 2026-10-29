# Coupled hinge motor and rotor package scope

Status: CLAIM ONLY; implementation and qualification pending.
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
contains a coupled soft-motor/rotor candidate. Its latest hosted checks succeeded,
but independent observation acceptance remains unresolved at this claim.
It is not yet qualified or merged. Neither that head nor any predicted merge
may become this fork's native pin before independently accepted native evidence
and the actual landed merge identity are supplied by the owning lane.

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
