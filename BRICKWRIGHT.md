# Brickwright fork of JoltPhysics.js

Branch `brickwright/gear-mesh-master` (not released): this branch with the Jolt pin moved to the JoltPhysics fork's
`brickwright/gear-mesh-master` (`7006be437d970eadd55c6454da9c46f169bce68b` = upstream master `01adafb` + the same gear
changes), to check the gear changes on current Jolt master. Binding changes: `PhysicsSettings.mDeterministicSimulation`
removed from `JoltJS.idl` (removed upstream; the simulation is always deterministic), `-DJPH_DEBUG` added to the Debug
glue defines (upstream no longer derives `JPH_DEBUG` from `NDEBUG`), and `OBJECT_LAYER_BITS` set as a forced cache variable (master made it a cache string defaulting to 16, so the plain `set` no longer reached the library and the 32-bit glue failed to link). The non-gear regression hash is checked against `f7b0121879392938`, the hash of a control build of plain upstream
master `01adafb` without the gear commits (branch `brickwright/plain-master-control`, Actions run 39223975871); 1.1.0
gives `582ce7fb0bd82275`, so upstream's non-gear changes since v5.6.0, not the gear commits, move it.

Branch `brickwright/gear-mesh` = upstream tag **1.1.0** (`c9c122b`) built against the Brickwright JoltPhysics fork
(<https://github.com/CrispStrobe/JoltPhysics/tree/brickwright/gear-mesh>, commit `fdedbe6` = Jolt v5.6.0 + gear
fixes). Nothing here goes upstream (owner decision). JoltPhysics.js and Jolt Physics are MIT licensed; `LICENSE` is
upstream's, unchanged.

## What changes

1. **`GearConstraint` passes torque × ratio** (Jolt applied body 2's impulse without the ratio: torque crossed the
   mesh 1:1, energy was lost, and r < 1 or r < 0 diverged). Same JS API. Details and the C++ diff rationale:
   the JoltPhysics fork's `BRICKWRIGHT.md`.
2. **New `GearMeshConstraint`**: a mesh as a tooth contact at the pitch point (linear + angular Jacobian for both
   gears), so the reaction loads the axles and a gearbox on a free frame conserves angular momentum. Works for
   parallel axes (spur, internal: pitch point from the ratio) and crossed axes (worm, bevel: pitch point given).
3. **A bounce across two bodies (1.1.0-bw.3)**: a `LinearCast` body whose impact would bounce while it touches
   another body as well (a brick landing across two beams) is no longer resolved by the CCD step at the first body's
   contact point (it spun and turned over; two loose beams put the whole reaction on one); the next step's discrete
   solve applies the restitution with every manifold. Single-body bounces and non-bouncing impacts are unchanged.
   C++ change and rationale: the JoltPhysics fork's `BRICKWRIGHT.md` section 3 (commit `a40ff0b`).
4. **`MaterialPairs` (1.1.0-bw.3)**, bindings only (`JoltJS.h`): restitution per material PAIR instead of Jolt's
   max(e1, e2). See the JS API below.
5. **`RackMeshConstraint` (1.1.0-bw.4)**: a rack and pinion whose relation holds relative to the frames carrying
   them (the pinion's hinge, the rack's slider), as `GearMeshConstraint` does for gears; `RackAndPinionConstraint`
   ties world rates, so a turning or driving frame drove the rack. C++ and rationale: the JoltPhysics fork's
   `BRICKWRIGHT.md` section 4 (commit `1631b95`).
6. **`PartPairSimShapeFilter` (1.1.0-bw.5)**, bindings only (`JoltJS.h`): collision filtered part by part between two
   bodies, in C++. Brickwright's Robot Lab keeps apart the parts of two jointed bodies that sit in or round each other
   (a pin in its hole) while the rest of the two bodies collides. That filter was a `SimShapeFilterJS`: a call into
   JavaScript for every sub-shape pair of every body pair in the narrow phase (5–12 % of a step on big models), and
   impossible with the multithreaded builds, where Jolt calls the filter from its worker threads and the JS object does
   not exist there. Each collider is a leaf shape whose user data is its index (`ShapeSettings.mUserData`);
   `SetColliderPart` maps it to a part; `AddBodyPair` + `AddPartPair` list part pairs kept apart, or with
   `inCollideList` the only pairs that collide. Read only during `Update`: thread safe. Verification scene I in
   `brickwright/verify.mjs`.
7. Package version `1.1.0-bw.5`; package name stays `jolt-physics`, so `import 'jolt-physics/wasm-compat'` is
   unchanged. All flavours (with the debug builds) are built by upstream's `build.sh`.

Files: `CMakeLists.txt` (FetchContent → fork commit), `JoltJS.idl` / `JoltJS.h` (bindings),
`brickwright/verify.mjs` (verification scenes), `.github/workflows/brickwright-release.yml`, `package.json`.

## JS API

```js
// Part-pair filter (1.1.0-bw.5): leaves carry their collider index as user data
const box = new Jolt.BoxShapeSettings(new Jolt.Vec3(0.01, 0.01, 0.01), 0.001); box.mUserData = colliderIndex;
compoundSettings.AddShape(position, rotation, box, colliderIndex);
const filter = new Jolt.PartPairSimShapeFilter();
filter.SetColliderPart(bodyId, colliderIndex, partNumber);         // per collider of each filtered body
filter.AddBodyPair(bodyA, bodyB, false);                           // false: list the pairs kept apart; true: the pairs that collide
filter.AddPartPair(bodyA, bodyB, partOfA, partOfB);
physicsSystem.SetSimShapeFilter(filter);                           // keep `filter` alive while the system uses it
```

```js
const s = new Jolt.GearMeshConstraintSettings();
s.mSpace = Jolt.EConstraintSpace_WorldSpace;
s.mPoint1 = new Jolt.RVec3(...c1);  s.mHingeAxis1 = new Jolt.Vec3(...a1);   // gear 1 centre / axis
s.mPoint2 = new Jolt.RVec3(...c2);  s.mHingeAxis2 = new Jolt.Vec3(...a2);   // gear 2 centre / axis
s.mRatio = -8 / 24;                 // ω2/ω1 RELATIVE TO THE FRAME about a2/a1 (external spur, same axes: negative)
s.mContactPoint = new Jolt.RVec3(...p);  // pitch point, only for non-parallel axes (worm/bevel); ignored otherwise
s.mNumVelocityStepsOverride = 30; s.mNumPositionStepsOverride = 10;          // as for GearConstraint
const mesh = Jolt.castObject(s.Create(gear1Body, gear2Body), Jolt.GearMeshConstraint);
Jolt.destroy(s);
physicsSystem.AddConstraint(mesh);
mesh.SetConstraints(hinge1, hinge2);  // the hinges holding gear 1 / gear 2 (gear may be either hinge body): drift correction
// mesh.GetContactPoint(), GetContactNormal(), GetPositionError() (m), GetTotalLambda() (N·s), GetRatio()
// constraint.GetSubType() === Jolt.EConstraintSubType_GearMesh
```

Worm: `mPoint1`/`mHingeAxis1` = worm, `mPoint2`/`mHingeAxis2` = wheel, `mRatio = ±1/teeth` (sign by thread hand,
about the given axes), `mContactPoint` = the worm's pitch point towards the wheel. No friction: no self-locking.

### Restitution per material pair (1.1.0-bw.3)

```js
// A body's material is the low 8 bits of its user data (0 = none). Materials 1 .. 31.
bodyCreationSettings.mUserData = 2;                       // e.g. 1 ABS, 2 rubber
Jolt.MaterialPairs.prototype.sSetRestitution(1, 2, 0.07); // both orders
Jolt.MaterialPairs.prototype.sInstall(physicsSystem);     // physicsSystem.SetCombineRestitution(table lookup)
// A contact between two bodies with materials and a table entry gets that e; any other pair Jolt's max(e1, e2).
// sGetRestitution(a, b) (-1: no entry), sClear(). The table is one per module (a C++ combine function has no
// context), shared by every PhysicsSystem that installs it.
```

### Rack and pinion relative to the frames (1.1.0-bw.4)

```js
const s = new Jolt.RackMeshConstraintSettings();       // body 1 the pinion, body 2 the rack (as RackAndPinion)
s.mSpace = Jolt.EConstraintSpace_WorldSpace;
s.mHingeAxis = new Jolt.Vec3(...a); s.mSliderAxis = new Jolt.Vec3(...b); s.mRatio = 1 / pitchRadius; // rad per metre
const rack = Jolt.castObject(s.Create(pinionBody, rackBody), Jolt.RackMeshConstraint);
physicsSystem.AddConstraint(rack);
rack.SetConstraints(pinionHinge, rackSlider);  // frames + drift since this call (call again to re-base)
// GetRatio(), GetPositionError() (rad), GetTotalLambda(); subtype Jolt.EConstraintSubType_RackMesh
```

## Build and release

GitHub Actions only (`brickwright-release.yml`): upstream `ci/build-examples.sh` (emsdk **6.0.2**, as upstream's
`ci/install-emsdk.sh`; `npm run build` → all flavours), then `brickwright/verify.mjs --assert` on the build and on
the packed tarball, a comparison run against npm `jolt-physics@1.1.0` (the non-gear regression scene must hash
identically), `npm pack`, and on a `v*-bw.*` tag a GitHub Release with the tarball, its sha256 and npm integrity.

## Release `v1.1.0-bw.1`

* Release: <https://github.com/CrispStrobe/JoltPhysics.js/releases/tag/v1.1.0-bw.1> (workflow run
  <https://github.com/CrispStrobe/JoltPhysics.js/actions/runs/39211374047>)
* Tarball: `https://github.com/CrispStrobe/JoltPhysics.js/releases/download/v1.1.0-bw.1/jolt-physics-1.1.0-bw.1.tgz`
* sha256 `6ba56b5743046f7ccbb650f05dc5c369567715faa5cf852a10f6d6b220db6fd4`
* npm integrity `sha512-4rL7nWPcCKRaQOPozSSXb9FnuftEdybCJIAeF4QGdY1T3zYs7T5syqBqf/8RjHkWJmlBhZVokoXU1EFEn9rOzw==`
* Built from this branch at `0f21a6b` with JoltPhysics fork `fdedbe6`. The build is reproducible: the branch run
  before the tag produced the same sha256.

## Using it in Brickwright (notes for the integration owner)

Pin: `"jolt-physics": "https://github.com/CrispStrobe/JoltPhysics.js/releases/download/v1.1.0-bw.1/jolt-physics-1.1.0-bw.1.tgz"`
(package.json and `scripts/integrate.mjs`), lockfile `integrity` as above. Imports (`jolt-physics/wasm-compat`) are
unchanged. `GearConstraint` alone already passes torque × ratio after the pin; the steps below remove the workarounds.

```js
// jolt-core.js, 'gear' kind (static AND moving frames) - replaces buildGearConstraint + movingFrameMesh:
const meshGear = (id, j1, j2, ratio, steps) => {
    const s1 = gearSide(j1), s2 = gearSide(j2);           // as now: {body, sign}; sign = -1 when the gear is hinge body1
    const ms = new J.GearMeshConstraintSettings();
    ms.mSpace = J.EConstraintSpace_WorldSpace;
    ms.mPoint1 = new J.RVec3(...j1.point); ms.mHingeAxis1 = new J.Vec3(...j1.axis.map(v => v * s1.sign));
    ms.mPoint2 = new J.RVec3(...j2.point); ms.mHingeAxis2 = new J.Vec3(...j2.axis.map(v => v * s2.sign));
    ms.mRatio = ratio;                                     // the contract ratio as is: w(joint2) = ratio * w(joint1)
    // worm only: ms.mContactPoint = point on the common perpendicular of the two axes, worm pitch radius from the worm axis
    ms.mNumVelocityStepsOverride = steps; ms.mNumPositionStepsOverride = GEAR_POSITION_STEPS;
    const c = J.castObject(ms.Create(getBodyForId(s1.body), getBodyForId(s2.body)), J.GearMeshConstraint);
    J.destroy(ms);
    physicsSystem.AddConstraint(c); allConstraints.push(c);
    c.SetConstraints(hingeMap.get(j1.id).hinge, hingeMap.get(j2.id).hinge);  // frames + drift correction: required
    return c;
};
```

* Delete: the `|ratio| <= 1` swap and axis flip (the divergence was the ratio bug), `movingFrameMesh` and its
  per-step re-made contact / `slip` bookkeeping (the mesh is exact on a moving frame), and the `relativeGear`
  carrier path's tooth contact: a planet on a carrier meshing with a sun works with `meshGear` as long as each
  gear's hinge is passed (its other body is that gear's frame).
* `worm`: `meshGear(..., g.ratio, WORM_VELOCITY_STEPS)` with `mContactPoint`; keep `wormHolds` (no friction in the
  mesh, so no self-locking).
* `rackAndPinion`: unchanged (it was right). Do not call its `SetConstraints` unless the hinge/slider axes have the
  same signs as `mHingeAxis`/`mSliderAxis` (its drift term reads raw joint values).
* `GEAR_MIN_INERTIA` / `GEAR_VELOCITY_STEPS`: keep at first; with the ratio fixed the floor may be lowered -
  re-measure the stalled-arm case before changing it.
* Re-run `test/bw-sim-validation-2.test.mjs` (gear sections) and the Excavator dig cycle.


## Results

| Scene | 1.1.0 `GearConstraint` | fork `GearConstraint` | fork `GearMeshConstraint` |
|---|---|---|---|
| Free discs 8:24, τ on 8T: α1 (expected τ/(I1 + I2·(1/3)²) = 5.00 rad/s²) | 2.50 | 5.00 | 5.00 |
| same: (KE − work)/work | −50 % | 5e-7 | 7e-7 |
| Floating gearbox, internal motor drives 8:24: max abs(L) / gear's abs(I1·ω1) | 0.68 | 0.58 | 5.6e-7 |
| same: relative ratio (expected −0.3333) | +0.54 | +0.41 | −0.3333 |
| same: frame turn in 2 s (inertias chosen so I1ω1 + I2ω2 = 0: physically 0°) | 64° | 159° | 0.000° |
| Worm 1:24 in a floating frame: abs(L) / gear's; ratio (expected 0.04167) | n/a | n/a | 0.0089; 0.04164 |
| Balance 1:3, drum weights m2 = 3·m1: angle after 2 s | 175° | 0° | 0° |
| Balance 1:3, m2 = m1 (must run away) | 0° (balanced: the bug) | 146° | 146° |
| 8T held by its motor, 0.05 N·m on 24T for 5 s: 8T drift / mesh phase error | 0.0003° / 0.0005° | 0° / 0° | 0° / 0° |
| Non-gear regression scene (contacts, hinge, rack and pinion; state hash after 2 s) | 582ce7fb… | 582ce7fb… (identical) | — |
| Rack and pinion travel per pinion radian (expected −0.008 m) | −0.008 | −0.008 | — |

Gear bodies: 8T I = 1e-4 / 3e-5 kg·m², 24T 9e-4 / 9e-5 kg·m² (Brickwright floors gear inertia at 3e-5),
30 velocity / 10 position steps on the gear constraints, 60 Hz. The worm's 0.9 % falls to 0.1 % when the frame's inertia is
made isotropic: most of it is the tumbling frame (Jolt integrates without a gyroscopic term), not the mesh.

## Release `v1.1.0-bw.3` (2026-10-20)

Built from `brickwright/gear-mesh-master` with the JoltPhysics fork at `a40ff0b` (change 3) and `MaterialPairs`
(change 4). `verify.mjs` adds scene F (a bounce across two beams: the bounce is e·v and the rebound e²·h, also off two
loose beams, where upstream gives 0.96·e·v and 4 % instead of 16 %) and scene G (material pairs: the table's 0.07
where both bodies have a material, Jolt's max 0.4 otherwise). Scene E's hash (`f7b0121879392938`) is unchanged:
nothing without a bouncing multi-body CCD impact or an installed table moves.

* Release: <https://github.com/CrispStrobe/JoltPhysics.js/releases/tag/v1.1.0-bw.3> (workflow run 39427519957; the
  branch run 39422167908 before the tag produced the same bytes)
* Tarball: `https://github.com/CrispStrobe/JoltPhysics.js/releases/download/v1.1.0-bw.3/jolt-physics-1.1.0-bw.3.tgz`
* sha256 `112f6281f8cddaa17927e63b2c9bae77f9f28c8b915ac8aeffac330d75df890c`
* npm integrity `sha512-xqXPa6PGTgd0I8j4BiAHmmTNQgrXqounlRkpQ7VeVre9/guWR/4SOyJcMynIOzXORE8aHqlw+M9bUhgQwhjjIQ==`
* Built from `a073cc3` with the JoltPhysics fork at `a40ff0b`.

## Release `v1.1.0-bw.4` (2026-10-20)

Built from `brickwright/gear-mesh-master` with the JoltPhysics fork at `1631b95` (change 5, `RackMeshConstraint`).
`verify.mjs` adds scene H (a chassis spinning at 3 rad/s or driving at 0.3 m/s: with the pinion held the rack stays put,
with the pinion turned 2 rad it travels r·θ; the RackAndPinionConstraint control slides 38 mm and -261 mm). Scene E's
hash is unchanged. The release notes carry the tarball's sha256 and npm integrity.



## Optional single-thread SIMD (1.1.0-bw.6)

`jolt-physics/wasm-simd-compat` adds a SIMD build of the same C++ source and
bindings without shared memory or pthreads. Baseline `wasm-compat` and all
multithread exports retain their names and build settings. A consumer must
select this optional flavor only on a SIMD-capable engine and retain baseline
fallback; this package does not change consumer selection automatically.

Qualification runs every original physical scene against both flavors and
requires the exact same non-gear regression hash. Browser measurements of the
same downloaded magnetic-chess mechanism showed approximately 2.06x faster
physics in WebKit and 1.77x in Chrome, with identical final body poses after
six simulated seconds. These are isolated physics measurements, not a claim
that every full application or moving program runs twice as fast.

## Retained distance-constraint impulse observation (1.1.0-bw.7 candidate)

The pinned C++ producer adds `DistanceConstraint.GetLastVelocityImpulse()`.
It retains the signed final velocity-solver impulse across position correction,
which can deactivate a unilateral limit and clear the existing live lambda.
The original getter, solver equations, warm starting and serialized solver state
are unchanged. The retained observation is cleared by setup/reset/restore;
sleeping constraints retain their last solve, and multiple collision substeps
are not summed. Consumers need a fresh active native step and its actual dt.

`brickwright/verify-distance-impulse.mjs` observes every 5 ms native step of a
four-rope suspension, checking gravity/drag/rope momentum balance and weight
support. The previous immutable bw.6 package is an explicit failing-readout
control. Both scalar and SIMD builds must preserve every actual body position,
rotation and velocity from their respective previous-package controls exactly.
The packed tarball repeats these checks, alongside all original gear, collision,
rack and regression-hash controls. No app dependency advances before the
producer, binding build, package and current app qualification pass.

## Relative rotor binding source candidate (not built or released)

The hinge binding candidate exposes `mReflectedRotorInertia` (float, default
zero), `SetReflectedRotorInertia(float)` (boolean), and float getters
`GetReflectedRotorInertia()`, `GetRotorAngularVelocity()` and
`GetTotalLambdaRotor()`. Reflected inertia is in kg m², scalar speed in rad/s,
and the signed current rotor impulse in N m s. Positive impulse applies a
negative body1 and positive body2 angular impulse and decreases scalar speed.
The scalar is native constraint state, with no hidden body or JS correction.

The agreed native setter rejects negative/nonfinite values without changing
state. Enabling initializes scalar speed from the current relative hinge rate;
changing a positive inertia retains speed and clears its warm-start impulse;
an identical value retains state; disabling clears scalar speed and impulse.
The caller activates dynamic endpoints when configuration should wake them.
Invalid settings must refuse creation. Native recorder state includes inertia,
speed and warm-start impulse; no JS configuration replay replaces that state.

`brickwright/verify-rotor-inertia.mjs` prepares independent unequal-inertia
torque, momentum and energy/work checks for free and clamped pairs in both
endpoint orders, reversal and variable dt. It also prepares default-zero exact
trajectories, invalid/same-value/configuration/disable/reenable checks, exact
native recorder restoration (including shared rotors), and offset anisotropic
bodies joined to an ordinary transverse hinge. The latter checks anchor-point
velocities and locked angular components that a JS common-rate kick violated.
Body masses and inertia tensors must remain unchanged. The existing release
workflow checks built and packed scalar/SIMD modules, retaining every existing
gear/rope/filter and regression-hash guard.

Each consumer writes a distinct `bw.relative-rotor.trace.v2` JSON observation
with at most 200 single-step samples per case. Initial body/hinge parameters,
actual before/after poses, rates, inertia, native scalar/impulses and commands
allow separate analytic and constraint auditing. Configuration and recorder
boundaries are explicit events. Neighboring-joint cases include paired Jr0
baselines; restore cases retain genuinely different intervening configurations.
Actual checkout/event/native identities, verifier/module/archive hashes, Node
and package versions, phase and flavor distinguish all four consumers. These
claims require independent CI/source/package admission; self-reported expected
values and booleans are not sufficient evidence. The separate observations
artifact retains partial first-failure traces marked `complete:false`; these
are diagnostics and cannot qualify a consumer. Existing package artifacts and
hash guards retain their original contents.

Packed and built rotor fixtures use the shipped Lite motor iteration boundary:
30 velocity iterations and five initial position iterations. Actual native
override getters and global physics settings are retained in the observations;
the fixtures do not qualify higher-iteration operation. All original precision
thresholds and commands remain. Global and body sleeping are disabled, matching
the current Lite core default. These scenes do not qualify arbitrary sleeping
or out-of-band body-rate changes. Disable/reenable explicitly resynchronizes
the scalar from the actual relative rate and is a recorded control, rather
than an implicit per-step reset. Energy/work checks apply to the active,
constant-inertia intervals, not across configuration changes or every possible
external API lifetime.

Velocity-constraint residuals use the native solve's pre-integration hinge axes
and anchor arms with solved velocities. Geometric gaps, body momentum and energy
use the actual post-step pose. Evaluating the solved Jacobian at the later pose
would conflate integration/frame drift with a velocity-constraint violation.

This checkpoint deliberately retains native pin
`84f66c4dd4cb18f4c7be1bfdedb3d9f6d86ac5ff`, package version and toolchain pins.
That native revision does not yet implement this API: this binding checkpoint
is not a buildable combined release. Only a separately qualified landed native
revision supplied for integration may replace it. No generated glue/types,
native execution, packed bytes, release or all-model acceptance is claimed by
source/syntax review. Hosted qualification must admit the actual combined
source and repeated built/packed controls before any release or consumer pin.
