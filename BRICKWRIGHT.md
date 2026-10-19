# Brickwright fork of JoltPhysics.js

Branch `brickwright/gear-mesh-master` (not released): this branch with the Jolt pin moved to the JoltPhysics fork's
`brickwright/gear-mesh-master` (`7006be437d970eadd55c6454da9c46f169bce68b` = upstream master `01adafb` + the same gear
changes), to check the gear changes on current Jolt master. Binding changes: `PhysicsSettings.mDeterministicSimulation`
removed from `JoltJS.idl` (removed upstream; the simulation is always deterministic), `-DJPH_DEBUG` added to the Debug
glue defines (upstream no longer derives `JPH_DEBUG` from `NDEBUG`). The workflow runs the 1.1.0 regression-hash
comparison as its last step, so the gear assertions and the pack run even when upstream's non-gear changes since
v5.6.0 move that hash.

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
3. Package version `1.1.0-bw.1`; package name stays `jolt-physics`, so `import 'jolt-physics/wasm-compat'` is
   unchanged. All flavours are built by upstream's `build.sh`.

Files: `CMakeLists.txt` (FetchContent → fork commit), `JoltJS.idl` / `JoltJS.h` (bindings),
`brickwright/verify.mjs` (verification scenes), `.github/workflows/brickwright-release.yml`, `package.json`.

## JS API

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
