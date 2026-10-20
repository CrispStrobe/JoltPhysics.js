// Brickwright fork: gear verification scenes (see ../BRICKWRIGHT.md).
//
//   node brickwright/verify.mjs <path to jolt-physics.wasm-compat.js> [--json] [--assert]
//
// Runs the same scenes with every gear coupling the module offers:
//   gear  - GearConstraint, set up the way Brickwright's jolt-core.js does (|r| >= 1 ordering, no SetConstraints)
//   mesh  - GearMeshConstraint (fork only) with SetConstraints(hinge1, hinge2)
// and prints the measured numbers. --assert exits 1 when a fork expectation fails (used by CI).
import { pathToFileURL } from 'node:url';
import { createHash } from 'node:crypto';

const args = process.argv.slice(2);
const modPath = args.find(a => !a.startsWith('--'));
if (!modPath) { console.error('usage: node verify.mjs <jolt-physics.wasm-compat.js> [--json] [--assert]'); process.exit(2); }
const J = await (await import(pathToFileURL(modPath).href)).default();
const HAS_MESH = typeof J.GearMeshConstraintSettings === 'function';

const DT = 1 / 60;
const L_GHOST = 0, L_MOVING = 1, L_STATIC = 2;

// ---------------------------------------------------------------- small vector helpers
const v3 = v => [v.GetX(), v.GetY(), v.GetZ()];
const add = (a, b) => a.map((x, i) => x + b[i]);
const sub = (a, b) => a.map((x, i) => x - b[i]);
const scale = (a, s) => a.map(x => x * s);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => Math.hypot(...a);
const rotate = (q, v) => { // q = [x, y, z, w]
    const u = [q[0], q[1], q[2]];
    const t = scale(cross(u, v), 2);
    return add(add(v, scale(t, q[3])), cross(u, t));
};
const conj = q => [-q[0], -q[1], -q[2], q[3]];
const perpOf = a => { const c = cross(a, Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0]); return scale(c, 1 / norm(c)); };

// ---------------------------------------------------------------- world
function makeWorld(gravity = [0, 0, 0]) {
    const s = new J.JoltSettings();
    const olp = new J.ObjectLayerPairFilterTable(3);
    olp.EnableCollision(L_MOVING, L_MOVING);
    olp.EnableCollision(L_MOVING, L_STATIC);
    const bp = new J.BroadPhaseLayerInterfaceTable(3, 2);
    bp.MapObjectToBroadPhaseLayer(L_GHOST, new J.BroadPhaseLayer(0));
    bp.MapObjectToBroadPhaseLayer(L_MOVING, new J.BroadPhaseLayer(0));
    bp.MapObjectToBroadPhaseLayer(L_STATIC, new J.BroadPhaseLayer(1));
    s.mObjectLayerPairFilter = olp;
    s.mBroadPhaseLayerInterface = bp;
    s.mObjectVsBroadPhaseLayerFilter = new J.ObjectVsBroadPhaseLayerFilterTable(bp, 2, olp, 3);
    const ji = new J.JoltInterface(s);
    J.destroy(s);
    const ps = ji.GetPhysicsSystem();
    const set = new J.PhysicsSettings();
    set.mDeterministicSimulation = true;
    set.mAllowSleeping = false;
    ps.SetPhysicsSettings(set);
    J.destroy(set);
    ps.SetGravity(new J.Vec3(...gravity));
    const bi = ps.GetBodyInterface();
    const bodies = [];
    const w = {
        ps, bi, bodies,
        body({ pos, mass = 0.01, inertia = [1e-5, 1e-5, 1e-5], motion = 'dynamic', layer = L_GHOST, shape = null, rot = [0, 0, 0, 1] }) {
            const sh = shape || new J.SphereShape(0.004);
            const mt = motion === 'static' ? J.EMotionType_Static : J.EMotionType_Dynamic;
            const bcs = new J.BodyCreationSettings(sh, new J.RVec3(...pos), new J.Quat(...rot), mt, motion === 'static' && layer === L_GHOST ? L_GHOST : layer);
            if (mt === J.EMotionType_Dynamic) {
                bcs.mOverrideMassProperties = J.EOverrideMassProperties_CalculateInertia;
                const mp = bcs.mMassPropertiesOverride; mp.mMass = mass; bcs.mMassPropertiesOverride = mp;
                bcs.mLinearDamping = 0; bcs.mAngularDamping = 0; bcs.mMaxAngularVelocity = 1e4;
            }
            const b = bi.CreateBody(bcs);
            J.destroy(bcs);
            if (mt === J.EMotionType_Dynamic && inertia) b.GetMotionProperties().SetInverseInertia(new J.Vec3(...inertia.map(x => 1 / x)), new J.Quat(0, 0, 0, 1));
            bi.AddBody(b.GetID(), J.EActivation_Activate);
            const e = { b, mass, inertia, dynamic: mt === J.EMotionType_Dynamic };
            bodies.push(e);
            return e;
        },
        hinge(b1, b2, point, axis, { motorTorque = 0, steps = 0 } = {}) {
            const hs = new J.HingeConstraintSettings();
            hs.mSpace = J.EConstraintSpace_WorldSpace;
            hs.mPoint1 = new J.RVec3(...point); hs.mPoint2 = new J.RVec3(...point);
            hs.mHingeAxis1 = new J.Vec3(...axis); hs.mHingeAxis2 = new J.Vec3(...axis);
            const p = perpOf(axis);
            hs.mNormalAxis1 = new J.Vec3(...p); hs.mNormalAxis2 = new J.Vec3(...p);
            if (motorTorque) { hs.mMotorSettings.mMinTorqueLimit = -motorTorque; hs.mMotorSettings.mMaxTorqueLimit = motorTorque; }
            if (steps) { hs.mNumVelocityStepsOverride = steps; hs.mNumPositionStepsOverride = 10; }
            const c = hs.Create(b1.b, b2.b);
            J.destroy(hs);
            ps.AddConstraint(c);
            return J.castObject(c, J.HingeConstraint);
        },
        fixed(b1, b2) {
            const fs = new J.FixedConstraintSettings();
            fs.mAutoDetectPoint = true;
            const c = fs.Create(b1.b, b2.b);
            J.destroy(fs);
            ps.AddConstraint(c);
            return c;
        },
        step() { ji.Step(DT, 1); },
        stepDt(dt) { ji.Step(dt, 1); },
        // Total linear and angular momentum (about the origin) of the dynamic bodies
        momentum() {
            let P = [0, 0, 0], L = [0, 0, 0];
            for (const e of bodies) {
                if (!e.dynamic) continue;
                const x = v3(e.b.GetCenterOfMassPosition()), v = v3(e.b.GetLinearVelocity()), om = v3(e.b.GetAngularVelocity());
                const q = e.b.GetRotation(); const qq = [q.GetX(), q.GetY(), q.GetZ(), q.GetW()];
                const local = rotate(conj(qq), om).map((x, i) => x * e.inertia[i]);
                P = add(P, scale(v, e.mass));
                L = add(L, add(cross(x, scale(v, e.mass)), rotate(qq, local)));
            }
            return { P, L };
        },
        stateHash() {
            const h = createHash('sha256');
            for (const e of bodies) {
                const x = e.b.GetCenterOfMassPosition(), q = e.b.GetRotation(), v = e.b.GetLinearVelocity(), om = e.b.GetAngularVelocity();
                h.update(new Float64Array([x.GetX(), x.GetY(), x.GetZ(), q.GetX(), q.GetY(), q.GetZ(), q.GetW(), v.GetX(), v.GetY(), v.GetZ(), om.GetX(), om.GetY(), om.GetZ()]));
            }
            return h.digest('hex').slice(0, 16);
        },
        destroy() { J.destroy(ji); },
    };
    return w;
}

/**
 * A gear coupling between gear bodies g1, g2 held by hinges h1, h2 (gear = hinge body2, frame = body1):
 * w2 = ratio * w1 (rates relative to the frame about axis1 / axis2).
 */
function couple(w, kind, { g1, g2, h1, h2, c1, c2, axis1, axis2, ratio, contact = [0, 0, 0], steps = 30 }) {
    if (kind === 'gear') {
        // jolt-core.js buildGearConstraint: |r| >= 1 ordering, positive ratio by flipping axis 2
        let A = { g: g1, axis: axis1 }, B = { g: g2, axis: axis2 }, r = ratio;
        if (Math.abs(r) > 1) { [A, B] = [B, A]; r = 1 / r; }
        const gs = new J.GearConstraintSettings();
        gs.mSpace = J.EConstraintSpace_WorldSpace;
        const flip = r > 0 ? -1 : 1;
        gs.mHingeAxis1 = new J.Vec3(...A.axis);
        gs.mHingeAxis2 = new J.Vec3(...B.axis.map(v => v * flip));
        gs.mRatio = 1 / Math.abs(r);
        gs.mNumVelocityStepsOverride = steps;
        gs.mNumPositionStepsOverride = 10;
        const c = gs.Create(A.g.b, B.g.b);
        J.destroy(gs);
        w.ps.AddConstraint(c);
        return c;
    }
    const ms = new J.GearMeshConstraintSettings();
    ms.mSpace = J.EConstraintSpace_WorldSpace;
    ms.mPoint1 = new J.RVec3(...c1); ms.mHingeAxis1 = new J.Vec3(...axis1);
    ms.mPoint2 = new J.RVec3(...c2); ms.mHingeAxis2 = new J.Vec3(...axis2);
    ms.mContactPoint = new J.RVec3(...contact);
    ms.mRatio = ratio;
    ms.mNumVelocityStepsOverride = steps;
    ms.mNumPositionStepsOverride = 10;
    const c = J.castObject(ms.Create(g1.b, g2.b), J.GearMeshConstraint);
    J.destroy(ms);
    w.ps.AddConstraint(c);
    if (h1 && h2) c.SetConstraints(h1, h2);
    return c;
}

// ---------------------------------------------------------------- scenes
const kinds = HAS_MESH ? ['gear', 'mesh'] : ['gear'];
const Y = [0, 1, 0], Z = [0, 0, 1], X = [1, 0, 0];
const results = {};

/** A: two free discs 8:24 on a static frame, torque on the small one. */
function sceneDiscs(kind) {
    const w = makeWorld();
    const I1 = 1e-4, I2 = 9e-4, tau = 1e-3, T = 1;
    const frame = w.body({ pos: [0, -0.05, 0], motion: 'static' });
    const c1 = [0, 0, 0], c2 = [0.016, 0, 0];
    const g1 = w.body({ pos: c1, mass: 0.01, inertia: [I1, I1, I1] });
    const g2 = w.body({ pos: c2, mass: 0.03, inertia: [I2, I2, I2] });
    const h1 = w.hinge(frame, g1, c1, Y), h2 = w.hinge(frame, g2, c2, Y);
    couple(w, kind, { g1, g2, h1, h2, c1, c2, axis1: Y, axis2: Y, ratio: -1 / 3 });
    let work = 0, w1 = 0;
    for (let t = 0; t < T / DT - 0.5; t++) {
        g1.b.AddTorque(new J.Vec3(0, tau, 0));
        w.step();
        const n1 = g1.b.GetAngularVelocity().GetY();
        work += tau * 0.5 * (w1 + n1) * DT; w1 = n1;
    }
    const om1 = g1.b.GetAngularVelocity().GetY(), om2 = g2.b.GetAngularVelocity().GetY();
    const ke = 0.5 * I1 * om1 * om1 + 0.5 * I2 * om2 * om2;
    w.destroy();
    return { alpha1: om1 / T, alphaExpected: tau / (I1 + I2 / 9), alphaBug: tau / (I1 + I2 / 3), ratio: om2 / om1, work, ke, energyErr: (ke - work) / work };
}

/** B: free-floating frame, two meshed gears, motor inside (hinge 1) drives gear 1. Total L must stay 0. */
function sceneFloating(kind, worm = false) {
    const w = makeWorld();
    const frame = w.body({ pos: [0.01, 0.002, 0.003], mass: 0.05, inertia: [2e-5, 3e-5, 2.5e-5] });
    let c1, c2, a1, a2, ratio, contact;
    if (!worm) { c1 = [0, 0, 0]; c2 = [0.016, 0, 0]; a1 = Y; a2 = Y; ratio = -1 / 3; contact = [0, 0, 0]; }
    else { c1 = [0, 0, 0]; c2 = [0, -0.017, 0]; a1 = X; a2 = Z; ratio = 1 / 24; contact = [0, -0.005, 0]; }
    const g1 = w.body({ pos: c1, mass: 0.005, inertia: [3e-5, 3e-5, 3e-5] });
    const g2 = w.body({ pos: c2, mass: 0.01, inertia: [9e-5, 9e-5, 9e-5] });
    const h1 = w.hinge(frame, g1, c1, a1, { motorTorque: 0.05, steps: 30 }), h2 = w.hinge(frame, g2, c2, a2);
    couple(w, kind, { g1, g2, h1, h2, c1, c2, axis1: a1, axis2: a2, ratio, contact });
    h1.SetMotorState(J.EMotorState_Velocity);
    let maxL = 0, maxScale = 0, ratioMeas = 0;
    const steps = 2 / DT;
    for (let t = 0; t < steps; t++) {
        h1.SetTargetAngularVelocity(10 * Math.min(1, t / 30));
        w.step();
        const { L } = w.momentum();
        maxL = Math.max(maxL, norm(L));
        const Om = v3(frame.b.GetAngularVelocity());
        const w1 = dot(sub(v3(g1.b.GetAngularVelocity()), Om), rotate(qOf(frame.b), a1));
        const w2 = dot(sub(v3(g2.b.GetAngularVelocity()), Om), rotate(qOf(frame.b), a2));
        maxScale = Math.max(maxScale, 3e-5 * Math.abs(w1));
        if (t === steps - 1) ratioMeas = w2 / w1;
    }
    // Frame yaw about the gear axis (the "spinning excavator")
    const q = qOf(frame.b);
    const yaw = 2 * Math.atan2(dot([q[0], q[1], q[2]], a1), q[3]);
    w.destroy();
    return { maxL, gearL: maxScale, relL: maxL / maxScale, ratio: ratioMeas, ratioExpected: ratio, frameTurnDeg: yaw * 180 / Math.PI };
}
const qOf = b => { const q = b.GetRotation(); return [q.GetX(), q.GetY(), q.GetZ(), q.GetW()]; };

/**
 * C: static balance on a 1:3 train (axes horizontal, static frame). Each gear carries a drum of radius L with a
 * weight hanging from it (a constant torque m g L, applied as a torque): m1 on the 8T, m2 on the 24T, both pulling
 * the same way round. The train balances at m2 = 3 m1. Gravity also pulls on the gear bodies (on their axes).
 */
function sceneBalance(kind, m2factor) {
    const w = makeWorld([0, -9.81, 0]);
    const frame = w.body({ pos: [0, -0.05, 0], motion: 'static' });
    const c1 = [0, 0, 0], c2 = [0.016, 0, 0], m1 = 0.01, L = 0.02, g = 9.81;
    const g1 = w.body({ pos: c1, mass: 0.005, inertia: [3e-5, 3e-5, 3e-5] });
    const g2 = w.body({ pos: c2, mass: 0.01, inertia: [3e-5, 3e-5, 3e-5] });
    const h1 = w.hinge(frame, g1, c1, Z), h2 = w.hinge(frame, g2, c2, Z);
    couple(w, kind, { g1, g2, h1, h2, c1, c2, axis1: Z, axis2: Z, ratio: -1 / 3 });
    let maxA = 0;
    for (let t = 0; t < 2 / DT; t++) {
        g1.b.AddTorque(new J.Vec3(0, 0, -m1 * g * L));
        g2.b.AddTorque(new J.Vec3(0, 0, -m1 * m2factor * g * L));
        w.step();
        maxA = Math.max(maxA, Math.abs(h1.GetCurrentAngle()));
    }
    const out = { angle1Deg: h1.GetCurrentAngle() * 180 / Math.PI, maxAngle1Deg: maxA * 180 / Math.PI };
    w.destroy();
    return out;
}

/** D: gear 1 held by its motor (velocity 0), constant torque on gear 2 for 5 s: phase drift of the mesh. */
function sceneHeld(kind) {
    const w = makeWorld();
    const frame = w.body({ pos: [0, -0.05, 0], motion: 'static' });
    const c1 = [0, 0, 0], c2 = [0.016, 0, 0];
    const g1 = w.body({ pos: c1, mass: 0.005, inertia: [3e-5, 3e-5, 3e-5] });
    const g2 = w.body({ pos: c2, mass: 0.01, inertia: [3e-5, 3e-5, 3e-5] });
    const h1 = w.hinge(frame, g1, c1, Y, { motorTorque: 10, steps: 30 }), h2 = w.hinge(frame, g2, c2, Y);
    couple(w, kind, { g1, g2, h1, h2, c1, c2, axis1: Y, axis2: Y, ratio: -1 / 3 });
    h1.SetMotorState(J.EMotorState_Velocity); h1.SetTargetAngularVelocity(0);
    let a1 = 0, a2 = 0, p1 = 0, p2 = 0;
    const unwrap = (acc, prev, cur) => acc + Math.atan2(Math.sin(cur - prev), Math.cos(cur - prev));
    for (let t = 0; t < 5 / DT; t++) {
        g2.b.AddTorque(new J.Vec3(0, 0.05, 0));
        w.step();
        const n1 = h1.GetCurrentAngle(), n2 = h2.GetCurrentAngle();
        a1 = unwrap(a1, p1, n1); a2 = unwrap(a2, p2, n2); p1 = n1; p2 = n2;
    }
    w.destroy();
    return { gear1Deg: a1 * 180 / Math.PI, gear2Deg: a2 * 180 / Math.PI, phaseErrDeg2: (a2 - (-1 / 3) * a1) * 180 / Math.PI };
}

/** E: unchanged physics: rack and pinion, a hinge pendulum and a contact pile, hashed state after 2 s. */
function sceneRegression() {
    const w = makeWorld([0, -9.81, 0]);
    const floor = w.body({ pos: [0, -0.5, 0], motion: 'static', layer: L_STATIC, shape: new J.BoxShape(new J.Vec3(1, 0.1, 1)) });
    for (let i = 0; i < 6; i++) w.body({ pos: [0.3 + 0.01 * i, -0.35 + 0.045 * i, 0.002 * i], mass: 0.1, inertia: null, layer: L_MOVING, shape: new J.BoxShape(new J.Vec3(0.02, 0.02, 0.02)) });
    const pend = w.body({ pos: [0.1, 0.2, 0], mass: 0.05, inertia: [1e-4, 1e-4, 1e-4] });
    w.hinge(floor, pend, [0, 0.2, 0], Z);
    // Rack and pinion on a static frame: pinion r = 8 mm, torque driven
    const pin = w.body({ pos: [-0.3, 0, 0], mass: 0.005, inertia: [3e-5, 3e-5, 3e-5] });
    const rack = w.body({ pos: [-0.3, -0.008, 0], mass: 0.02, inertia: [1e-5, 1e-5, 1e-5] });
    const hp = w.hinge(floor, pin, [-0.3, 0, 0], Z);
    const ss = new J.SliderConstraintSettings();
    ss.mAutoDetectPoint = true; ss.mSliderAxis1 = new J.Vec3(1, 0, 0); ss.mSliderAxis2 = new J.Vec3(1, 0, 0);
    ss.mNormalAxis1 = new J.Vec3(0, 1, 0); ss.mNormalAxis2 = new J.Vec3(0, 1, 0);
    const sl = J.castObject(ss.Create(floor.b, rack.b), J.SliderConstraint); w.ps.AddConstraint(sl); J.destroy(ss);
    const rs = new J.RackAndPinionConstraintSettings();
    rs.mSpace = J.EConstraintSpace_WorldSpace; rs.mHingeAxis = new J.Vec3(0, 0, 1); rs.mSliderAxis = new J.Vec3(-1, 0, 0); rs.mRatio = 1 / 0.008;
    const rc = J.castObject(rs.Create(pin.b, rack.b), J.RackAndPinionConstraint); w.ps.AddConstraint(rc); J.destroy(rs);
    // No SetConstraints, as jolt-core.js (its drift correction assumes the hinge/slider axes match mHingeAxis/mSliderAxis signs)
    let ang = 0, prev = 0;
    for (let t = 0; t < 2 / DT; t++) {
        pin.b.AddTorque(new J.Vec3(0, 0, 1e-3));
        w.step();
        const n = hp.GetCurrentAngle(); ang += Math.atan2(Math.sin(n - prev), Math.cos(n - prev)); prev = n;
    }
    const out = { hash: w.stateHash(), rackX: sl.GetCurrentPosition(), pinionRad: ang, rackPerRad: sl.GetCurrentPosition() / ang };
    w.destroy();
    return out;
}

/**
 * F: continuous collision with a bounce (fork change 3, BRICKWRIGHT.md). A 2 × 4 brick-sized box (32 × 9.6 × 16 mm,
 * 2.7 g) falls 20 cm flat onto two 8 mm beams 16 mm apart (static, or two loose dynamic ones on a floor), restitution
 * 0.4 on everything, LinearCast, 200 Hz: it hits both at once. Upstream resolves the impact against the one beam the
 * cast hit first, off the brick's centre: it spins. The fork leaves the bounce to the next discrete step: straight up.
 * Also on a single floor (control: both builds bounce straight) and with restitution 0 (both stop dead).
 */
function sceneCCDBounce({ restitution = 0.4, twoBeams = true, dynamicBeams = false, dropM = 0.2 } = {}) {
    const w = makeWorld([0, -9.81, 0]);
    const ps = new J.PhysicsSettings();
    ps.mAllowSleeping = false; ps.mSpeculativeContactDistance = 0.002; ps.mPenetrationSlop = 0.0002; ps.mLinearCastThreshold = 0.25;
    w.ps.SetPhysicsSettings(ps); J.destroy(ps);
    const mk = (pos, half, motion, mass) => {
        const sh = new J.BoxShape(new J.Vec3(...half), Math.min(0.0005, 0.25 * Math.min(...half)));
        const mt = motion === 'static' ? J.EMotionType_Static : J.EMotionType_Dynamic;
        const bcs = new J.BodyCreationSettings(sh, new J.RVec3(...pos), new J.Quat(0, 0, 0, 1), mt, motion === 'static' ? L_STATIC : L_MOVING);
        bcs.mRestitution = restitution;
        if (mt === J.EMotionType_Dynamic) {
            bcs.mMotionQuality = J.EMotionQuality_LinearCast;
            bcs.mOverrideMassProperties = J.EOverrideMassProperties_CalculateInertia;
            const mp = bcs.mMassPropertiesOverride; mp.mMass = mass; bcs.mMassPropertiesOverride = mp;
        }
        const b = w.bi.CreateBody(bcs); J.destroy(bcs); w.bi.AddBody(b.GetID(), J.EActivation_Activate);
        return b;
    };
    mk([0, -0.01, 0], [0.5, 0.01, 0.5], 'static');
    const beamHalf = [0.06, 0.004, 0.004];
    if (twoBeams) for (const z of [-0.008, 0.008]) mk([0, 0.004, z], beamHalf, dynamicBeams ? 'dynamic' : 'static', 0.0048);
    const top = twoBeams ? 0.008 : 0;
    const brick = mk([0, top + 0.0048 + dropM, 0], [0.016, 0.0048, 0.008], 'dynamic', 0.0027);
    let vIn = 0, vOut = null, maxSpin = 0, prevV = 0, apex = -Infinity, lowest = Infinity;
    const DT200 = 1 / 200;
    for (let i = 0; i < 1.2 / DT200; i++) {
        w.stepDt(DT200); // 200 Hz, as Brickwright steps
        const v = v3(brick.GetLinearVelocity())[1];
        const om = norm(v3(brick.GetAngularVelocity()));
        const y = v3(brick.GetCenterOfMassPosition())[1];
        if (vOut === null && v > 0) { vOut = v; vIn = -prevV; }
        if (vOut !== null) { maxSpin = Math.max(maxSpin, om); if (v > 0 || apex === -Infinity) apex = Math.max(apex, y); }
        lowest = Math.min(lowest, y);
        prevV = v;
    }
    const q = brick.GetRotation();
    const tiltDeg = 2 * Math.acos(Math.min(1, Math.abs(q.GetW()))) * 180 / Math.PI;
    const out = { restitution, twoBeams, dynamicBeams, dropM, bounce: vOut === null ? 0 : vOut / vIn, maxSpin, tiltDeg, rebound: vOut === null ? 0 : (apex - lowest) / dropM };
    w.destroy();
    return out;
}

/**
 * G: restitution per material pair (fork change 4, JoltJS.h MaterialPairs). A body's material is the low 8 bits of its
 * user data. A box of material 1 (e 0.4) dropped 20 cm on a floor of material 2 (e 0.05): Jolt's max rule bounces it at
 * 0.4; with the pair (1, 2) = 0.07 in the table and the table installed, at 0.07. Bodies without a material: max rule.
 */
function sceneMaterialPairs({ install = true, userData = true } = {}) {
    const HAS_PAIRS = typeof J.MaterialPairs === 'function';
    if (!HAS_PAIRS) return { skipped: true };
    const w = makeWorld([0, -9.81, 0]);
    J.MaterialPairs.prototype.sClear();
    J.MaterialPairs.prototype.sSetRestitution(1, 2, 0.07);
    if (install) J.MaterialPairs.prototype.sInstall(w.ps);
    const mk = (pos, half, dyn, e, material) => {
        const bcs = new J.BodyCreationSettings(new J.BoxShape(new J.Vec3(...half)), new J.RVec3(...pos), new J.Quat(0, 0, 0, 1), dyn ? J.EMotionType_Dynamic : J.EMotionType_Static, dyn ? L_MOVING : L_STATIC);
        bcs.mRestitution = e;
        if (userData) bcs.mUserData = material;
        const b = w.bi.CreateBody(bcs); J.destroy(bcs); w.bi.AddBody(b.GetID(), J.EActivation_Activate);
        return b;
    };
    mk([0, -0.1, 0], [1, 0.1, 1], false, 0.05, 2);
    const box = mk([0, 0.25, 0], [0.05, 0.05, 0.05], true, 0.4, 1);
    let prev = 0, bounce = null;
    for (let i = 0; i < 0.5 / DT && bounce === null; i++) {
        w.step();
        const v = box.GetLinearVelocity().GetY();
        if (v > 0) bounce = v / -prev;
        prev = v;
    }
    J.MaterialPairs.prototype.sClear();
    w.destroy();
    return { install, userData, bounce, lookup: J.MaterialPairs.prototype.sGetRestitution(1, 2) };
}

/**
 * H: rack and pinion on a moving frame (fork change 5, RackMeshConstraint). A 200 g chassis carries a pinion (hinge,
 * axis z) and a rack (slider along x), free in space. The chassis spins about z at 3 rad/s, or drives along x at
 * 0.3 m/s; the pinion is held to the chassis (hinge friction) or turned by its motor at 2 rad/s relative to it.
 * RackAndPinionConstraint ties world rates: the chassis's own motion drives the rack. RackMeshConstraint (with the
 * hinge and slider handed over) ties the rates relative to the chassis: the rack moves only by r times the pinion's turn.
 */
function sceneRackOnFrame(kind, mode, { motor = false } = {}) {
    const w = makeWorld([0, 0, 0]);
    const chassis = w.body({ pos: [0, 0, 0], mass: 0.2, inertia: [1e-4, 1e-4, 1e-4], layer: L_GHOST });
    const pinion = w.body({ pos: [0, 0.02, 0], mass: 0.02, inertia: [3e-5, 3e-5, 3e-5], layer: L_GHOST });
    const rack = w.body({ pos: [0, 0.012, 0], mass: 0.02, inertia: [1e-5, 1e-5, 1e-5], layer: L_GHOST });
    const hs = new J.HingeConstraintSettings();
    hs.mSpace = J.EConstraintSpace_WorldSpace;
    hs.mPoint1 = new J.RVec3(0, 0.02, 0); hs.mPoint2 = new J.RVec3(0, 0.02, 0);
    hs.mHingeAxis1 = new J.Vec3(0, 0, 1); hs.mHingeAxis2 = new J.Vec3(0, 0, 1);
    hs.mNormalAxis1 = new J.Vec3(1, 0, 0); hs.mNormalAxis2 = new J.Vec3(1, 0, 0);
    hs.mMaxFrictionTorque = 10;
    hs.mMotorSettings.mMinTorqueLimit = -10; hs.mMotorSettings.mMaxTorqueLimit = 10;
    const hinge = J.castObject(hs.Create(chassis.b, pinion.b), J.HingeConstraint);
    J.destroy(hs);
    w.ps.AddConstraint(hinge);
    if (motor) { hinge.SetMotorState(J.EMotorState_Velocity); hinge.SetTargetAngularVelocity(2); }
    const ss = new J.SliderConstraintSettings();
    ss.mAutoDetectPoint = true;
    ss.mSliderAxis1 = new J.Vec3(1, 0, 0); ss.mSliderAxis2 = new J.Vec3(1, 0, 0);
    ss.mNormalAxis1 = new J.Vec3(0, 1, 0); ss.mNormalAxis2 = new J.Vec3(0, 1, 0);
    const slider = J.castObject(ss.Create(chassis.b, rack.b), J.SliderConstraint);
    J.destroy(ss);
    w.ps.AddConstraint(slider);
    const r = 0.008; // pinion pitch radius: 1/r rad per metre
    if (kind === 'world') {
        const rs = new J.RackAndPinionConstraintSettings();
        rs.mSpace = J.EConstraintSpace_WorldSpace; rs.mHingeAxis = new J.Vec3(0, 0, 1); rs.mSliderAxis = new J.Vec3(1, 0, 0); rs.mRatio = 1 / r;
        w.ps.AddConstraint(rs.Create(pinion.b, rack.b)); J.destroy(rs);
    } else {
        const rs = new J.RackMeshConstraintSettings();
        rs.mSpace = J.EConstraintSpace_WorldSpace; rs.mHingeAxis = new J.Vec3(0, 0, 1); rs.mSliderAxis = new J.Vec3(1, 0, 0); rs.mRatio = 1 / r;
        const c = J.castObject(rs.Create(pinion.b, rack.b), J.RackMeshConstraint); J.destroy(rs);
        w.ps.AddConstraint(c);
        c.SetConstraints(hinge, slider);
    }
    const id = chassis.b.GetID();
    if (mode === 'spin') w.bi.SetAngularVelocity(id, new J.Vec3(0, 0, 3)); else w.bi.SetLinearVelocity(id, new J.Vec3(0.3, 0, 0));
    for (let i = 0; i < 1 / DT; i++) w.step();
    const out = { kind, mode, motor, slide: slider.GetCurrentPosition(), pinion: hinge.GetCurrentAngle(), P: w.momentum().P };
    out.expected = out.pinion * r;
    w.destroy();
    return out;
}

for (const k of kinds) {
    results[k] = {
        discs: sceneDiscs(k),
        floating: sceneFloating(k),
        balance3: sceneBalance(k, 3),
        balance1: sceneBalance(k, 1),
        held: sceneHeld(k),
    };
}
if (HAS_MESH) results.mesh.worm = sceneFloating('mesh', true);
results.regression = sceneRegression();
results.rackOnFrame = typeof J.RackMeshConstraintSettings === 'function' ? {
    worldSpin: sceneRackOnFrame('world', 'spin'),
    worldDrive: sceneRackOnFrame('world', 'drive'),
    meshSpin: sceneRackOnFrame('mesh', 'spin'),
    meshDrive: sceneRackOnFrame('mesh', 'drive'),
    meshSpinMotor: sceneRackOnFrame('mesh', 'spin', { motor: true }),
    meshDriveMotor: sceneRackOnFrame('mesh', 'drive', { motor: true }),
} : { skipped: true };
results.materialPairs = {
    table: sceneMaterialPairs(),
    notInstalled: sceneMaterialPairs({ install: false }),
    noMaterial: sceneMaterialPairs({ userData: false }),
};
results.ccdBounce = {
    twoStatic: sceneCCDBounce(),
    twoLoose: sceneCCDBounce({ dynamicBeams: true }),
    twoStatic1m: sceneCCDBounce({ dropM: 1 }),
    floor: sceneCCDBounce({ twoBeams: false }),
    inelastic: sceneCCDBounce({ restitution: 0 }),
};

if (args.includes('--json')) console.log(JSON.stringify({ module: modPath, hasMesh: HAS_MESH, results }, null, 1));
else {
    console.log(`module ${modPath} (GearMeshConstraint: ${HAS_MESH ? 'yes' : 'no'})`);
    for (const [k, r] of Object.entries(results)) console.log(k, JSON.stringify(r, (key, v) => typeof v === 'number' ? Number(v.toPrecision(5)) : v, 1));
}

if (args.includes('--assert')) {
    const fails = [];
    const ok = (cond, msg) => { if (!cond) fails.push(msg); };
    const near = (a, b, tol) => Math.abs(a - b) <= tol * Math.abs(b);
    for (const k of kinds) {
        const r = results[k];
        ok(near(r.discs.alpha1, r.discs.alphaExpected, 0.01), `${k}: discs alpha ${r.discs.alpha1} != ${r.discs.alphaExpected}`);
        ok(Math.abs(r.discs.energyErr) < 0.01, `${k}: discs energy error ${r.discs.energyErr}`);
        ok(Math.abs(r.balance3.maxAngle1Deg) < 1, `${k}: balance m2=3m1 moved ${r.balance3.maxAngle1Deg} deg`);
    }
    if (HAS_MESH) {
        const m = results.mesh;
        ok(m.floating.relL < 0.01, `mesh: floating frame |L| ${m.floating.relL} of the gear's`);
        ok(near(m.floating.ratio, -1 / 3, 0.01), `mesh: floating ratio ${m.floating.ratio}`);
        ok(Math.abs(m.held.phaseErrDeg2) < 0.5, `mesh: held phase error ${m.held.phaseErrDeg2} deg`);
        ok(Math.abs(m.held.gear1Deg) < 0.5, `mesh: held gear moved ${m.held.gear1Deg} deg`);
        ok(m.worm.relL < 0.01 && near(m.worm.ratio, 1 / 24, 0.02), `mesh: worm L ${m.worm.relL} ratio ${m.worm.ratio}`);
    }
    // Fork change 5 (RackMeshConstraint): the rack moves by r times the pinion's turn relative to the chassis, whatever
    // the chassis does (RackAndPinionConstraint's world rates do not: the controls slide by centimetres).
    if (typeof J.RackMeshConstraintSettings === 'function') {
        const k = results.rackOnFrame;
        for (const c of [k.meshSpin, k.meshDrive, k.meshSpinMotor, k.meshDriveMotor])
            ok(Math.abs(c.slide - c.expected) < 1e-4, `rackOnFrame mesh ${c.mode}${c.motor ? ' motor' : ''}: slide ${c.slide} m for r·θ ${c.expected}`);
        ok(Math.abs(k.meshSpinMotor.pinion) > 1 && Math.abs(k.meshDriveMotor.pinion) > 1, 'rackOnFrame: the motor turned the pinion');
        ok(Math.abs(k.worldSpin.slide - k.worldSpin.expected) > 1e-3 && Math.abs(k.worldDrive.slide - k.worldDrive.expected) > 1e-3,
            'rackOnFrame: the world-rate control slides (the scene tells them apart)');
    }
    // Fork change 4 (material pairs): the table's e where both bodies have a material and the table is installed.
    if (typeof J.MaterialPairs === 'function') {
        const m = results.materialPairs;
        ok(Math.abs(m.table.bounce - 0.07) < 0.01, `materialPairs: bounce ${m.table.bounce} for the pair's 0.07`);
        ok(Math.abs(m.notInstalled.bounce - 0.4) < 0.02, `materialPairs: not installed, bounce ${m.notInstalled.bounce} for max 0.4`);
        ok(Math.abs(m.noMaterial.bounce - 0.4) < 0.02, `materialPairs: no material, bounce ${m.noMaterial.bounce} for max 0.4`);
        ok(m.table.lookup === -1, 'materialPairs: sClear forgets the entries');
    }
    // Fork change 3 (CCD bounce, BRICKWRIGHT.md): every bounce leaves at e·v and rises to e²·h, also off two loose
    // beams (upstream: the first-hit beam took the whole CCD impulse, the brick left at ~0.96 of e·v and rose to 4 %).
    if (HAS_MESH) for (const [k, c] of Object.entries(results.ccdBounce)) {
        ok(c.maxSpin < 0.5, `ccdBounce ${k}: spin ${c.maxSpin} rad/s after the bounce`);
        if (c.restitution > 0) {
            ok(Math.abs(c.bounce - c.restitution) < 0.03, `ccdBounce ${k}: bounce ${c.bounce} for e ${c.restitution}`);
            ok(Math.abs(c.rebound / c.restitution ** 2 - 1) < 0.15, `ccdBounce ${k}: rebound ${c.rebound} of the drop for e² ${c.restitution ** 2}`);
        } else ok(c.rebound < 0.001, `ccdBounce ${k}: an inelastic impact rebounded ${c.rebound} of the drop`);
    }
    if (fails.length) { console.error('FAIL\n' + fails.join('\n')); process.exit(1); }
    console.log('ASSERT OK');
}
process.exit(0);
