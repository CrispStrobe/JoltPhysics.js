// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 CrispStrobe
// Run against BOTH built and packed scalar/SIMD bytes. No JS velocity correction.
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
const modulePath = process.argv[2];
assert.ok(modulePath, 'usage: node brickwright/verify-rotor-inertia.mjs module.js');
const J = await (await import(pathToFileURL(modulePath).href)).default();
const DT = .005, JR = .003, TORQUE = .001;
const xyz = v => [v.GetX(), v.GetY(), v.GetZ()];
const quat = q => [q.GetX(), q.GetY(), q.GetZ(), q.GetW()];
const add = (a, b) => a.map((v, i) => v + b[i]);
const sub = (a, b) => a.map((v, i) => v - b[i]);
const mul = (a, k) => a.map(v => v * k);
const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const norm = a => Math.hypot(...a);
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const rotate = (q, v) => {const t = mul(cross(q, v), 2); return add(add(v, mul(t, q[3])), cross(q, t));};
const near = (a, b, tolerance, message) => assert.ok(Math.abs(a-b) <= tolerance, `${message}: ${a} vs ${b}`);
const results = [];

function world() {
    const settings = new J.JoltSettings();
    const pairs = new J.ObjectLayerPairFilterTable(1); // Deliberately no contacts in analytic scenes.
    const broad = new J.BroadPhaseLayerInterfaceTable(1, 1);
    broad.MapObjectToBroadPhaseLayer(0, new J.BroadPhaseLayer(0));
    settings.mObjectLayerPairFilter = pairs;
    settings.mBroadPhaseLayerInterface = broad;
    settings.mObjectVsBroadPhaseLayerFilter = new J.ObjectVsBroadPhaseLayerFilterTable(broad, 1, pairs, 1);
    const ji = new J.JoltInterface(settings); J.destroy(settings);
    const ps = ji.GetPhysicsSystem(), bi = ps.GetBodyInterface(), bodies = [], hinges = [];
    const gravity = new J.Vec3(0, 0, 0); ps.SetGravity(gravity); J.destroy(gravity);
    function body({position = [0,0,0], inertia = [.002,.002,.002], mass = .1, fixed = false, rotation = [0,0,0,1]} = {}) {
        const shape = new J.SphereShape(.004), p = new J.RVec3(...position), q = new J.Quat(...rotation);
        const settings = new J.BodyCreationSettings(shape, p, q, fixed ? J.EMotionType_Static : J.EMotionType_Dynamic, 0);
        settings.mAllowSleeping = false;
        settings.mLinearDamping = 0; settings.mAngularDamping = 0;
        settings.mMaxAngularVelocity = 10000;
        if (!fixed) {
            settings.mOverrideMassProperties = J.EOverrideMassProperties_CalculateInertia;
            const mp = settings.mMassPropertiesOverride; mp.mMass = mass; settings.mMassPropertiesOverride = mp;
        }
        const b = bi.CreateBody(settings); J.destroy(settings); J.destroy(p); J.destroy(q);
        if (!fixed) {
            const inv = new J.Vec3(...inertia.map(i => 1/i)), frame = new J.Quat(0,0,0,1);
            b.GetMotionProperties().SetInverseInertia(inv, frame); J.destroy(inv); J.destroy(frame);
            b.SetApplyGyroscopicForce(true);
        }
        bi.AddBody(b.GetID(), fixed ? J.EActivation_DontActivate : J.EActivation_Activate);
        const item = {b, inertia, mass, fixed}; bodies.push(item); return item;
    }
    function hinge(a, b, point = [0,0,0], axis = [0,1,0], jr) {
        const s = new J.HingeConstraintSettings();
        assert.equal(s.mReflectedRotorInertia, 0, 'settings default is disabled');
        s.mSpace = J.EConstraintSpace_WorldSpace;
        const p = new J.RVec3(...point), ax = new J.Vec3(...axis);
        const normal = new J.Vec3(...(Math.abs(axis[0]) < .9 ? cross(axis,[1,0,0]) : cross(axis,[0,1,0])));
        s.mPoint1 = p; s.mPoint2 = p; s.mHingeAxis1 = ax; s.mHingeAxis2 = ax;
        s.mNormalAxis1 = normal; s.mNormalAxis2 = normal;
        s.mNumVelocityStepsOverride = 64; s.mNumPositionStepsOverride = 16;
        if (jr !== undefined) s.mReflectedRotorInertia = jr;
        const constraint = s.Create(a.b, b.b);
        J.destroy(p); J.destroy(ax); J.destroy(normal); J.destroy(s);
        assert.ok(J.getPointer(constraint) !== 0, 'valid settings create a real constraint');
        const h = J.castObject(constraint, J.HingeConstraint); ps.AddConstraint(h);
        for (const name of ['SetReflectedRotorInertia','GetReflectedRotorInertia','GetRotorAngularVelocity','GetTotalLambdaRotor']) assert.equal(typeof h[name], 'function', `native API ${name}`);
        hinges.push({h,a,b,axis}); return h;
    }
    function configure(h, jr) {
        assert.equal(h.SetReflectedRotorInertia(jr), true);
        // Native setters intentionally do not wake bodies; this is caller ownership.
        for (const item of bodies) if (!item.fixed) bi.ActivateBody(item.b.GetID());
    }
    function motor(h, sign = 1) {
        const m = h.GetMotorSettings(); m.mMinTorqueLimit = -TORQUE; m.mMaxTorqueLimit = TORQUE;
        h.SetMotorState(J.EMotorState_Velocity); h.SetTargetAngularVelocity(sign * 10000);
    }
    const snapshot = () => bodies.map(({b}) => [...xyz(b.GetPosition()), ...quat(b.GetRotation()), ...xyz(b.GetLinearVelocity()), ...xyz(b.GetAngularVelocity())]);
    const tensorIdentity = () => bodies.filter(b => !b.fixed).map(({b}) => {
        const p = b.GetMotionProperties();
        return [p.GetInverseMass(), ...xyz(p.GetInverseInertiaDiagonal()), ...quat(p.GetInertiaRotation())];
    });
    const momentum = () => bodies.reduce((total, {b, mass, inertia, fixed}) => {
        if (fixed) return total;
        const q = quat(b.GetRotation()), w = xyz(b.GetAngularVelocity());
        const local = rotate([-q[0],-q[1],-q[2],q[3]], w);
        const spin = rotate(q, local.map((v,i) => v*inertia[i]));
        return add(total, add(spin, cross(xyz(b.GetCenterOfMassPosition()), mul(xyz(b.GetLinearVelocity()),mass))));
    }, [0,0,0]);
    const step = (count = 1, dt = DT) => {for (let i=0; i<count; i++) ji.Step(dt,1);};
    return {ji,ps,bi,bodies,hinges,body,hinge,configure,motor,snapshot,tensorIdentity,momentum,step,destroy:()=>J.destroy(ji)};
}
function scene(name, run) {
    const w = world();
    try {const result = run(w); results.push({name, ...result});} finally {w.destroy();}
}
function principal(w, {swap = false, fixed = false, jr = JR} = {}) {
    const a = w.body({inertia:[.002,.002,.002], fixed});
    const b = w.body({inertia:[.0007,.0007,.0007]});
    const h = swap ? w.hinge(b,a,[0,0,0],[0,1,0],jr) : w.hinge(a,b,[0,0,0],[0,1,0],jr);
    return {a:swap?b:a,b:swap?a:b,h};
}

for (const swap of [false,true]) for (const fixed of [false,true]) scene(`analytic-${fixed?'clamped':'free'}-${swap?'reverse-bodies':'forward'}`, w => {
    const {a,b,h} = principal(w,{swap,fixed});
    const original = w.tensorIdentity();
    const i1 = a.fixed ? Infinity : a.inertia[1], i2 = b.fixed ? Infinity : b.inertia[1];
    const mu = fixed ? (a.fixed ? i2 : i1) : 1/(1/i1+1/i2);
    let elapsed=0, target=0, work=0, before=0, minLambda=Infinity, maxLambda=-Infinity;
    for (const sign of [1,-1]) {
        w.motor(h,sign);
        for (let n=0;n<80;n++) {
            const dt = n % 2 ? DT : DT/2; // Exercise accumulated warm-start rescaling.
            w.step(1,dt); elapsed+=dt;
            const rate = xyz(b.b.GetAngularVelocity())[1]-xyz(a.b.GetAngularVelocity())[1];
            target += sign*TORQUE*dt/(mu+JR);
            near(rate,target,2e-5,'independent relative-inertia torque solve');
            near(h.GetRotorAngularVelocity(),rate,2e-5,'native scalar tracks relative rate');
            work+=sign*TORQUE*dt*(before+rate)/2;before=rate;
            const wa=xyz(a.b.GetAngularVelocity())[1],wb=xyz(b.b.GetAngularVelocity())[1];
            const kinetic=(a.fixed?0:i1*wa*wa/2)+(b.fixed?0:i2*wb*wb/2)+JR*h.GetRotorAngularVelocity()**2/2;
            near(kinetic,work,Math.max(1e-8,Math.abs(work)*.002),'motor work includes internal scalar energy');
            minLambda=Math.min(minLambda,h.GetTotalLambdaRotor());maxLambda=Math.max(maxLambda,h.GetTotalLambdaRotor());
            if (!fixed) assert.ok(norm(w.momentum())<2e-7,'bare body angular momentum stays internal');
        }
    }
    // Reversal leaves little net energy; check work identity at a fresh positive endpoint too.
    w.motor(h,1);w.step(80);
    const wa=xyz(a.b.GetAngularVelocity())[1],wb=xyz(b.b.GetAngularVelocity())[1],wr=h.GetRotorAngularVelocity();
    const energy=(a.fixed?0:i1*wa*wa/2)+(b.fixed?0:i2*wb*wb/2)+JR*wr*wr/2;
    const expected=.5*(mu+JR)*(TORQUE*.4/(mu+JR))**2;
    near(energy,expected,Math.max(1e-8,expected*.002),'independent kinetic energy');
    assert.deepEqual(w.tensorIdentity(),original,'rotor must not change mass or body inertia');
    assert.ok(minLambda<0&&maxLambda>0,'rotor impulse reverses sign with motor torque');
    return {elapsed,finalRelativeRate:wr,energy,expectedEnergy:expected,workDuringReversal:work};
});

scene('default-zero-exact-trajectory', w => {
    const a=w.body(),b=w.body({inertia:[.0007,.0007,.0007]});
    const h=w.hinge(a,b); // Leave the native settings default untouched in the first trajectory.
    assert.equal(h.GetReflectedRotorInertia(),0);w.motor(h);
    const recorder = new J.StateRecorderImpl();w.ps.SaveState(recorder,J.EStateRecorderState_All);
    const trace=[];for(let n=0;n<60;n++){w.step();trace.push(w.snapshot());}
    recorder.Rewind();assert.equal(w.ps.RestoreState(recorder),true);w.configure(h,0);
    const again=[];for(let n=0;n<60;n++){w.step();again.push(w.snapshot());}
    assert.deepEqual(again,trace,'explicit zero has no active scalar row');
    assert.equal(h.GetRotorAngularVelocity(),0);assert.equal(h.GetTotalLambdaRotor(),0);J.destroy(recorder);
    return {frames:trace.length,sha256:createHash('sha256').update(JSON.stringify(trace)).digest('hex')};
});

scene('configuration-invalid-atomic-and-native-restore', w => {
    const {a,b,h}=principal(w);w.motor(h);w.step(20);
    const nativeState=()=>[h.GetReflectedRotorInertia(),h.GetRotorAngularVelocity(),h.GetTotalLambdaRotor()];
    const before=nativeState();
    for(const bad of [-1,NaN,Infinity,-Infinity]) {assert.equal(h.SetReflectedRotorInertia(bad),false);assert.deepEqual(nativeState(),before);}
    w.configure(h,h.GetReflectedRotorInertia());assert.deepEqual(nativeState(),before,'same value preserves native scalar and lambda');
    const recorder=new J.StateRecorderImpl();w.ps.SaveState(recorder,J.EStateRecorderState_All);
    w.step(30);const expected=[w.snapshot(),nativeState()];
    const priorRate=h.GetRotorAngularVelocity();w.configure(h,JR*2);
    assert.equal(h.GetRotorAngularVelocity(),priorRate,'positive reconfiguration preserves scalar speed');
    assert.equal(h.GetTotalLambdaRotor(),0,'positive reconfiguration clears rotor warm-start impulse');
    w.step(10);w.configure(h,0);assert.equal(h.GetRotorAngularVelocity(),0);assert.equal(h.GetTotalLambdaRotor(),0);
    recorder.Rewind();assert.equal(w.ps.RestoreState(recorder),true);w.step(30);
    assert.deepEqual([w.snapshot(),nativeState()],expected,'recorder restores Jr, scalar and warm-start state');J.destroy(recorder);
    w.configure(h,0);const relative=xyz(b.b.GetAngularVelocity())[1]-xyz(a.b.GetAngularVelocity())[1];w.configure(h,JR);
    near(h.GetRotorAngularVelocity(),relative,2e-6,'enabling initializes current relative rate');
    return {invalidValues:4,restoredExactly:true};
});

scene('invalid-settings-create-refuses', w => {
    const a=w.body(),b=w.body();
    for(const bad of [-1,NaN,Infinity]) {
        const s=new J.HingeConstraintSettings();s.mReflectedRotorInertia=bad;
        const constraint=s.Create(a.b,b.b);assert.equal(J.getPointer(constraint),0,'invalid settings cannot create a native constraint');J.destroy(s);
    }
    return {invalidSettings:3};
});

for(const shared of [false,true]) scene(`neighboring-transverse-hinge-and-anchor-${shared?'shared-rotors':'one-rotor'}`, w => {
    const a=w.body({position:[0,0,0],inertia:[.002,.001,.003],rotation:[0,0,Math.sin(.3),Math.cos(.3)]});
    const b=w.body({position:[0,.04,0],inertia:[.0007,.0012,.0009]});
    const c=w.body({position:[.04,.04,0],inertia:[.0015,.002,.001]});
    const first=w.hinge(a,b,[0,.02,0],[0,1,0],JR);
    const second=w.hinge(b,c,[.02,.04,0],[1,0,0],shared?JR/2:0);
    w.motor(first);if(shared)w.motor(second,-1);
    const initial=w.tensorIdentity();let maxPointSpeed=0,maxLockedRate=0,maxPointGap=0;
    for(let n=0;n<80;n++) {
        w.step();assert.ok(norm(w.momentum())<3e-6,'bare multibody angular momentum');
        for(const {h,a,b,axis} of w.hinges) {
            const qa=quat(a.b.GetRotation()),qb=quat(b.b.GetRotation());
            const ra=rotate(qa,xyz(h.GetLocalSpacePoint1())),rb=rotate(qb,xyz(h.GetLocalSpacePoint2()));
            const pa=add(xyz(a.b.GetCenterOfMassPosition()),ra),pb=add(xyz(b.b.GetCenterOfMassPosition()),rb);
            const va=add(xyz(a.b.GetLinearVelocity()),cross(xyz(a.b.GetAngularVelocity()),ra));
            const vb=add(xyz(b.b.GetLinearVelocity()),cross(xyz(b.b.GetAngularVelocity()),rb));
            maxPointSpeed=Math.max(maxPointSpeed,norm(sub(vb,va)));maxPointGap=Math.max(maxPointGap,norm(sub(pb,pa)));
            const currentAxis=rotate(qa,xyz(h.GetLocalSpaceHingeAxis1()));
            const delta=sub(xyz(b.b.GetAngularVelocity()),xyz(a.b.GetAngularVelocity()));
            maxLockedRate=Math.max(maxLockedRate,norm(sub(delta,mul(currentAxis,dot(delta,currentAxis)))));
        }
    }
    assert.ok(maxPointSpeed<2e-4,'anchor-point velocity constraint survives native rotor solve');
    assert.ok(maxLockedRate<2e-4,'ordinary transverse hinge locked rates survive');
    assert.ok(maxPointGap<2e-5,'anchor positions remain joined');
    assert.deepEqual(w.tensorIdentity(),initial,'shared rotors never inflate body tensors');
    assert.ok(Math.abs(first.GetRotorAngularVelocity())>.001,'motor actually ran');
    if(shared) {
        const recorder=new J.StateRecorderImpl();w.ps.SaveState(recorder,J.EStateRecorderState_All);
        const state=()=>[w.snapshot(),...[first,second].map(h=>[h.GetReflectedRotorInertia(),h.GetRotorAngularVelocity(),h.GetTotalLambdaRotor()])];
        w.step(20);const expected=state();
        w.configure(first,JR*2);w.configure(second,0);w.step(5);
        recorder.Rewind();assert.equal(w.ps.RestoreState(recorder),true);w.step(20);
        assert.deepEqual(state(),expected,'shared configuration and scalar rows restore together');J.destroy(recorder);
    }
    return {shared,maxPointSpeed,maxLockedRate,maxPointGap};
});

console.log(JSON.stringify({schema:'bw.relative-rotor.verification.v1',modulePath,dtSeconds:DT,results,boundary:'Native scalar/constraint, analytic and recorder scenes only; existing gear/rope/non-gear guards remain separate. No physical-accuracy or all-model acceptance claim.'}));
