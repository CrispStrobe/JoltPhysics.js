// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 CrispStrobe
// Run against BOTH built and packed scalar/SIMD bytes. No JS velocity correction.
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
import {readFileSync,writeFileSync} from 'node:fs';
import path from 'node:path';
import {execFileSync} from 'node:child_process';
const modulePath = process.argv[2];
assert.ok(modulePath, 'usage: node brickwright/verify-rotor-inertia.mjs module.js');
const option = name => process.argv.slice(3).find(a => a.startsWith(`--${name}=`))?.slice(name.length+3);
const phase=option('phase'),flavor=option('flavor'),out=option('out');
assert.ok(['built','packed'].includes(phase) && ['wasm-compat','simd.wasm-compat'].includes(flavor) && out,'explicit phase/flavor/output required');
assert.equal(path.basename(modulePath),`jolt-physics.${flavor}.js`);
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const nativeHead=readFileSync(new URL('../CMakeLists.txt',import.meta.url),'utf8').match(/GIT_TAG "([a-f0-9]{40})"/)?.[1];
assert.match(process.env.GITHUB_SHA||'',/^[a-f0-9]{40}$/,'actual checkout identity required');
assert.match(nativeHead||'',/^[a-f0-9]{40}$/,'actual CMake native pin required');
const sourceHead=execFileSync('git',['rev-parse','HEAD'],{cwd:new URL('../',import.meta.url),encoding:'utf8'}).trim();
assert.match(sourceHead,/^[a-f0-9]{40}$/);
const identity={sourceHead,eventHead:process.env.GITHUB_SHA,nativeHead,nodeVersion:process.version,phase,flavor,
    verifierSha256:sha(readFileSync(new URL(import.meta.url))),moduleSha256:sha(readFileSync(modulePath)),
    packageVersion:JSON.parse(readFileSync(path.join(path.dirname(path.dirname(path.resolve(modulePath))),'package.json'),'utf8')).version,
    packageSha256:phase==='packed'?sha(readFileSync(option('package'))):null};
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
    const ps = ji.GetPhysicsSystem(), bi = ps.GetBodyInterface(), bodies = [], hinges = [], events=[], steps=[];
    let segment=0;
    const physics=new J.PhysicsSettings();physics.mAllowSleeping=false;ps.SetPhysicsSettings(physics);J.destroy(physics);
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
        const item = {id:`b${bodies.length}`,b, inertia, mass, fixed,position,rotation}; bodies.push(item); return item;
    }
    function hinge(a, b, point = [0,0,0], axis = [0,1,0], jr) {
        const s = new J.HingeConstraintSettings();
        assert.equal(s.mReflectedRotorInertia, 0, 'settings default is disabled');
        s.mSpace = J.EConstraintSpace_WorldSpace;
        const p = new J.RVec3(...point), ax = new J.Vec3(...axis);
        const normal = new J.Vec3(...(Math.abs(axis[0]) < .9 ? cross(axis,[1,0,0]) : cross(axis,[0,1,0])));
        s.mPoint1 = p; s.mPoint2 = p; s.mHingeAxis1 = ax; s.mHingeAxis2 = ax;
        s.mNormalAxis1 = normal; s.mNormalAxis2 = normal;
        // Match the shipped Lite motor settings; higher iterations cannot qualify these fixtures.
        s.mNumVelocityStepsOverride = 30; s.mNumPositionStepsOverride = 5;
        if (jr !== undefined) s.mReflectedRotorInertia = jr;
        const constraint = s.Create(a.b, b.b);
        J.destroy(p); J.destroy(ax); J.destroy(normal); J.destroy(s);
        assert.ok(J.getPointer(constraint) !== 0, 'valid settings create a real constraint');
        const h = J.castObject(constraint, J.HingeConstraint); ps.AddConstraint(h);
        assert.equal(h.GetNumVelocityStepsOverride(),30,'shipped motor velocity iterations');
        assert.equal(h.GetNumPositionStepsOverride(),5,'shipped initial motor position iterations');
        for (const name of ['SetReflectedRotorInertia','GetReflectedRotorInertia','GetRotorAngularVelocity','GetTotalLambdaRotor']) assert.equal(typeof h[name], 'function', `native API ${name}`);
        hinges.push({id:`h${hinges.length}`,h,a,b,axis,point}); return h;
    }
    function configure(h, jr) {
        const before=state();
        assert.equal(h.SetReflectedRotorInertia(jr), true);
        // Native setters intentionally do not wake bodies; this is caller ownership.
        for (const item of bodies) if (!item.fixed) bi.ActivateBody(item.b.GetID());
        events.push({kind:'configure',requestedJr:jr,before,after:state(),step:steps.length});
    }
    function motor(h, sign = 1) {
        const m = h.GetMotorSettings(); m.mMinTorqueLimit = -TORQUE; m.mMaxTorqueLimit = TORQUE;
        h.SetMotorState(J.EMotorState_Velocity); h.SetTargetAngularVelocity(sign * 10000);
        events.push({kind:'command',sign,requestedTorqueLimit:TORQUE,requestedTargetVelocity:sign*10000,step:steps.length,after:state()});
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
    const state = () => ({bodies:bodies.map(item=>{
        const {b,fixed,id}=item,p=fixed?null:b.GetMotionProperties();
        return {id,position:xyz(b.GetCenterOfMassPosition()),rotation:quat(b.GetRotation()),velocity:xyz(b.GetLinearVelocity()),omega:xyz(b.GetAngularVelocity()),
            awake:b.IsActive(),sleepingAllowed:b.GetAllowSleeping(),gyroscopic:b.GetApplyGyroscopicForce(),inverseMass:p?p.GetInverseMass():0,inverseInertia:p?xyz(p.GetInverseInertiaDiagonal()):null,inertiaRotation:p?quat(p.GetInertiaRotation()):null};
    }),hinges:hinges.map(({id,h})=>({id,active:h.IsActive(),velocityStepsOverride:h.GetNumVelocityStepsOverride(),positionStepsOverride:h.GetNumPositionStepsOverride(),jr:h.GetReflectedRotorInertia(),wr:h.GetRotorAngularVelocity(),lambdaRotor:h.GetTotalLambdaRotor(),lambdaMotor:h.GetTotalLambdaMotor(),
        motorState:h.GetMotorState(),targetVelocity:h.GetTargetAngularVelocity(),minTorque:h.GetMotorSettings().mMinTorqueLimit,maxTorque:h.GetMotorSettings().mMaxTorqueLimit,
        localAxis1:xyz(h.GetLocalSpaceHingeAxis1()),localAxis2:xyz(h.GetLocalSpaceHingeAxis2()),localPoint1:xyz(h.GetLocalSpacePoint1()),localPoint2:xyz(h.GetLocalSpacePoint2())}))});
    const step = (count = 1, dt = DT) => {for (let i=0; i<count; i++) {
        assert.ok(steps.length<200,'bounded raw trace');
        const before=state();ji.Step(dt,1);steps.push({index:steps.length,segment,dt,before,after:state()});
    }};
    const restore = (recorder,label) => {const before=state();recorder.Rewind();assert.equal(ps.RestoreState(recorder),true);segment++;events.push({kind:'restore',label,step:steps.length,before,after:state()});};
    const checkpoint = (recorder,label) => {ps.SaveState(recorder,J.EStateRecorderState_All);events.push({kind:'checkpoint',label,step:steps.length,after:state()});};
    const event = data => events.push({...data,step:steps.length});
    const evidence = () => ({inputs:{physicsSettings:{velocitySteps:ps.GetPhysicsSettings().mNumVelocitySteps,positionSteps:ps.GetPhysicsSettings().mNumPositionSteps,allowSleeping:ps.GetPhysicsSettings().mAllowSleeping},
        bodies:bodies.map(({id,mass,inertia,fixed,position,rotation})=>({id,mass,inertia,fixed,position,rotation,gyroscopic:!fixed,sleepingAllowed:false})),hinges:hinges.map(({id,a,b,axis,point})=>({id,body1:a.id,body2:b.id,axis,point,velocityStepsOverride:30,positionStepsOverride:5}))},events,steps});
    return {ji,ps,bi,bodies,hinges,body,hinge,configure,motor,snapshot,tensorIdentity,momentum,step,restore,checkpoint,event,state,evidence,destroy:()=>J.destroy(ji)};
}
function scene(name, run) {
    const w = world();
    try {const result = run(w); results.push({name, ...result,...w.evidence()});}
    catch(error) {
        // Preserve the first actual failing state; it is diagnostic, never acceptance.
        writeFileSync(out,JSON.stringify({schema:'bw.relative-rotor.trace.v2',identity,complete:false,
            results:[...results,{name,error:String(error),...w.evidence()}]})+'\n');
        throw error;
    } finally {w.destroy();}
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
    let elapsed=0, target=0, work=0, before=0, minLambda=Infinity, maxLambda=-Infinity,peakEnergy=0;
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
            peakEnergy=Math.max(peakEnergy,kinetic);
            near(kinetic,work,Math.max(1e-8,Math.abs(work)*.002),'motor work includes internal scalar energy');
            minLambda=Math.min(minLambda,h.GetTotalLambdaRotor());maxLambda=Math.max(maxLambda,h.GetTotalLambdaRotor());
            if (!fixed) assert.ok(norm(w.momentum())<2e-7,'bare body angular momentum stays internal');
        }
    }
    const wa=xyz(a.b.GetAngularVelocity())[1],wb=xyz(b.b.GetAngularVelocity())[1],wr=h.GetRotorAngularVelocity();
    const energy=(a.fixed?0:i1*wa*wa/2)+(b.fixed?0:i2*wb*wb/2)+JR*wr*wr/2;
    const expected=.5*(mu+JR)*target**2;
    near(energy,expected,Math.max(1e-8,expected*.002),'independent kinetic energy');
    assert.deepEqual(w.tensorIdentity(),original,'rotor must not change mass or body inertia');
    assert.ok(minLambda<0&&maxLambda>0,'rotor impulse reverses sign with motor torque');
    assert.ok(peakEnergy>1e-6,'positive acceleration and energy actually occurred before reversal');
    return {elapsed,finalRelativeRate:wr,energy,expectedEnergy:expected,peakEnergy,workDuringReversal:work};
});

scene('default-zero-exact-trajectory', w => {
    const a=w.body(),b=w.body({inertia:[.0007,.0007,.0007]});
    const h=w.hinge(a,b); // Leave the native settings default untouched in the first trajectory.
    assert.equal(h.GetReflectedRotorInertia(),0);w.motor(h);
    const recorder = new J.StateRecorderImpl();w.checkpoint(recorder,'default-zero');
    const trace=[];for(let n=0;n<60;n++){w.step();trace.push(w.snapshot());}
    w.restore(recorder,'default-zero');w.configure(h,0);
    const again=[];for(let n=0;n<60;n++){w.step();again.push(w.snapshot());}
    assert.deepEqual(again,trace,'explicit zero has no active scalar row');
    assert.equal(h.GetRotorAngularVelocity(),0);assert.equal(h.GetTotalLambdaRotor(),0);J.destroy(recorder);
    return {frames:trace.length,sha256:createHash('sha256').update(JSON.stringify(trace)).digest('hex')};
});

scene('configuration-invalid-atomic-and-native-restore', w => {
    const {a,b,h}=principal(w);w.motor(h);w.step(20);
    const nativeState=()=>[h.GetReflectedRotorInertia(),h.GetRotorAngularVelocity(),h.GetTotalLambdaRotor()];
    const before=nativeState();
    for(const bad of [-1,NaN,Infinity,-Infinity]) {
        const prior=w.state(),returned=h.SetReflectedRotorInertia(bad);
        w.event({kind:'invalid-setter',valueTag:String(bad),returned,before:prior,after:w.state()});
        assert.equal(returned,false);assert.deepEqual(nativeState(),before);
    }
    w.configure(h,h.GetReflectedRotorInertia());assert.deepEqual(nativeState(),before,'same value preserves native scalar and lambda');
    const recorder=new J.StateRecorderImpl();w.checkpoint(recorder,'configuration');
    w.step(30);const expected=[w.snapshot(),nativeState()];
    const priorRate=h.GetRotorAngularVelocity();w.configure(h,JR*2);
    assert.equal(h.GetRotorAngularVelocity(),priorRate,'positive reconfiguration preserves scalar speed');
    assert.equal(h.GetTotalLambdaRotor(),0,'positive reconfiguration clears rotor warm-start impulse');
    w.step(10);w.configure(h,0);assert.equal(h.GetRotorAngularVelocity(),0);assert.equal(h.GetTotalLambdaRotor(),0);
    assert.notDeepEqual([w.snapshot(),nativeState()],expected,'changed configuration produces a genuinely different state');
    w.restore(recorder,'configuration');w.step(30);
    assert.deepEqual([w.snapshot(),nativeState()],expected,'recorder restores Jr, scalar and warm-start state');J.destroy(recorder);
    w.configure(h,0);const relative=xyz(b.b.GetAngularVelocity())[1]-xyz(a.b.GetAngularVelocity())[1];w.configure(h,JR);
    near(h.GetRotorAngularVelocity(),relative,2e-6,'enabling initializes current relative rate');
    return {invalidValues:4,restoredExactly:true};
});

scene('invalid-settings-create-refuses', w => {
    const a=w.body(),b=w.body();
    for(const bad of [-1,NaN,Infinity]) {
        const s=new J.HingeConstraintSettings();s.mReflectedRotorInertia=bad;
        const constraint=s.Create(a.b,b.b),pointer=J.getPointer(constraint);
        w.event({kind:'invalid-settings',valueTag:String(bad),pointer,after:w.state()});
        assert.equal(pointer,0,'invalid settings cannot create a native constraint');J.destroy(s);
    }
    return {invalidSettings:3};
});

for(const shared of [false,true]) for(const zeroBaseline of [true,false]) scene(`neighboring-transverse-hinge-and-anchor-${shared?'shared-rotors':'one-rotor'}-${zeroBaseline?'Jr0-baseline':'candidate'}`, w => {
    const a=w.body({position:[0,0,0],inertia:[.002,.001,.003],rotation:[0,0,Math.sin(.3),Math.cos(.3)]});
    const b=w.body({position:[0,.04,0],inertia:[.0007,.0012,.0009]});
    const c=w.body({position:[.04,.04,0],inertia:[.0015,.002,.001]});
    const first=w.hinge(a,b,[0,.02,0],[0,1,0],zeroBaseline?0:JR);
    const second=w.hinge(b,c,[.02,.04,0],[1,0,0],shared&&!zeroBaseline?JR/2:0);
    w.motor(first);if(shared)w.motor(second,-1);
    const initial=w.tensorIdentity();let maxPointSpeed=0,maxLockedRate=0,maxPointGap=0;
    for(let n=0;n<80;n++) {
        const pre=w.state(); // Native velocity Jacobians are cached before pose integration.
        w.step();assert.ok(norm(w.momentum())<3e-6,'bare multibody angular momentum');
        for(const {h,a,b,axis} of w.hinges) {
            const qa=pre.bodies.find(body=>body.id===a.id).rotation,qb=pre.bodies.find(body=>body.id===b.id).rotation;
            const ra=rotate(qa,xyz(h.GetLocalSpacePoint1())),rb=rotate(qb,xyz(h.GetLocalSpacePoint2()));
            // Geometric gaps are POST-state quantities; Cdot uses PRE arms with solved velocities.
            const pa=add(xyz(a.b.GetCenterOfMassPosition()),rotate(quat(a.b.GetRotation()),xyz(h.GetLocalSpacePoint1())));
            const pb=add(xyz(b.b.GetCenterOfMassPosition()),rotate(quat(b.b.GetRotation()),xyz(h.GetLocalSpacePoint2())));
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
    const relative=xyz(b.b.GetAngularVelocity())[1]-xyz(a.b.GetAngularVelocity())[1];
    assert.ok(Math.abs(zeroBaseline?relative:first.GetRotorAngularVelocity())>.001,'motor actually ran');
    if(shared&&!zeroBaseline) {
        const recorder=new J.StateRecorderImpl();w.checkpoint(recorder,'shared');
        const state=()=>[w.snapshot(),...[first,second].map(h=>[h.GetReflectedRotorInertia(),h.GetRotorAngularVelocity(),h.GetTotalLambdaRotor()])];
        w.step(20);const expected=state();
        w.configure(first,JR*2);w.configure(second,0);w.step(5);
        assert.notDeepEqual(state(),expected,'shared changed configuration is a genuine inequality control');
        w.restore(recorder,'shared');w.step(20);
        assert.deepEqual(state(),expected,'shared configuration and scalar rows restore together');J.destroy(recorder);
    }
    return {shared,zeroBaseline,maxPointSpeed,maxLockedRate,maxPointGap};
});

const report={schema:'bw.relative-rotor.trace.v2',identity,complete:true,results,boundary:'Bounded native state/impulse facts; independent source/package admission and analytic auditing required. No physical/all-model acceptance.'};
writeFileSync(out,JSON.stringify(report)+'\n');
console.log(JSON.stringify({schema:report.schema,identity,outputSha256:sha(readFileSync(out)),cases:results.map(({name,steps})=>({name,steps:steps.length}))}));
