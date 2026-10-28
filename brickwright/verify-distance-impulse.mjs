// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 CrispStrobe
// Native four-rope support: real impulses, complete 5ms momentum balance,
// and exact body-trajectory comparison with the previous package.
import assert from 'node:assert/strict';
import {readFileSync,writeFileSync} from 'node:fs';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
const [modulePath,...args]=process.argv.slice(2),legacy=args.includes('--legacy');
assert.ok(modulePath,'usage: module.js [--legacy] [--out=file] [--reference=file]');
const option=name=>args.find(a=>a.startsWith('--'+name+'='))?.slice(name.length+3);
const J=await(await import(pathToFileURL(modulePath).href)).default();
const read=v=>[v.GetX(),v.GetY(),v.GetZ()],sub=(a,b)=>a.map((v,i)=>v-b[i]),add=(a,b)=>a.map((v,i)=>v+b[i]),scale=(v,k)=>v.map(x=>x*k),norm=v=>Math.hypot(...v);
const settings=new J.JoltSettings(),pairs=new J.ObjectLayerPairFilterTable(1),broad=new J.BroadPhaseLayerInterfaceTable(1,1);
broad.MapObjectToBroadPhaseLayer(0,new J.BroadPhaseLayer(0));settings.mObjectLayerPairFilter=pairs;settings.mBroadPhaseLayerInterface=broad;
settings.mObjectVsBroadPhaseLayerFilter=new J.ObjectVsBroadPhaseLayerFilterTable(broad,1,pairs,1);
const world=new J.JoltInterface(settings);J.destroy(settings);
try{
 const ps=world.GetPhysicsSystem(),bi=ps.GetBodyInterface(),gravity=[0,-9.81,0],g=new J.Vec3(...gravity);ps.SetGravity(g);J.destroy(g);
 const makeBody=(position,dynamic)=>{
  const shape=new J.SphereShape(.001),pos=new J.RVec3(...position),q=new J.Quat(0,0,0,1);
  const s=new J.BodyCreationSettings(shape,pos,q,dynamic?J.EMotionType_Dynamic:J.EMotionType_Static,0);
  s.mAllowSleeping=false;
  if(dynamic){s.mOverrideMassProperties=J.EOverrideMassProperties_CalculateInertia;const props=s.mMassPropertiesOverride;props.mMass=.0109;s.mMassPropertiesOverride=props;}
  const b=bi.CreateBody(s);J.destroy(s);J.destroy(pos);J.destroy(q);bi.AddBody(b.GetID(),dynamic?J.EActivation_Activate:J.EActivation_DontActivate);return b;
 };
 const anchor=makeBody([0,0,0],false),payload=makeBody([0,-.1,0],true),ropes=[];
 for(const x of [-.064,.064])for(const z of [-.024,.024]){
  const ds=new J.DistanceConstraintSettings(),p1=new J.RVec3(x,0,z),p2=new J.RVec3(0,-.1,0);
  ds.mPoint1=p1;ds.mPoint2=p2;ds.mMinDistance=0;ds.mMaxDistance=Math.hypot(x,.1,z);
  const constraint=J.castObject(ds.Create(anchor,payload),J.DistanceConstraint);ps.AddConstraint(constraint);
  J.destroy(ds);J.destroy(p1);J.destroy(p2);
  if(!legacy)assert.equal(typeof constraint.GetLastVelocityImpulse,'function','required native impulse getter');
  ropes.push({constraint,anchor:[x,0,z]});
 }
 const defaults=new J.BodyCreationSettings(),damping=defaults.mLinearDamping;J.destroy(defaults);
 const mass=.0109,dt=.005,trace=[];let impulse=[0,0,0],oldImpulse=[0,0,0],drag=[0,0,0],initialVelocity;
 for(let i=0;i<400;i++){
  const before=read(payload.GetLinearVelocity()),position=read(payload.GetPosition());if(i===300)initialVelocity=before;
  world.Step(dt,1);
  const p=read(payload.GetPosition()),r=payload.GetRotation(),v=read(payload.GetLinearVelocity());trace.push([...p,r.GetX(),r.GetY(),r.GetZ(),r.GetW(),...v]);
  if(i>=300){
   for(const rope of ropes){const vector=sub(position,rope.anchor),normal=scale(vector,1/norm(vector)),old=rope.constraint.GetTotalLambdaPosition();
    impulse=add(impulse,scale(normal,legacy?old:rope.constraint.GetLastVelocityImpulse()));oldImpulse=add(oldImpulse,scale(normal,old));}
   drag=add(drag,scale(add(before,scale(gravity,dt)),-mass*damping*dt));
  }
 }
 const weightImpulse=mass*norm(gravity)*.5,momentum=scale(sub(read(payload.GetLinearVelocity()),initialVelocity),mass),gravityImpulse=scale(gravity,mass*.5);
 const residual=value=>norm(sub(add(add(value,gravityImpulse),drag),momentum))/weightImpulse;
 const result={legacy,frames:trace.length,relativeResidual:residual(impulse),supportFraction:impulse[1]/weightImpulse,legacyRelativeResidual:residual(oldImpulse),trajectorySha256:createHash('sha256').update(JSON.stringify(trace)).digest('hex'),trace};
 if(option('out'))writeFileSync(option('out'),JSON.stringify(result)+'\n');
 console.log(JSON.stringify({...result,trace:undefined}));
 if(legacy)assert.ok(result.relativeResidual>.05,'legacy control exposes missing impulse');
 else{assert.ok(result.relativeResidual<.05,'full native impulse/momentum balance');assert.ok(result.supportFraction>.9&&result.supportFraction<1.1,'native ropes support weight');}
 if(option('reference'))assert.deepEqual(result.trace,JSON.parse(readFileSync(option('reference'))).trace,'observation must preserve every actual body pose and velocity');
}finally{J.destroy(world);}
