import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
const $=id=>document.getElementById(id), manifest=window.ROLLING_MANIFEST;
const scene=new THREE.Scene();scene.background=new THREE.Color('#101827');
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));$('viewport').appendChild(renderer.domElement);
const camera=new THREE.PerspectiveCamera(43,1,.01,100);camera.up.set(0,0,1);
const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;
scene.add(new THREE.HemisphereLight(0xffffff,0x526277,2));
const grid=new THREE.GridHelper(5,50,0x33475f,0x1d2f45);grid.rotation.x=Math.PI/2;scene.add(grid);
let group=new THREE.Group();scene.add(group);let data,frame=0,playing=false,last=0,body,path,future,initial,range,frameRecord,loadedName;
const line=(color)=>new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color}));
function points(object,poses){const xyz=new Float32Array(poses.flatMap(p=>p.slice(0,3)));object.geometry.dispose();object.geometry=new THREE.BufferGeometry();object.geometry.setAttribute('position',new THREE.BufferAttribute(xyz,3));}
function marker(pose,color){const mesh=new THREE.Mesh(new THREE.SphereGeometry(.035,16,12),new THREE.MeshBasicMaterial({color}));mesh.position.set(...pose.slice(0,3));return mesh;}
function dispose(root){root.traverse(o=>{o.geometry?.dispose();if(Array.isArray(o.material))o.material.forEach(m=>m.dispose());else o.material?.dispose();});}
function draw(index){
  frame=Math.max(0,Math.min(data.executed.length-1,index));const pose=data.executed[frame];
  frameRecord=[...data.records].reverse().find(r=>r.start_index<=frame)??data.records[0];
  const isEnd=frame===data.executed.length-1;
  const attempt=frameRecord?.attempts?.at(-1);
  points(path,data.executed.slice(0,frame+1));
  const showFuture=attempt&&!(isEnd&&data.metrics.goal_reached);
  let pending=[];
  if(showFuture){
    // Remove the part of this local band already traversed in this cycle.
    const passed=data.executed.slice(frameRecord.start_index,frame+1);
    const walked=passed.slice(1).reduce((sum,p,i)=>sum+Math.hypot(...p.slice(0,3).map((v,j)=>v-passed[i][j])),0);
    let length=0;for(let i=1;i<attempt.final.length;i++){length+=Math.hypot(...attempt.final[i].slice(0,3).map((v,j)=>v-attempt.final[i-1][j]));if(length>walked+1e-6)pending.push(attempt.final[i]);}
    pending.unshift(pose);
  }
  points(future,pending);points(initial,showFuture?attempt.initial:[]);
  body.position.set(...pose.slice(0,3));body.rotation.set(pose[3],pose[4],pose[5],'ZYX');range.position.set(pose[0],pose[1],.015);
  $('timeline').value=frame;$('frame').textContent=`${frame} / ${data.executed.length-1}`;
  $('cycle').textContent=`${(frameRecord?.cycle??0)+1} / ${data.records.length}`;
  $('commands').textContent=frame;
  $('distance').textContent=`${Math.hypot(...pose.slice(0,3).map((v,i)=>v-data.goal[i])).toFixed(3)} m`;
  $('position').textContent=pose.slice(0,3).map(v=>v.toFixed(2)).join(', ');
  $('cycle-time').textContent=frameRecord?`${(frameRecord.seconds*1000).toFixed(0)} ms`:'—';
  $('recovery').textContent=attempt?.recovery?'已缩短至可达短段':'直连短期目标';
  $('status').textContent=isEnd?(data.metrics.goal_reached?(data.metrics.planning_success?'已到达终点 · 连续无碰撞':'已到达终点 · 连续验证未通过'):`规划停止：${({no_local_frontier:'未找到局部可达前沿',unsafe_prefix_rejected:'拒绝不安全的推进段',stalled:'推进停滞',replan_limit:'达到滚动次数上限'})[data.metrics.status]??data.metrics.status}`):`第 ${(frameRecord?.cycle??0)+1} 轮：仅规划前方一小段`;
  $('status').className=isEnd&&!data.metrics.planning_success?'bad':'good';
}
async function load(name){
  playing=false;$('play').textContent='▶ 播放';const entry=manifest.find(s=>s.name===name);
  await new Promise((resolve,reject)=>{const s=document.createElement('script');s.src=entry.file;s.onload=()=>{s.remove();resolve();};s.onerror=reject;document.head.appendChild(s);});
  data=window.ROLLING_DATA;loadedName=name;scene.remove(group);dispose(group);group=new THREE.Group();scene.add(group);
  $('scene').value=name;
  const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(data.cloud.flat(),3));group.add(new THREE.Points(geometry,new THREE.PointsMaterial({color:0x66859e,size:.009,transparent:true,opacity:.65})));
  path=line(0x76b8f5);future=line(0x65dbbc);initial=line(0xe9ad67);group.add(path,future,initial);
  body=new THREE.Group();const box=new THREE.BoxGeometry(...data.half.map(v=>v*2));const mesh=new THREE.Mesh(box,new THREE.MeshStandardMaterial({color:0xe4bc64,transparent:true,opacity:.9}));body.add(mesh,new THREE.LineSegments(new THREE.EdgesGeometry(box),new THREE.LineBasicMaterial({color:0xfff0ce})));body.add(new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(0,0,data.half[2]+.02),.35,0xffffff,.06,.035));group.add(body,marker(data.start,0x76b8f5),marker(data.goal,0xff8080));
  range=line(0x57718b);points(range,Array.from({length:97},(_,i)=>[.65*Math.cos(i/96*2*Math.PI),.65*Math.sin(i/96*2*Math.PI),0]));group.add(range);
  const center=new THREE.Vector3(...data.start.slice(0,3)).add(new THREE.Vector3(...data.goal.slice(0,3))).multiplyScalar(.5);controls.target.copy(center);camera.position.copy(center).add(new THREE.Vector3(2.6,-3.6,2.5));controls.update();
  $('title').textContent=data.label;$('description').textContent='从蓝色起点出发。绿色只表示本轮局部计划，蓝色路径随推进逐段增长。';
  $('source').hidden=!data.source;if(data.source)$('source').href=data.source;
  $('timeline').max=data.executed.length-1;
  $('reached').textContent=data.metrics.goal_reached?'是':'否';$('reached').className=data.metrics.goal_reached?'good':'bad';
  $('clearance').textContent=`${(data.metrics.continuous_lower_bound_against_surfaces_m*1000).toFixed(1)} mm`;$('clearance').className=data.metrics.continuous_lower_bound_against_surfaces_m>=0?'good':'bad';
  $('total-time').textContent=`${data.metrics.planning_seconds.toFixed(2)} s`;draw(0);
}
manifest.forEach(s=>{const option=document.createElement('option');option.value=s.name;option.textContent=s.label;$('scene').appendChild(option);});
$('scene').onchange=()=>load($('scene').value).catch(error=>{$('error').textContent=String(error);});
$('timeline').oninput=()=>{playing=false;$('play').textContent='▶ 播放';draw(Number($('timeline').value));};
$('play').onclick=()=>{if(frame>=data.executed.length-1)draw(0);playing=!playing;$('play').textContent=playing?'Ⅱ 暂停':'▶ 播放';};$('reset').onclick=()=>{playing=false;$('play').textContent='▶ 播放';draw(0);};
new ResizeObserver(()=>{const {width,height}=$('viewport').getBoundingClientRect();renderer.setSize(width,height);camera.aspect=width/height;camera.updateProjectionMatrix();}).observe($('viewport'));
function animate(time){requestAnimationFrame(animate);if(playing&&time-last>65){draw(frame+1);last=time;if(frame>=data.executed.length-1){playing=false;$('play').textContent='▶ 播放';}}controls.update();renderer.render(scene,camera);}requestAnimationFrame(animate);
window.rollingViewer={setScene:load,setFrame:draw,getState:()=>({ready:!!data,scene:loadedName,frame,pose:data?.executed[frame],committed:data?.executed.slice(0,frame+1),cycle:frameRecord?.cycle,goal:data?.goal,metrics:data?.metrics})};
load(manifest[0].name).catch(error=>{$('error').textContent=String(error);});
