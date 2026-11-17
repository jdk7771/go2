import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const viewport=document.querySelector('#viewport'),status=document.querySelector('#status');
const scene=new THREE.Scene();scene.background=new THREE.Color(0xeff1e8);
const camera=new THREE.PerspectiveCamera(42,1,.015,250);camera.up.set(0,0,1);
const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;viewport.appendChild(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);controls.minDistance=.05;controls.maxDistance=100;controls.maxPolarAngle=Math.PI/2;
const groups=[],dimensions=new THREE.Group();scene.add(dimensions);
const grid=new THREE.GridHelper(8,8,0xb4bca9,0xd8dece);grid.rotation.x=Math.PI/2;grid.position.set(0,0,-.025);scene.add(grid);
const state={ready:false,meta:null,view:'local'};
function draw(){renderer.render(scene,camera);}
function setView(name){
  const h=state.meta?.corridor.clear_height_m??.6;
  const views={local:{position:[5.8,-6.2,3.9],target:[0,0,.45],name:`通道局部 · 净高 ${h.toFixed(2)} m`},
    entrance:{position:[-7.5,0,.6],target:[0,0,.6],name:'入口正视 · 三个场景使用相同相机'},
    inside:{position:[-1.55,0,h/2],target:[2.7,0,h/2],name:`通道内部 · 相机高 ${(h/2).toFixed(2)} m`},
    overall:{position:[6,-8,5],target:[0,0,.45],name:'独立通道整体 · 地面 6 × 4 m'},
    top:{position:[0,-.001,9],target:[0,0,0],name:'俯视 · 独立通道与地面'}};
  const p=views[name];state.view=name;camera.position.set(...p.position);controls.target.set(...p.target);
  if(state.meta&&['overall','top'].includes(name)){
    const back=camera.position.clone().sub(controls.target).normalize(),right=new THREE.Vector3().crossVectors(camera.up,back).normalize(),up=new THREE.Vector3().crossVectors(back,right).normalize();
    const tangent=Math.tan(THREE.MathUtils.degToRad(camera.fov/2));let distance=1;
    for(const x of state.meta.bounds_m.map(v=>v[0]))for(const y of state.meta.bounds_m.map(v=>v[1]))for(const z of state.meta.bounds_m.map(v=>v[2])){
      const corner=new THREE.Vector3(x,y,z).sub(controls.target),depth=corner.dot(back);
      distance=Math.max(distance,Math.abs(corner.dot(right))/(tangent*camera.aspect)+depth,Math.abs(corner.dot(up))/tangent+depth);
    }
    camera.position.copy(controls.target).addScaledVector(back,distance*1.13);
  }
  controls.update();updateDimensions();
  document.querySelector('#view-label').textContent=p.name;
  document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===name));draw();
}
function updateDimensions(){dimensions.visible=document.querySelector('#dimensions').checked&&!['inside','overall','top'].includes(state.view);}
function addDimension(h){
  const x=-2.3,y=-1.45;
  const points=[[x,y,0],[x,y,h],[x,y,0],[x+.12,y,0],[x-.12,y,0],[x,y,0],[x,y,h],[x+.12,y,h],[x-.12,y,h]];
  dimensions.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(...p))),new THREE.LineBasicMaterial({color:0x325345})));
  const canvas=document.createElement('canvas');canvas.width=512;canvas.height=100;const ctx=canvas.getContext('2d');
  ctx.fillStyle='#ffffffe6';ctx.beginPath();ctx.roundRect(4,4,504,92,12);ctx.fill();ctx.font='500 42px "Noto Sans CJK SC",sans-serif';
  ctx.fillStyle='#315645';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(`净高 ${h.toFixed(2)} m`,256,50);
  const texture=new THREE.CanvasTexture(canvas);texture.colorSpace=THREE.SRGBColorSpace;
  const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:texture,transparent:true,depthTest:false}));
  sprite.position.set(x,-1.9,h/2+.1);sprite.scale.set(1.25,.245,1);sprite.renderOrder=10;dimensions.add(sprite);
}
controls.addEventListener('change',draw);
document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>setView(b.dataset.view)));
function visibility(){
  let n=0;
  groups.forEach((g,i)=>{g.visible=document.querySelector(`#layer-${i}`).checked;if(g.visible)n+=state.meta.layers[i].count;});
  document.querySelector('#visible-count').textContent=`当前显示 ${n.toLocaleString('zh-CN')} / ${state.meta.point_count.toLocaleString('zh-CN')} 点`;draw();
}
const stops=[new THREE.Color('#3b7c6e'),new THREE.Color('#8cae69'),new THREE.Color('#daba68'),new THREE.Color('#d97943')];
function colorize(){
  const height=document.querySelector('#color-mode').value==='height',max=state.meta.bounds_m[1][2],color=new THREE.Color();
  for(const group of groups){
    const geometry=group.children[0].geometry,position=geometry.attributes.position,rgb=geometry.attributes.color,source=new THREE.Color(state.meta.layers[group.userData.layer].color);
    for(let i=0;i<position.count;i++){
      if(height){const t=Math.max(0,Math.min(.999999,position.getZ(i)/max))*3,k=Math.floor(t);color.copy(stops[k]).lerp(stops[k+1],t-k);}
      else color.copy(source);
      rgb.setXYZ(i,color.r,color.g,color.b);
    }rgb.needsUpdate=true;
  }document.querySelector('#height-key').hidden=!height;draw();
}
document.querySelector('#color-mode').addEventListener('change',()=>state.ready&&colorize());
document.querySelector('#grid').addEventListener('change',e=>{grid.visible=e.target.checked;draw();});
document.querySelector('#dimensions').addEventListener('change',()=>{updateDimensions();draw();});
document.querySelector('#point-size').addEventListener('input',e=>{const size=Number(e.target.value);for(const g of groups)g.children[0].material.size=size;document.querySelector('#point-size-value').textContent=`${size} px`;draw();});
function resize(){const {width,height}=viewport.getBoundingClientRect();camera.aspect=width/height;camera.updateProjectionMatrix();renderer.setSize(width,height);if(state.ready&&['overall','top'].includes(state.view))setView(state.view);else draw();}
new ResizeObserver(resize).observe(viewport);resize();setView('local');
async function load(){
  const [response,buffer]=await Promise.all([fetch('./scene.json'),fetch('./points.bin')]);if(!response.ok||!buffer.ok)throw Error('读取场景失败');
  const meta=await response.json(),data=new Float32Array(await buffer.arrayBuffer());if(data.length!==meta.point_count*4)throw Error('点云长度不符');state.meta=meta;
  const positions=meta.layers.map(l=>new Float32Array(l.count*3)),offsets=meta.layers.map(()=>0);
  for(let i=0;i<data.length;i+=4){const k=data[i+3],o=offsets[k];positions[k][o]=data[i];positions[k][o+1]=data[i+1];positions[k][o+2]=data[i+2];offsets[k]+=3;}
  meta.layers.forEach((layer,i)=>{
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions[i],3));geometry.setAttribute('color',new THREE.BufferAttribute(new Float32Array(positions[i].length),3));geometry.computeBoundingBox();
    const object=new THREE.Points(geometry,new THREE.PointsMaterial({size:2,sizeAttenuation:false,vertexColors:true})),group=new THREE.Group();
    group.userData.layer=i;group.userData.heightRange=[geometry.boundingBox.min.z,geometry.boundingBox.max.z];group.add(object);groups.push(group);scene.add(group);
    const item=document.createElement('label'),checkbox=document.createElement('input'),dot=document.createElement('i'),text=document.createElement('span'),count=document.createElement('small');
    checkbox.type='checkbox';checkbox.checked=true;checkbox.id=`layer-${i}`;checkbox.addEventListener('change',visibility);dot.style.background=layer.color;text.textContent=layer.name;count.textContent=layer.count.toLocaleString('zh-CN');item.append(checkbox,dot,text,count);document.querySelector('#layers').append(item);
  });
  const c=meta.corridor;addDimension(c.clear_height_m);document.title=meta.title;document.querySelector('#title').textContent=meta.title+'，两端敞开。';
  document.querySelector('#clear-height').innerHTML=`${c.clear_height_m.toFixed(2)} <small>m</small>`;document.querySelector('#total-points').textContent=meta.point_count.toLocaleString('zh-CN');
  document.querySelector('#height-max').textContent=meta.bounds_m[1][2].toFixed(2);
  document.querySelectorAll('[data-clearance]').forEach(a=>{if(Number(a.dataset.clearance)===c.clear_height_m){a.classList.add('active');a.setAttribute('aria-current','page');}});
  document.querySelector('#geometry-summary').textContent=`通道长 4 m、净宽 2 m、净高 ${c.clear_height_m.toFixed(2)} m；顶板底面 Z = ${c.clear_height_m.toFixed(2)} m，上表面 Z = ${c.roof_top_z_m.toFixed(2)} m。顶板与墙体厚度均为 12 cm。`;
  document.querySelector('#source-summary').textContent=`完整加载 ${meta.point_count.toLocaleString('zh-CN')} 点，仅包含通道结构和 6 × 4 m 地面。三个场景的地面逐点相同。`;
  document.querySelector('#fit-link').href=`../?scene=${meta.name}`;
  state.ready=true;status.hidden=true;colorize();visibility();setView('local');window.corridorScene={state,scene,camera,controls,renderer,groups,grid,dimensions,setView,draw};
}
load().catch(e=>{status.hidden=false;status.textContent=`加载失败：${e.message}`;console.error(e);});
