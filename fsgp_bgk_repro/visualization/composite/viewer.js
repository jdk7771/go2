import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

const viewport=document.querySelector('#viewport'),status=document.querySelector('#status');
const scene=new THREE.Scene();scene.background=new THREE.Color(0xeff1e8);
const camera=new THREE.PerspectiveCamera(42,1,.02,250);camera.up.set(0,0,1);
const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;
viewport.appendChild(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);
controls.minDistance=.35;controls.maxDistance=100;controls.maxPolarAngle=Math.PI/2-.002;
const groups=[],markers=new THREE.Group();scene.add(markers);
const grid=new THREE.GridHelper(36,36,0xb4bca9,0xd8dece);grid.rotation.x=Math.PI/2;
grid.position.set(1,0,-.025);scene.add(grid);
const state={ready:false,meta:null,view:'overall'};
const presets={
  overall:{position:[20,-29,26],target:[1,0,.65],name:'整体场景 · 32 × 12 m'},
  hill:{position:[-3,-10,7],target:[-10,0,.6],name:'丘陵 · 原始地形仅平移'},
  forest:{position:[19,-10,8],target:[11,0,1],name:'森林 · 保留完整点云'},
  table:{position:[5,-6,4],target:[0,0,.55],name:'桌子局部 · 桌面 3 × 2 m'},
  under:{position:[3.5,-4.6,.5],target:[0,0,.5],name:'桌底视角 · 净空 1.10 m'},
  top:{position:[1,-.001,36],target:[1,0,0],name:'俯视 · 丘陵 → 空地 / 桌子 → 森林'},
};
function draw(){renderer.render(scene,camera);}
function setView(name){
  const p=presets[name];state.view=name;camera.position.set(...p.position);controls.target.set(...p.target);
  if(state.meta&&['overall','top'].includes(name)){
    const back=camera.position.clone().sub(controls.target).normalize(),right=new THREE.Vector3().crossVectors(camera.up,back).normalize(),up=new THREE.Vector3().crossVectors(back,right).normalize();
    const tangent=Math.tan(THREE.MathUtils.degToRad(camera.fov/2));let distance=1;
    for(const x of state.meta.bounds_m.map(v=>v[0]))for(const y of state.meta.bounds_m.map(v=>v[1]))for(const z of state.meta.bounds_m.map(v=>v[2])){
      const corner=new THREE.Vector3(x,y,z).sub(controls.target),depth=corner.dot(back);
      distance=Math.max(distance,Math.abs(corner.dot(right))/(tangent*camera.aspect)+depth,Math.abs(corner.dot(up))/tangent+depth);
    }
    camera.position.copy(controls.target).addScaledVector(back,distance*1.13);
  }
  controls.update();updateMarkers();
  document.querySelector('#view-label').textContent=p.name;
  document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===name));draw();
}
controls.addEventListener('change',draw);
document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>setView(b.dataset.view)));
function label(text,position,width=3){
  const canvas=document.createElement('canvas');canvas.width=640;canvas.height=100;
  const ctx=canvas.getContext('2d');ctx.fillStyle='#ffffffe6';ctx.beginPath();ctx.roundRect(4,4,632,92,14);ctx.fill();
  ctx.font='500 42px "Noto Sans CJK SC",sans-serif';ctx.fillStyle='#3b5d40';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(text,320,50);
  const texture=new THREE.CanvasTexture(canvas);texture.colorSpace=THREE.SRGBColorSpace;
  const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:texture,depthTest:false,transparent:true}));
  sprite.position.set(...position);sprite.scale.set(width,width/6.4,1);sprite.renderOrder=10;markers.add(sprite);
}
function visibility(){
  let count=0;const isolated=document.querySelector('#isolate-table').checked;
  groups.forEach((group,i)=>{
    group.visible=document.querySelector(`#layer-${i}`).checked&&(!isolated||[2,4,5].includes(i));
    if(group.visible)count+=state.meta.layers[i].count;
  });
  document.querySelector('#visible-count').textContent=`当前显示 ${count.toLocaleString('zh-CN')} / ${state.meta.point_count.toLocaleString('zh-CN')} 点`;
  draw();
}
const stops=[new THREE.Color('#3b7c6e'),new THREE.Color('#8cae69'),new THREE.Color('#daba68'),new THREE.Color('#d97943')];
function colorize(){
  const height=document.querySelector('#color-mode').value==='height',max=state.meta.bounds_m[1][2],color=new THREE.Color();
  for(const group of groups){
    const object=group.children[0],position=object.geometry.attributes.position,colorAttribute=object.geometry.attributes.color;
    const source=new THREE.Color(state.meta.layers[group.userData.layer].color);
    for(let i=0;i<position.count;i++){
      if(height){const t=Math.max(0,Math.min(.999999,position.getZ(i)/max))*3,k=Math.floor(t);color.copy(stops[k]).lerp(stops[k+1],t-k);}
      else{
        color.copy(source);
        // Source hue identifies the input cloud. Height-dependent brightness
        // gives dense, unlit points enough contrast to reveal hills and trees.
        if([0,1,6].includes(group.userData.layer)){
          const [lo,hi]=group.userData.heightRange,t=(position.getZ(i)-lo)/Math.max(1e-6,hi-lo);
          color.multiplyScalar(.62+.55*t);
        }
      }
      colorAttribute.setXYZ(i,color.r,color.g,color.b);
    }
    colorAttribute.needsUpdate=true;
  }
  document.querySelector('#height-key').hidden=!height;draw();
}
document.querySelector('#color-mode').addEventListener('change',()=>state.ready&&colorize());
document.querySelector('#isolate-table').addEventListener('change',()=>state.ready&&visibility());
function updateMarkers(){markers.visible=document.querySelector('#labels').checked&&!['table','under'].includes(state.view);}
document.querySelector('#labels').addEventListener('change',()=>{updateMarkers();draw();});
document.querySelector('#grid').addEventListener('change',e=>{grid.visible=e.target.checked;draw();});
document.querySelector('#point-size').addEventListener('input',e=>{
  const size=Number(e.target.value);for(const group of groups)group.children[0].material.size=size;
  document.querySelector('#point-size-value').textContent=`${size} px`;draw();
});
function resize(){const {width,height}=viewport.getBoundingClientRect();camera.aspect=width/height;camera.updateProjectionMatrix();renderer.setSize(width,height);if(state.ready&&['overall','top'].includes(state.view))setView(state.view);else draw();}
new ResizeObserver(resize).observe(viewport);resize();setView('overall');
async function load(){
  const [metaResponse,bufferResponse]=await Promise.all([fetch('./scene.json'),fetch('./points.bin')]);
  if(!metaResponse.ok||!bufferResponse.ok)throw Error('无法读取场景文件');
  const meta=await metaResponse.json(),data=new Float32Array(await bufferResponse.arrayBuffer());
  if(data.length!==meta.point_count*4)throw Error('点云文件长度与元数据不一致');
  state.meta=meta;
  const positions=meta.layers.map(l=>new Float32Array(l.count*3)),offsets=meta.layers.map(()=>0);
  for(let i=0;i<data.length;i+=4){const k=data[i+3],o=offsets[k];positions[k][o]=data[i];positions[k][o+1]=data[i+1];positions[k][o+2]=data[i+2];offsets[k]+=3;}
  meta.layers.forEach((layer,i)=>{
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions[i],3));
    geometry.setAttribute('color',new THREE.BufferAttribute(new Float32Array(positions[i].length),3));
    const object=new THREE.Points(geometry,new THREE.PointsMaterial({size:2,sizeAttenuation:false,vertexColors:true}));
    geometry.computeBoundingBox();
    const group=new THREE.Group();group.userData.layer=i;group.userData.heightRange=[geometry.boundingBox.min.z,geometry.boundingBox.max.z];group.add(object);groups.push(group);scene.add(group);
    const item=document.createElement('label'),checkbox=document.createElement('input'),dot=document.createElement('i'),text=document.createElement('span'),count=document.createElement('small');
    checkbox.type='checkbox';checkbox.checked=true;checkbox.id=`layer-${i}`;checkbox.addEventListener('change',visibility);
    dot.style.background=layer.color;text.textContent=layer.name;count.textContent=layer.count.toLocaleString('zh-CN');
    item.append(checkbox,dot,text,count);document.querySelector('#layers').append(item);
  });
  label('丘陵',[-10,2,2.1],2.3);label('中央空地 · 大桌子',[0,3.7,1.7],4);label('森林',[11,2.5,3.4],2.3);
  document.querySelector('#total-points').textContent=meta.point_count.toLocaleString('zh-CN');
  document.querySelector('#height-max').textContent=meta.bounds_m[1][2].toFixed(2);
  document.querySelector('#source-summary').textContent=`丘陵 ${meta.sources[0].retained_unique_points.toLocaleString('zh-CN')} 点，森林 ${meta.sources[1].retained_unique_points.toLocaleString('zh-CN')} 点；原桌子点云完整保留，另补空地与连接区域。网页加载全部 ${meta.point_count.toLocaleString('zh-CN')} 个点。`;
  state.ready=true;status.hidden=true;colorize();visibility();setView(state.view);
  window.combinedScene={state,scene,camera,renderer,controls,groups,markers,grid,setView,draw};
}
load().catch(e=>{status.hidden=false;status.textContent=`加载失败：${e.message}`;console.error(e);});
