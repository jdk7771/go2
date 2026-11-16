import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

const $=id=>document.getElementById(id);
const state={ready:false,data:null,manifest:null,view:'orbit',request:0,syncing:false,panels:[],webglError:null};
const camera=new THREE.PerspectiveCamera(43,1,.01,500);
camera.up.set(0,0,1);
const target=new THREE.Vector3();
const cache=new Map();
const definitions=[['cloud-view','cloud'],['gp-view','gp'],['quad-view','quad']];
function geometry(values){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(values,3));return g;}
function dispose(group){group.traverse(o=>{o.geometry?.dispose();if(o.material)(Array.isArray(o.material)?o.material:[o.material]).forEach(m=>m.dispose());});group.clear();}
function draw(){
  for(const p of state.panels){
    const w=p.viewport.clientWidth,h=p.viewport.clientHeight;
    camera.aspect=w/h;camera.updateProjectionMatrix();p.renderer.render(p.scene,camera);
  }
}
try{
  for(const [id,type] of definitions){
    const viewport=$(id),scene=new THREE.Scene();scene.background=new THREE.Color('#eaf0f6');
    scene.add(new THREE.HemisphereLight(0xffffff,0x8da6b9,1.5));
    const light=new THREE.DirectionalLight(0xffffff,2.0);light.position.set(-3,-4,8);scene.add(light);
    const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});
    renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.setPixelRatio(Math.min(devicePixelRatio,2));
    viewport.appendChild(renderer.domElement);
    const controls=new OrbitControls(camera,renderer.domElement);controls.target=target;
    controls.enableDamping=false;controls.minDistance=.1;controls.maxDistance=200;
    controls.addEventListener('change',()=>{if(!state.syncing)draw();});
    const content=new THREE.Group(),slice=new THREE.Group();scene.add(content,slice);
    const panel={viewport,scene,renderer,controls,content,slice,type};state.panels.push(panel);
    new ResizeObserver(()=>{renderer.setSize(viewport.clientWidth,viewport.clientHeight);if(state.ready)setView(state.view);else draw();}).observe(viewport);
  }
}catch(error){
  state.webglError=error.message;
  for(const [id] of definitions)if(!$(id).querySelector('canvas'))$(id).textContent='三维未启用，可查看下方剖面和静态对比图。';
}

function points(data,featureColors=false){
  const g=geometry(data.points),colors=[];
  if(featureColors){
    for(const flag of data.feature_mask)new THREE.Color(flag?0xd39235:0x9baab8).toArray(colors,colors.length);
    g.setAttribute('color',new THREE.Float32BufferAttribute(colors,3));
  }
  const object=new THREE.Points(g,new THREE.PointsMaterial({color:featureColors?0xffffff:0x344e63,vertexColors:featureColors,size:featureColors?2.6:2,sizeAttenuation:false}));
  object.userData.input=true;object.renderOrder=3;return object;
}
function selected(data){return new THREE.Points(geometry(data.inducing),new THREE.PointsMaterial({color:0xc23c3b,size:5.2,sizeAttenuation:false,depthTest:false}));}
function surface(field,color){
  const d=state.data,nx=d.x.length,ny=d.y.length,positions=[],indices=[];
  for(let j=0;j<ny;j++)for(let i=0;i<nx;i++)positions.push(d.x[i],d.y[j],Number.isFinite(field[j*nx+i])?field[j*nx+i]:0);
  for(let j=0;j<ny-1;j++)for(let i=0;i<nx-1;i++){
    const a=j*nx+i,b=a+1,c=a+nx,e=c+1;
    for(const tri of [[a,b,c],[b,e,c]])if(tri.every(k=>Number.isFinite(field[k])))indices.push(...tri);
  }
  const g=geometry(positions);g.setIndex(indices);g.computeVertexNormals();g.computeBoundingBox();
  const mesh=new THREE.Mesh(g,new THREE.MeshStandardMaterial({color,side:THREE.DoubleSide,roughness:.82,metalness:0,polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1}));
  mesh.userData.surface=true;
  const wire=new THREE.LineSegments(new THREE.WireframeGeometry(g),new THREE.LineBasicMaterial({color:0x476f8b,transparent:true,opacity:.20}));
  wire.userData.wire=true;wire.visible=$('wire').checked;
  return [mesh,wire];
}
function controlField(){return $('control').value==='global'?'quadratic_height':'local_height';}
function install(){
  const d=state.data;
  for(const panel of state.panels){
    dispose(panel.content);dispose(panel.slice);
    const cloud=points(d,panel.type==='cloud');
    cloud.visible=panel.type==='cloud'||$('overlay').checked;panel.content.add(cloud);
    if(panel.type==='cloud')panel.content.add(selected(d));
    if(panel.type==='gp')surface(d.gp_height,0x328bd0).forEach(m=>panel.content.add(m));
    if(panel.type==='quad')surface(d[controlField()],$('control').value==='global'?0x9878bb:0xdd983f).forEach(m=>panel.content.add(m));
    const center=d.bounds[0].map((v,i)=>(v+d.bounds[1][i])/2);
    const size=Math.ceil(Math.max(d.x.at(-1)-d.x[0],d.y.at(-1)-d.y[0])+1);
    const grid=new THREE.GridHelper(size,size*2,0xb5c8d9,0xd0dce7);grid.rotation.x=Math.PI/2;
    grid.position.set(center[0],center[1],d.bounds[0][2]-.05);grid.material.transparent=true;grid.material.opacity=.6;
    panel.content.add(grid);
  }
  $('control-title').textContent=$('control').value==='global'?'③ 全窗单个二次函数对照':'③ 局部二次函数对照';
  $('control-caption').textContent=$('control').value==='global'?'紫色实面 · 整个窗口只拟合一个二次函数':'橙色实面 · 使用相同保留点的 20 近邻';
  buildInfo();plotProfile();setView(state.view);
}
function setView(name){
  state.view=name;if(!state.data||!state.panels.length)return;
  const d=state.data,box=new THREE.Box3(new THREE.Vector3(...d.bounds[0]),new THREE.Vector3(...d.bounds[1]));
  for(const key of ['gp_height',controlField()])for(let i=0;i<d[key].length;i++)if(Number.isFinite(d[key][i])){
    box.min.z=Math.min(box.min.z,d[key][i]);box.max.z=Math.max(box.max.z,d[key][i]);
  }
  const size=box.getSize(new THREE.Vector3()),center=box.getCenter(new THREE.Vector3());
  const aspect=Math.min(...state.panels.map(p=>p.viewport.clientWidth/p.viewport.clientHeight));
  const half=THREE.MathUtils.degToRad(camera.fov/2),fov=Math.min(half,Math.atan(Math.tan(half)*aspect));
  const distance=Math.max(size.length()/2,.2)/Math.sin(fov)*1.04;
  const direction=new THREE.Vector3(...(name==='top'?[0,-.0001,1]:name==='side'?[0,-1,.12]:[.9,-1.2,.8])).normalize();
  target.copy(center);camera.position.copy(center).addScaledVector(direction,distance);
  state.syncing=true;state.panels.forEach(p=>p.controls.update());state.syncing=false;
  document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===name));draw();
}
function textNode(tag,content,className){const n=document.createElement(tag);n.textContent=content;if(className)n.className=className;return n;}
function fmt(value,digits=2){return Number.isFinite(value)?value.toFixed(digits):'—';}
function buildInfo(){
  const d=state.data,p=d.partition;
  const steps=[[p.input,'输入点'],[p.feature,'特征点'],[p.voxel_representatives,'非特征体素代表'],[p.candidates,'合并候选'],[p.retained,'保留点 M']];
  $('pipeline').replaceChildren();
  steps.forEach(([value,label],i)=>{
    if(i)$('pipeline').append(textNode('span','→','arrow'));
    const node=document.createElement('div');node.className='step';node.append(textNode('b',value.toLocaleString()),label);$('pipeline').append(node);
  });
  const analytic=!!d.analytic_errors;
  $('truth-legend').hidden=!analytic;
  $('error-title').textContent=analytic?'组合地形：与解析真值的高程误差':'输入点重建高程误差';
  $('error-note').textContent=analytic?'三种方法使用同一组有效查询格；真值 z=0.3·clip(x+1,0,2)。':'含训练点及参与特征计算的输入点，带有观测噪声；不是独立测试集或论文官方真值。';
  const errors=analytic?d.analytic_errors.all:d.reconstruction;
  const table=document.createElement('table');table.className='metrics-table';
  const head=document.createElement('tr');['方法','MAE (cm)','RMSE (cm)','格/点数'].forEach(s=>head.append(textNode('th',s)));table.append(head);
  for(const [key,label] of [['gp','FSGP'],['local','局部二次对照'],['global','全窗二次对照']]){
    const row=document.createElement('tr'),e=errors[key];
    [label,fmt(e?.mae_m*100),fmt(e?.rmse_m*100),e?.n?.toLocaleString()??'—'].forEach(s=>row.append(textNode('td',s)));table.append(row);
  }
  $('metrics').replaceChildren(table);
  $('seam-metrics').textContent=analytic?`两个接缝 ±0.2 m：FSGP MAE ${fmt(d.analytic_errors.seams.gp?.mae_m*100)} cm，局部二次 ${fmt(d.analytic_errors.seams.local?.mae_m*100)} cm，全窗二次 ${fmt(d.analytic_errors.seams.global?.mae_m*100)} cm。`:'';
  $('equation').textContent=`(x₀,y₀)=(${d.quadratic.origin.map(v=>fmt(v,4)).join(', ')}) m\n(a,b,c,d,e,f)=(${d.quadratic.coefficients.map(v=>v.toPrecision(5)).join(', ')})`;
  $('settings').textContent='3D KNN=20；κ 阈值 0.03；平均绝对高差阈值 0.03 m；非特征体素边长 0.10 m；IDW=5 近邻。二次函数对照不使用区域生长或语义分割。';
  $('preview').src=`./${d.scene}-surfaces.png`;
  $('preview').alt=`${d.label}，默认第8帧M=125：论文点云处理、FSGP曲面和局部二次函数对照`;
  $('surface-image').href=$('preview').src;$('profile-image').href=`./${d.scene}-profile.png`;
  $('scene-note').textContent=d.scene==='table'?'桌子保留地面、桌面、桌底和四条腿，没有提前分离高度层。同一 XY 的多层几何不能由一张高度函数完整表示。':d.scene==='flat_ramp'?'组合场景：x≤−1 m 是 z=0 的平地，−1<x<1 m 是坡度 0.3（16.70°）的斜坡，x≥1 m 是 z=0.6 m 的高处平地。两个接缝高度连续。':'原页面同一场景、第8帧的完整观测点，不使用旧图中900点显示抽样作为计算输入。';
}

const NS='http://www.w3.org/2000/svg';
function svgElement(tag,attributes={},content){const e=document.createElementNS(NS,tag);for(const [k,v]of Object.entries(attributes))e.setAttribute(k,v);if(content!==undefined)e.textContent=content;return e;}
function segments(values){const result=[];let current=[];values.forEach((v,i)=>{if(Number.isFinite(v))current.push(i);else{if(current.length)result.push(current);current=[];}});if(current.length)result.push(current);return result;}
function slice3D(row){
  const d=state.data,n=d.x.length,y=d.y[row];
  for(const p of state.panels){
    dispose(p.slice);
    let positions=[];
    if(p.type==='cloud'){
      const z=d.bounds[0][2]-.025;positions=[d.x[0],y,z,d.x.at(-1),y,z];
    }else{
      const field=p.type==='gp'?d.gp_height:d[controlField()];
      for(let i=0;i<n-1;i++){const a=row*n+i,b=a+1;if(Number.isFinite(field[a])&&Number.isFinite(field[b]))positions.push(d.x[i],y,field[a],d.x[i+1],y,field[b]);}
    }
    p.slice.add(new THREE.LineSegments(geometry(positions),new THREE.LineBasicMaterial({color:0x26965f,depthTest:false,transparent:true,opacity:.95})));
  }
  draw();
}
function plotProfile(){
  if(!state.data)return;
  const d=state.data,n=d.x.length,row=Math.min(d.y.length-1,Number($('slice').value)),y0=d.y[row];
  $('slice-value').textContent=`${fmt(y0,3)} m`;
  const keys=['gp_height','local_height','quadratic_height'];
  const arrays=Object.fromEntries(keys.map(k=>[k,d[k].slice(row*n,(row+1)*n)]));
  const truth=d.truth?.slice(row*n,(row+1)*n);
  const sigma=d.gp_sigma.slice(row*n,(row+1)*n);
  const observed=[];
  for(let i=0;i<d.points.length;i+=3)if(Math.abs(d.points[i+1]-y0)<=.1&&d.points[i]>=d.x[0]&&d.points[i]<=d.x.at(-1))observed.push([d.points[i],d.points[i+2]]);
  const shown=['gp_height','local_height'];if($('show-global').checked)shown.push('quadratic_height');
  const ranges=shown.flatMap(k=>arrays[k].filter(Number.isFinite)).concat(observed.map(p=>p[1]),truth??[]);
  if($('uncertainty').checked)arrays.gp_height.forEach((v,i)=>{if(Number.isFinite(v))ranges.push(v-1.96*sigma[i],v+1.96*sigma[i]);});
  let lo=Math.min(...ranges),hi=Math.max(...ranges);
  if(!Number.isFinite(lo)||!Number.isFinite(hi)){lo=0;hi=1;}
  const pad=Math.max((hi-lo)*.12,.025);lo-=pad;hi+=pad;
  const width=Math.max($('profile').clientWidth,280),height=width<600?275:320;
  const left=54,right=18,top=18,bottom=42,w=width-left-right,h=height-top-bottom;
  const sx=x=>left+(x-d.x[0])/(d.x.at(-1)-d.x[0])*w,sy=z=>top+(hi-z)/(hi-lo)*h;
  const svg=svgElement('svg',{viewBox:`0 0 ${width} ${height}`,role:'img','aria-label':`y=${fmt(y0,3)}米的剖面拟合曲线`});
  for(let i=0;i<=4;i++){
    const x=d.x[0]+(d.x.at(-1)-d.x[0])*i/4,z=lo+(hi-lo)*i/4;
    svg.append(svgElement('line',{x1:sx(x),x2:sx(x),y1:top,y2:top+h,stroke:'#e5ecf2'}));
    svg.append(svgElement('line',{x1:left,x2:left+w,y1:sy(z),y2:sy(z),stroke:'#e5ecf2'}));
    svg.append(svgElement('text',{x:sx(x),y:height-18,'text-anchor':'middle',fill:'#8294a3','font-size':11},fmt(x,1)));
    svg.append(svgElement('text',{x:left-7,y:sy(z)+4,'text-anchor':'end',fill:'#8294a3','font-size':11},fmt(z,2)));
  }
  svg.append(svgElement('text',{x:left+4,y:12,fill:'#71879b','font-size':11},'Z (m)'));
  svg.append(svgElement('text',{x:width-18,y:height-2,'text-anchor':'end',fill:'#71879b','font-size':11},'X (m)'));
  if(d.analytic_errors)for(const edge of [-1,1])if(edge>=d.x[0]&&edge<=d.x.at(-1))svg.append(svgElement('line',{x1:sx(edge),x2:sx(edge),y1:top,y2:top+h,stroke:'#a6b8c4','stroke-dasharray':'3 4'}));
  if($('uncertainty').checked)for(const ids of segments(arrays.gp_height)){
    const upper=ids.map(i=>`${sx(d.x[i])},${sy(arrays.gp_height[i]+1.96*sigma[i])}`);
    const lower=[...ids].reverse().map(i=>`${sx(d.x[i])},${sy(arrays.gp_height[i]-1.96*sigma[i])}`);
    svg.append(svgElement('polygon',{points:upper.concat(lower).join(' '),fill:'#267dbb',opacity:.11}));
  }
  for(const p of observed)svg.append(svgElement('circle',{cx:sx(p[0]),cy:sy(p[1]),r:2.4,fill:'#7f8f9e',opacity:.75}));
  for(const [key,color,dash]of [['quadratic_height','#9477b7','6 5'],['local_height','#d58c30',''],['gp_height','#287eb9','']]){
    if(!shown.includes(key))continue;
    for(const ids of segments(arrays[key]))svg.append(svgElement('polyline',{points:ids.map(i=>`${sx(d.x[i])},${sy(arrays[key][i])}`).join(' '),fill:'none',stroke:color,'stroke-width':2,'stroke-dasharray':dash}));
  }
  if(truth)svg.append(svgElement('polyline',{points:truth.map((z,i)=>`${sx(d.x[i])},${sy(z)}`).join(' '),fill:'none',stroke:'#23394e','stroke-width':1.6,'stroke-dasharray':'2 4'}));
  const hair=svgElement('line',{x1:left,x2:left,y1:top,y2:top+h,stroke:'#8ba7bc',visibility:'hidden'});svg.append(hair);
  const value=textNode('p','移动到曲线上可读取同一 X 位置的拟合高程。');value.style.fontSize='10px';
  svg.addEventListener('pointermove',event=>{
    const point=svg.createSVGPoint();point.x=event.clientX;point.y=event.clientY;const local=point.matrixTransform(svg.getScreenCTM().inverse());
    const index=Math.round(Math.min(1,Math.max(0,(local.x-left)/w))*(n-1));
    hair.setAttribute('x1',sx(d.x[index]));hair.setAttribute('x2',sx(d.x[index]));hair.setAttribute('visibility','visible');
    value.textContent=`X=${fmt(d.x[index],3)} m；FSGP=${fmt(arrays.gp_height[index],3)} m；局部二次=${fmt(arrays.local_height[index],3)} m；全窗二次=${fmt(arrays.quadratic_height[index],3)} m${truth?`；真值=${fmt(truth[index],3)} m`:''}`;
  });
  $('profile').replaceChildren(svg,value);slice3D(row);
}

async function json(path){const r=await fetch(path);if(!r.ok)throw new Error(`${r.status}: ${path}`);return r.json();}
function frames(){
  const entry=state.manifest.scenes.find(s=>s.id===$('scene').value);
  const old=$('frame').value===''?state.manifest.default_frame:Number($('frame').value);
  $('frame').replaceChildren(...entry.frames.map(f=>new Option(`第 ${f+1} 帧`,String(f))));
  $('frame').value=String(entry.frames.includes(old)?old:entry.frames.at(-1));
}
async function loadCase(){
  const request=++state.request;state.ready=false;$('status').textContent='正在读取同一帧的拟合结果…';$('status').classList.remove('error');
  try{
    const key=`${$('scene').value}_m${$('count').value}_${$('frame').value.padStart(3,'0')}`;
    if(!cache.has(key))cache.set(key,await json(`./cases/${key}.json`));
    if(request!==state.request)return;
    state.data=cache.get(key);$('slice').max=state.data.y.length-1;
    $('slice').value=state.data.y.reduce((best,v,i)=>Math.abs(v-.1)<Math.abs(state.data.y[best]-.1)?i:best,0);
    install();state.ready=true;
    $('status').textContent=`${state.data.label} · 第 ${state.data.frame+1} 帧 · ${state.data.partition.input} 输入点 / ${state.data.count} 保留点`;
  }catch(error){if(request!==state.request)return;$('status').textContent=error.message;$('status').classList.add('error');console.error(error);}
}
$('scene').addEventListener('change',()=>{frames();loadCase();});
$('count').addEventListener('change',loadCase);$('frame').addEventListener('change',loadCase);
$('control').addEventListener('change',()=>{if(state.ready)install();});
for(const id of ['slice','uncertainty','show-global'])$(id).addEventListener('input',plotProfile);
$('overlay').addEventListener('change',()=>{for(const p of state.panels)p.content.children.filter(o=>o.userData.input).forEach(o=>o.visible=p.type==='cloud'||$('overlay').checked);draw();});
$('wire').addEventListener('change',()=>{for(const p of state.panels)p.content.children.filter(o=>o.userData.wire).forEach(o=>o.visible=$('wire').checked);draw();});
document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>setView(b.dataset.view)));
let lastProfileWidth=0;
new ResizeObserver(()=>{const width=$('profile').clientWidth;if(width!==lastProfileWidth){lastProfileWidth=width;if(state.ready)plotProfile();}}).observe($('profile'));
window.fsgpFitViewer={state,camera,target,loadCase,setView,plotProfile,draw};
try{
  state.manifest=await json('./manifest.json');
  for(const s of state.manifest.scenes)$('scene').add(new Option(s.name,s.id));
  for(const count of state.manifest.counts)$('count').add(new Option(String(count),String(count)));
  $('scene').value=state.manifest.default_scene;$('count').value=String(state.manifest.default_count);frames();
  await loadCase();
}catch(error){$('status').textContent=error.message;$('status').classList.add('error');console.error(error);}
