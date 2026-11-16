import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

const $=id=>document.getElementById(id),finite=Number.isFinite;
const state={ready:false,data:null,view:'orbit',panels:[],request:0,syncing:false,webglError:null};
const camera=new THREE.PerspectiveCamera(42,1,.01,100),target=new THREE.Vector3();
camera.up.set(0,0,1);
const cache=new Map(),colors=['#21856b','#417cc4','#e49b35'];
const fmt=(v,n=3)=>finite(v)?v.toFixed(n):'空缺';
const el=(tag,text)=>{const n=document.createElement(tag);if(text!==undefined)n.textContent=text;return n;};
function geometry(values){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(values,3));return g;}
function dispose(group){group.traverse(o=>{o.geometry?.dispose();if(o.material)(Array.isArray(o.material)?o.material:[o.material]).forEach(m=>m.dispose());});group.clear();}
function draw(){for(const p of state.panels){camera.aspect=p.viewport.clientWidth/p.viewport.clientHeight;camera.updateProjectionMatrix();p.renderer.render(p.scene,camera);}}
try{
  for(const [id,type]of [['cloud-view','cloud'],['old-view','old'],['multi-view','multi']]){
    const viewport=$(id),scene=new THREE.Scene();scene.background=new THREE.Color('#eaf0f6');
    scene.add(new THREE.HemisphereLight(0xffffff,0x8096a8,1.7));
    const light=new THREE.DirectionalLight(0xffffff,2);light.position.set(-3,-4,7);scene.add(light);
    const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});
    renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;viewport.append(renderer.domElement);
    const controls=new OrbitControls(camera,renderer.domElement);controls.target=target;controls.enableDamping=false;controls.maxDistance=40;controls.minDistance=.1;
    controls.addEventListener('change',()=>{if(!state.syncing)draw();});
    const content=new THREE.Group(),highlight=new THREE.Group(),slice=new THREE.Group();scene.add(content,highlight,slice);
    const p={viewport,scene,renderer,controls,content,highlight,slice,type};state.panels.push(p);
    new ResizeObserver(()=>{renderer.setSize(viewport.clientWidth,viewport.clientHeight);if(state.ready)setView(state.view);else draw();}).observe(viewport);
  }
}catch(error){state.webglError=error.message;for(const id of ['cloud-view','old-view','multi-view'])if(!$(id).querySelector('canvas'))$(id).textContent='三维不可用，可查看下方真实剖面及静态图。';}
const enabled=i=>$(`layer-${i}`)?.checked??true;
const offset=i=>$('explode').checked?i*.35:0;
function inputCloud(shift=true){
  const d=state.data,positions=[],rgb=[];
  for(let i=0;i<d.labels.length;i++){
    const layer=d.labels[i];if(layer>=0&&!enabled(layer))continue;
    positions.push(d.points[3*i],d.points[3*i+1],d.points[3*i+2]+(shift&&layer>=0?offset(layer):0));
    new THREE.Color(layer<0?'#82909f':colors[layer%3]).toArray(rgb,rgb.length);
  }
  const g=geometry(positions);g.setAttribute('color',new THREE.Float32BufferAttribute(rgb,3));
  return new THREE.Points(g,new THREE.PointsMaterial({vertexColors:true,size:2.6,sizeAttenuation:false}));
}
function surface(height,indices,color,zoffset=0){
  const d=state.data,vertices=[];
  for(let j=0;j<d.y.length;j++)for(let i=0;i<d.x.length;i++)vertices.push(d.x[i],d.y[j],finite(height[j*d.x.length+i])?height[j*d.x.length+i]+zoffset:0);
  const g=geometry(vertices);g.setIndex(indices);g.computeVertexNormals();
  const mesh=new THREE.Mesh(g,new THREE.MeshStandardMaterial({color,side:THREE.DoubleSide,roughness:.85,polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1}));
  mesh.userData.surface=true;
  const wire=new THREE.LineSegments(new THREE.WireframeGeometry(g),new THREE.LineBasicMaterial({color:'#425e70',transparent:true,opacity:.22}));wire.visible=$('wire').checked;
  return [mesh,wire];
}
function install(){
  const d=state.data;
  for(const p of state.panels){
    dispose(p.content);
    if(p.type==='cloud'||$('overlay').checked){const cloud=inputCloud(p.type!=='old');p.content.add(cloud);}
    if(p.type==='old')surface(d.old_height,d.old_triangles,'#9878b9').forEach(o=>p.content.add(o));
    if(p.type==='multi')for(const layer of d.layers)if(enabled(layer.id)){
      const objects=surface(layer.height,layer.triangles,layer.color,offset(layer.id));
      objects.forEach(o=>{o.userData.layer=layer.id;p.content.add(o);});
    }
    const grid=new THREE.GridHelper(5,10,0xb4c8d5,0xd2dfe7);grid.rotation.x=Math.PI/2;grid.position.z=-.06;p.content.add(grid);
  }
  $('explode-note').textContent=$('explode').checked?'展开显示：第 0/1/2 层分别上移 0/0.35/0.70 m，仅影响点云与多层三维显示；单层对照及下方剖面仍为真实高度。':'三维与剖面均为真实高度；三个视角同步，拖动旋转、滚轮缩放。';
  updateProbe();plotProfile();draw();
}
function setView(name){
  state.view=name;
  if(!state.data||!state.panels.length)return;
  const centre=new THREE.Vector3(0,0,$('explode').checked?.95:.6);
  const aspect=Math.min(...state.panels.map(p=>p.viewport.clientWidth/p.viewport.clientHeight));
  const half=THREE.MathUtils.degToRad(camera.fov/2),fov=Math.min(half,Math.atan(Math.tan(half)*aspect));
  const distance=3.7/Math.sin(fov);
  const dir=new THREE.Vector3(...(name==='side'?[0,-1,.035]:name==='under'?[.55,-1,-.18]:name==='top'?[0,-.0001,1]:[.8,-1,.57])).normalize();
  target.copy(centre);camera.position.copy(centre).addScaledVector(dir,distance);
  state.syncing=true;state.panels.forEach(p=>p.controls.update());state.syncing=false;
  document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===name));draw();
}
function buildInfo(){
  const d=state.data;
  $('summary').replaceChildren();
  for(const [value,label]of [[d.n.toLocaleString(),'输入点'],[30,'每次拟合近邻'],['0.75 m','空间半径'],[`±${Math.round(d.height_gate_m*100)} cm`,'本层高度门限'],[d.layers.length,'独立高度层']]){
    const item=el('span');item.append(el('b',value),label);$('summary').append(item);
  }
  $('layer-toggles').replaceChildren();$('probe-layer').replaceChildren();
  d.layers.forEach(layer=>{
    const label=el('label'),input=el('input');input.type='checkbox';input.id=`layer-${layer.id}`;input.checked=true;input.addEventListener('change',install);
    const dot=el('i');dot.style.background=layer.color;label.append(input,dot,`${layer.name} ${fmt(layer.centre)} m`);$('layer-toggles').append(label);
    const option=el('option',layer.name);option.value=layer.id;$('probe-layer').append(option);
  });
  $('probe-layer').value='2';$('probe').replaceChildren();
  d.probes.forEach((p,i)=>{const option=el('option',`${p.name} (${p.xy.map(v=>fmt(v,2)).join(', ')})`);option.value=i;$('probe').append(option);});
  const table=el('table'),head=el('tr');['层 / 点数','有效覆盖','MAE cm','网格面数'].forEach(t=>head.append(el('th',t)));table.append(head);
  for(const layer of d.layers){const row=el('tr'),e=layer.metrics;[`${layer.name} / ${layer.points}`,`${fmt(e.coverage*100,1)}%`,finite(e.mae_m)?fmt(e.mae_m*100,3):'无有效拟合',(layer.triangles.length/3).toLocaleString()].forEach(t=>row.append(el('td',t)));table.append(row);}
  $('metrics').replaceChildren(table);
  $('outside-note').textContent=`真实表面范围外有效格：${d.layers.map(l=>`${l.name} ${l.metrics.outside_face_queries}`).join('，')}。高度带外 ${d.unassigned} 个点仍显示为灰色；不用于本次水平层拟合。`;
}
function cross(x,y,z){const delta=.055;return [x-delta,y,z,x+delta,y,z,x,y-delta,z,x,y+delta,z,x,y,z-delta,x,y,z+delta];}
function updateProbe(){
  if(!state.data)return;
  const d=state.data,probe=d.probes[Number($('probe').value)||0],i=Number($('probe-layer').value)||0,entry=probe.layers[i],layer=d.layers[i];
  $('probe-info').textContent=`${layer.name}峰值 ${fmt(layer.centre,5)} m；${entry.reason}。半径内共有 ${entry.available} 个本层候选点，选中 ${entry.ids.length} 个。`;
  $('probe-info').className=entry.valid?'valid':'invalid';
  $('probe-details').textContent=entry.ids.length?`近邻 Z 范围 ${entry.neighbor_z_range.map(v=>fmt(v,4)).join(' ～ ')} m；相对本层峰最大高差 ${fmt(entry.max_height_delta*100,2)} cm；最大 3D 距离 ${fmt(entry.max_distance,3)} m。`:'该高度带及半径内没有可用近邻。';
  $('probe-heights').replaceChildren();
  probe.layers.forEach((e,j)=>{const text=el('span',`${d.layers[j].name} Z = ${fmt(e.height,4)}${finite(e.height)?' m':''}`);text.style.color=colors[j%3];$('probe-heights').append(text);});
  for(const p of state.panels){
    dispose(p.highlight);if(!$('show-neighbors').checked)continue;
    const zoffset=p.type==='old'?0:offset(i),positions=[];
    for(const id of entry.ids)positions.push(d.points[3*id],d.points[3*id+1],d.points[3*id+2]+zoffset);
    const selected=new THREE.Points(geometry(positions),new THREE.PointsMaterial({color:'#d92e40',size:5.5,sizeAttenuation:false,depthTest:false}));selected.renderOrder=6;p.highlight.add(selected);
    const z=(finite(entry.height)?entry.height:layer.centre)+zoffset;
    const mark=new THREE.LineSegments(geometry(cross(...probe.xy,z)),new THREE.LineBasicMaterial({color:'#b42a3a',depthTest:false}));mark.renderOrder=7;p.highlight.add(mark);
  }
  draw();
}
const NS='http://www.w3.org/2000/svg';
function svgNode(tag,attrs={},text){const n=document.createElementNS(NS,tag);Object.entries(attrs).forEach(([k,v])=>n.setAttribute(k,v));if(text!==undefined)n.textContent=text;return n;}
function segments(field,maxJump){const out=[];let run=[];field.forEach((v,i)=>{if(finite(v)){if(run.length&&maxJump&&Math.abs(v-field[run.at(-1)])>maxJump){out.push(run);run=[];}run.push(i);}else if(run.length){out.push(run);run=[];}});if(run.length)out.push(run);return out;}
function slice3D(row){
  const d=state.data,n=d.x.length,y=d.y[row];
  for(const p of state.panels){
    dispose(p.slice);const fields=p.type==='old'?[[d.old_height,0]]:p.type==='multi'?d.layers.filter(l=>enabled(l.id)).map(l=>[l.height,offset(l.id)]):[];
    const values=[];
    if(p.type==='cloud')values.push(d.x[0],y,-.03,d.x.at(-1),y,-.03);
    for(const [field,delta]of fields)for(let i=0;i<n-1;i++){
      const a=row*n+i,b=a+1;
      if(finite(field[a])&&finite(field[b])&&(p.type==='old'||Math.abs(field[a]-field[b])<=.04))values.push(d.x[i],y,field[a]+delta,d.x[i+1],y,field[b]+delta);
    }
    const line=new THREE.LineSegments(geometry(values),new THREE.LineBasicMaterial({color:'#263e4f',depthTest:false}));line.renderOrder=4;p.slice.add(line);
  }
}
function plotProfile(){
  if(!state.data)return;
  const d=state.data,n=d.x.length,row=Number($('slice').value),y=d.y[row],fields=d.layers.map(l=>l.height.slice(row*n,(row+1)*n));
  const old=d.old_height.slice(row*n,(row+1)*n);$('slice-value').textContent=`${fmt(y,2)} m`;
  const width=Math.max($('profile').clientWidth,270),height=width<600?280:325,left=47,right=16,top=22,bottom=38,w=width-left-right,h=height-top-bottom;
  const lo=-.09,hi=1.34,sx=x=>left+(x+2.5)/5*w,sy=z=>top+(hi-z)/(hi-lo)*h;
  const svg=svgNode('svg',{viewBox:`0 0 ${width} ${height}`});
  for(const z of [0,.3,.6,.9,1.1,1.2]){
    svg.append(svgNode('line',{x1:left,x2:left+w,y1:sy(z),y2:sy(z),stroke:'#e5ecf2'}));
    svg.append(svgNode('text',{x:left-6,y:sy(z)+4,'text-anchor':'end','font-size':11,fill:'#7b8c9b'},z.toFixed(1)));
  }
  for(const x of [-2,-1,0,1,2]){
    svg.append(svgNode('line',{x1:sx(x),x2:sx(x),y1:top,y2:top+h,stroke:'#edf1f5'}));
    svg.append(svgNode('text',{x:sx(x),y:height-17,'text-anchor':'middle','font-size':11,fill:'#7b8c9b'},x));
  }
  svg.append(svgNode('text',{x:5,y:14,'font-size':11,fill:'#617b8f'},'Z (m)'));
  svg.append(svgNode('text',{x:width-8,y:height-2,'text-anchor':'end','font-size':11,fill:'#617b8f'},'X (m)'));
  for(let i=0;i<d.labels.length;i++)if(Math.abs(d.points[3*i+1]-y)<=.1&&Math.abs(d.points[3*i])<=2.5&&d.points[3*i+2]>=lo&&d.points[3*i+2]<=hi){
    svg.append(svgNode('circle',{cx:sx(d.points[3*i]),cy:sy(d.points[3*i+2]),r:1.9,fill:'#8996a4',opacity:.65}));
  }
  const drawField=(field,color,dash,maxJump,layer)=>{
    for(const ids of segments(field,maxJump))svg.append(svgNode('polyline',{points:ids.map(i=>`${sx(d.x[i])},${sy(field[i])}`).join(' '),fill:'none',stroke:color,'stroke-width':layer===undefined?1.5:2.5,'stroke-dasharray':dash,'data-layer':layer??'old'}));
  };
  if($('show-old').checked)drawField(old,'#9878b9','5 4',null);
  fields.forEach((field,i)=>{if(enabled(i))drawField(field,colors[i%3],'',.04,i);});
  const hair=svgNode('line',{x1:sx(0),x2:sx(0),y1:top,y2:top+h,stroke:'#c0cad4','stroke-dasharray':'2 3'});svg.append(hair);
  const read=i=>{$('profile-value').textContent=`X=${fmt(d.x[i],2)} m；${d.layers.map((l,j)=>`${l.name} ${fmt(fields[j][i],4)}${finite(fields[j][i])?' m':''}`).join('；')}${$('show-old').checked?`；单层 ${fmt(old[i],4)} m`:''}`;};
  svg.addEventListener('pointermove',event=>{const point=svg.createSVGPoint();point.x=event.clientX;point.y=event.clientY;const local=point.matrixTransform(svg.getScreenCTM().inverse());const i=Math.round(Math.min(1,Math.max(0,(local.x-left)/w))*(n-1));hair.setAttribute('x1',sx(d.x[i]));hair.setAttribute('x2',sx(d.x[i]));read(i);});
  $('profile').replaceChildren(svg);read(Math.floor(n/2));slice3D(row);draw();
}
async function loadCase(){
  const request=++state.request,key=`${$('source').value}_${$('gate').value}cm`;
  state.ready=false;$('status').textContent='读取中…';
  try{
    if(!cache.has(key)){const response=await fetch(`./cases/${key}.json`);if(!response.ok)throw Error(`HTTP ${response.status}`);cache.set(key,await response.json());}
    if(request!==state.request)return;
    state.data=cache.get(key);state.ready=true;buildInfo();install();setView(state.view);
    $('status').textContent=`已显示 ${state.data.n.toLocaleString()} 点输入 · ${state.data.layers.length} 层${state.webglError?' · 三维不可用，剖面可查看':''}`;
    $('status').classList.remove('error');
  }catch(error){$('status').textContent=`读取失败：${error.message}`;$('status').classList.add('error');}
}
for(const id of ['source','gate'])$(id).addEventListener('change',loadCase);
for(const id of ['overlay','wire','explode'])$(id).addEventListener('change',()=>{install();if(id==='explode')setView(state.view);});
for(const id of ['probe','probe-layer','show-neighbors'])$(id).addEventListener('change',updateProbe);
for(const id of ['slice','show-old'])$(id).addEventListener('input',plotProfile);
document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>setView(b.dataset.view)));
new ResizeObserver(()=>{if(state.ready)plotProfile();}).observe($('profile'));
window.tableLayers={state,camera,target,loadCase,setView,plotProfile,updateProbe,draw};
loadCase();
