import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

const $=id=>document.getElementById(id),BASE=new URL('../assessment/',import.meta.url);
// Raw values stay in their stored units; only the displayed number is converted.
const fields={
 jump_b_m:{label:'突变高度 · 双侧',scale:.12,factor:100,unit:'cm',digits:2,note:'双侧曲面在同一候选位置的高度差；红端是显示范围，不是抬腿切换阈值。'},
 slope_deg:{label:'坡度',scale:45,factor:1,unit:'°',digits:1,note:'局部曲面中心的坡度。'},
 roughness_cost:{label:'粗糙度',scale:1,factor:1,unit:'',digits:3,note:'由拟合残差与法向变化构成的工程代价，范围 0–1。'},
 max_error_m:{label:'最大拟合残差',scale:.02,factor:1000,unit:'mm',digits:2,note:'本次拟合点的最大绝对残差，不是独立测试精度。'},
 rmse_m:{label:'拟合 RMSE',scale:.02,factor:1000,unit:'mm',digits:2,note:'本次拟合点的均方根残差，不是独立测试精度。'},
 std_patch_m:{label:'预测不确定性 σ',scale:.01,factor:1000,unit:'mm',digits:2,note:'实际面片域内采样的最大预测均值标准差，尚未做传感器标定。'},
 clearance_cost:{label:'上方占用',scale:1,factor:100,unit:'%',digits:1,note:'局部曲面上方 10–60 cm 的观测列占用比例；没有观测到障碍不等于已证明自由空间。'},
 foot_cost:{label:'落足评价',scale:1,factor:1,unit:'',digits:3,note:'综合坡度、粗糙度、突变、占用和不确定性的工程代价，范围 0–1。'},
 body_cost:{label:'机身评价 · 80×50 cm',scale:1,factor:1,unit:'',digits:3,note:'80×50 cm 窗口的几何综合代价，范围 0–1；不是实机通过概率。'},
 jump_a_m:{label:'突变 A · 单侧预测',scale:.12,factor:100,unit:'cm',digits:2,note:'已有局部函数预测对方中心的残差；噪声和连续折角也会增大该值。'},
 jump_upper_m:{label:'突变 C · 加不确定性',scale:.12,factor:100,unit:'cm',digits:2,note:'双侧突变 B＋2σΔ；不确定性也会使颜色变红，不能直接据此决定抬腿。'}
};
const state={ready:false,data:null,catalog:null,metric:'jump_b_m',angle:'3d',selected:null,token:0,renderToken:0};
const viewport=$('viewport'),scene=new THREE.Scene();scene.background=new THREE.Color('#dfe6ed');
const camera=new THREE.PerspectiveCamera(42,1,.005,5000);camera.up.set(0,0,1);
const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.localClippingEnabled=true;viewport.append(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);controls.minDistance=.03;controls.maxDistance=2000;controls.addEventListener('change',draw);
const content=new THREE.Group(),highlight=new THREE.Group();scene.add(content,highlight);
const format=(x,spec=fields[state.metric])=>Number.isFinite(x)?`${(x*spec.factor).toFixed(spec.digits)}${spec.unit?' '+spec.unit:''}`:'未知';
const number=x=>Number(x).toLocaleString('zh-CN');
function value(i,key){const d=state.data;return d.models[i*d.meta.fields.length+d.columns[key]];}
function xyz(i){return Array.from(state.data.positions.subarray(i*3,i*3+3));}
function colorFor(x,scale,color=new THREE.Color()){
 if(!Number.isFinite(x))return color.set('#8d969f');
 const t=Math.max(0,Math.min(1,x/scale));return color.setRGB(1,1-t,1-t,THREE.SRGBColorSpace);
}
function dispose(group){for(const object of [...group.children]){object.geometry?.dispose();object.material?.dispose();group.remove(object);}}
function draw(){const w=viewport.clientWidth,h=viewport.clientHeight;if(!w||!h)return;camera.aspect=w/h;camera.updateProjectionMatrix();renderer.render(scene,camera);}
new ResizeObserver(()=>{renderer.setSize(viewport.clientWidth,viewport.clientHeight);if(state.ready)frame(camera.position.clone().sub(controls.target).normalize());else draw();}).observe(viewport);
function geometry(positions){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(positions,3));return g;}
const coverage=()=>$('fit-coverage').checked;
function clipping(){return $('hide-overhead').checked?[new THREE.Plane(new THREE.Vector3(0,0,-1),state.data.meta.bounds[0][2]+.10)]:[];}
function pointCloud(positions,color,size=1.3){return new THREE.Points(geometry(positions),new THREE.PointsMaterial({color,size,sizeAttenuation:false,clippingPlanes:clipping()}));}
function share(){if(!state.ready)return;const url=new URL(location.href);url.searchParams.set('scene',state.data.meta.id);url.searchParams.set('metric',state.metric);url.searchParams.set('angle',state.angle);url.searchParams.set('mesh',$('mesh-mode').checked?'1':'0');url.searchParams.set('points',$('cloud-overlay').checked?'1':'0');url.searchParams.set('hide',$('hide-overhead').checked?'1':'0');url.searchParams.set('coverage',coverage()?'1':'0');if(state.selected===null)url.searchParams.delete('model');else url.searchParams.set('model',state.selected);history.replaceState(null,'',url);}
function updateNumbers(){
 if(!state.ready)return;const d=state.data,spec=fields[state.metric];let sum=0,count=0,max=-Infinity;
 for(let i=0;i<d.meta.centers;i++)if(value(i,'valid')&&Number.isFinite(value(i,state.metric))){const v=value(i,state.metric);sum+=v;count++;max=Math.max(max,v);}
 state.stats={known:count,valid:d.meta.valid,mean:count?sum/count:NaN,max:count?max:NaN};
 $('mean-value').textContent=format(state.stats.mean);$('max-value').textContent=format(state.stats.max);$('known-value').textContent=d.meta.valid?(count/d.meta.valid*100).toFixed(1)+'%':'暂无拟合';$('known-value').title=`${number(count)} / ${number(d.meta.valid)} 个有效函数有该指标`;
 const combined=$('mesh-mode').checked,missing=combined?d.coverage.validUnassembled:d.coverage.validNoPatch;
 $('map-title').textContent=`${d.meta.name} · ${coverage()?(combined?'组合覆盖':'拟合覆盖'):spec.label}`;$('scale-low').textContent=format(0);$('scale-high').textContent='≥ '+format(spec.scale);
 $('metric-legend').hidden=coverage();$('coverage-legend').hidden=!coverage();$('unshown-label').textContent=combined?'有效但未组合':'有效但未成面';
 $('metric-note').textContent=(coverage()?'蓝色只画实际保留的曲面；橙色点未拟合，紫色点拟合有效但当前没有面。':spec.note+' 白色实体面已拟合，表示指标低；灰色实体面已拟合，但该指标未知。')+' 背景色空隙没有显示曲面，不能仅凭空隙认定拟合失败。'+(combined?' 当前为组合网格；连接检查也会产生缺口。'+(coverage()?'':' 颜色在顶点间插值，点选显示最近顶点函数的值。'):' 独立面片受观测点凸包和 ±4 cm 高度范围裁剪。')+($('hide-overhead').checked?' 当前已裁剪：仅显示全局最低点以上 10 cm。':'')+(!d.meta.valid?' 本场景没有有效拟合，灰色原始点只作参考。':'');
 $('fit-summary').textContent=`有效拟合 ${number(d.meta.valid)} / ${number(d.meta.centers)}（${(100*d.meta.valid/d.meta.centers).toFixed(1)}%） · 点数不足 ${number(d.coverage.insufficient)} · 求解后未通过 ${number(d.coverage.rejected)} · ${combined?'有效但未组合':'有效但未成面'} ${number(missing)}`;
 $('fit-summary').title='按局部拟合中心计数；不是点云点数比例，也不是曲面覆盖面积。';
 $('source-info').textContent=`原始点 ${number(d.meta.raw_points)} · ${number(d.meta.centers)} 个拟合中心 · 20×20 cm，至少 ${d.meta.config.minimum_points} 点，窗口内全部合格点参与拟合`;
 $('details-link').href=new URL(`details.html?scene=${d.meta.id}${state.selected===null?'':'&seed='+state.selected}`,BASE);
 $('old-viewer-link').href=new URL(`../scene-fits/?scene=${d.meta.id}`,BASE);$('csv-link').href=new URL(`${d.meta.id}/per-model.csv`,BASE);
 updateSelection();
}
function paint(){
 if(!state.ready)return;const object=content.children.find(o=>o.userData.surface);
 if(object){const d=state.data,isMesh=object.userData.mode==='mesh',positions=object.geometry.attributes.position.array,colors=new Float32Array(positions.length),color=new THREE.Color();for(let k=0;k<positions.length/3;k++){const i=isMesh?k:d.patchOwners[k];if(coverage())color.set('#2586cb');else colorFor(value(i,state.metric),fields[state.metric].scale,color);color.toArray(colors,k*3);}object.geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));object.userData.field=state.metric;object.userData.coverage=coverage();}
 let edges=content.children.find(o=>o.userData.coverageEdges);
 if(coverage()&&object&&!edges){edges=new THREE.LineSegments(new THREE.EdgesGeometry(object.geometry,20),new THREE.LineBasicMaterial({color:'#163f61',transparent:true,opacity:.35,clippingPlanes:clipping()}));edges.userData={coverageEdges:true};content.add(edges);}
 if(edges)edges.visible=coverage();
 updateNumbers();draw();share();
}
async function extra(d,key,file,Type){if(d[key])return d[key];d.pending??={};if(!d.pending[key])d.pending[key]=d.get(file).then(b=>d[key]=new Type(b));return d.pending[key];}
async function install(){
 if(!state.ready)return;const d=state.data,token=++state.renderToken,isMesh=$('mesh-mode').checked;
 try{
  const mesh=isMesh?await extra(d,'mesh','mesh.bin',Uint32Array):null;
  const raw=$('cloud-overlay').checked||!d.meta.valid?await extra(d,'points','points.bin',Float32Array):null;
  if(d!==state.data||token!==state.renderToken)return;dispose(content);
  if(d.meta.valid){const g=geometry(isMesh?d.positions:d.patches);g.setIndex(new THREE.BufferAttribute(isMesh?mesh:d.patchFaces,1));const object=new THREE.Mesh(g,new THREE.MeshBasicMaterial({vertexColors:true,side:THREE.DoubleSide,clippingPlanes:clipping(),polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1}));object.userData={surface:true,mode:isMesh?'mesh':'patch'};content.add(object);}
  // A missing function is a marker, never an invented surface. Keep it distinct
  // from both low-valued white surfaces and grey metric-unknown surfaces.
  for(const [kind,color,predicate]of [['unfitted','#c27619',i=>!value(i,'valid')],['unshown','#8c5db7',i=>value(i,'valid')&&!(isMesh?value(i,'assembled'):d.patchCounts[i])]]){
   const ids=[];for(let i=0;i<d.meta.centers;i++)if(predicate(i))ids.push(i);
   if(ids.length){const object=pointCloud(new Float32Array(ids.flatMap(xyz)),color,2.2);object.userData={kind,modelIds:ids};content.add(object);}
  }
  if(raw)content.add(pointCloud(raw,'#98a3ae',1.1));paint();updateSelection();draw();state.renderedScene=d.meta.id;
 }catch(e){if(e.name!=='AbortError')throw e;}
}
function frame(dir){
 const b=state.data.meta.bounds.map(p=>p.slice());if($('hide-overhead').checked)b[1][2]=Math.min(b[1][2],b[0][2]+.1);
 const center=new THREE.Vector3(...b[0]).add(new THREE.Vector3(...b[1])).multiplyScalar(.5);
 camera.position.copy(center).add(dir);camera.lookAt(center);const right=new THREE.Vector3(1,0,0).applyQuaternion(camera.quaternion),up=new THREE.Vector3(0,1,0).applyQuaternion(camera.quaternion),tanV=Math.tan(THREE.MathUtils.degToRad(camera.fov/2)),tanH=tanV*viewport.clientWidth/viewport.clientHeight;let distance=.2;
 for(const x of [b[0][0],b[1][0]])for(const y of [b[0][1],b[1][1]])for(const z of [b[0][2],b[1][2]]){const q=new THREE.Vector3(x,y,z).sub(center);distance=Math.max(distance,q.dot(dir)+Math.abs(q.dot(right))/tanH,q.dot(dir)+Math.abs(q.dot(up))/tanV);}
 controls.target.copy(center);camera.position.copy(center).addScaledVector(dir,distance*1.08);controls.update();draw();
}
function setAngle(angle){
 state.angle=['3d','top','side'].includes(angle)?angle:'3d';document.querySelectorAll('[data-angle]').forEach(b=>b.classList.toggle('active',b.dataset.angle===state.angle));
 if(!state.ready)return;frame(new THREE.Vector3(...(state.angle==='top'?[0,-.001,1]:state.angle==='side'?[0,-1,.16]:[.65,-1,.8])).normalize());share();
}
function updateSelection(){
 dispose(highlight);const i=state.selected;$('clear-selection').hidden=i===null;
 if(i===null){$('selection-info').textContent='点击曲面查看该处的具体数值。';return;}
 const d=state.data,p=xyz(i),valid=!!value(i,'valid'),n=value(i,'point_count');
 const reason=valid?'拟合有效':n<d.meta.config.minimum_points?`未拟合：仅 ${n} 点，不足 ${d.meta.config.minimum_points} 点`:value(i,'rmse_m')>d.meta.config.max_rmse_m?'未通过：RMSE 超过 2 cm':Math.abs(value(i,'c0'))>d.meta.config.height_gate_m?'未通过：中心偏移超过 4 cm':'未通过：秩或数值检查';
 $('selection-info').textContent=`${valid?'局部函数':'拟合中心'} #${i} · ${reason} · ${n} 点 · ${d.patchCounts[i]} 个独立面片三角形 · ${fields[state.metric].label} ${format(valid?value(i,state.metric):NaN)} · XYZ (${p.map(v=>v.toFixed(2)).join(', ')}) m`;
 // Outline only the selected function's actual triangles, with no cross-patch
 // connections. A flat fitted quadratic legitimately has zero curvature.
 if(d.patchCounts[i]){const selected=d.patchFaceIds[i],vertices=new Float32Array(selected.length*3);for(let k=0;k<selected.length;k++)vertices.set(d.patches.subarray(selected[k]*3,selected[k]*3+3),k*3);const g=geometry(vertices),wire=new THREE.WireframeGeometry(g);g.dispose();const object=new THREE.LineSegments(wire,new THREE.LineBasicMaterial({color:'#243e58',depthTest:false,clippingPlanes:clipping()}));object.userData={selectedPatch:i};object.renderOrder=9;highlight.add(object);}
 const [x,y,z]=p,vertices=new Float32Array([x-.04,y,z+.005,x+.04,y,z+.005,x,y-.04,z+.005,x,y+.04,z+.005]),g=geometry(vertices),mark=new THREE.LineSegments(g,new THREE.LineBasicMaterial({color:'#242a31',depthTest:false,clippingPlanes:clipping()}));mark.renderOrder=10;highlight.add(mark);
}
function select(i){if(!state.ready)return;state.selected=i===null?null:Math.max(0,Math.min(state.data.meta.centers-1,Math.round(i)));updateSelection();updateNumbers();share();draw();}
let down=null;renderer.domElement.addEventListener('pointerdown',e=>down=[e.clientX,e.clientY]);
renderer.domElement.addEventListener('pointerup',e=>{
 if(!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>4||!state.ready){down=null;return;}down=null;
 const rect=renderer.domElement.getBoundingClientRect(),ray=new THREE.Raycaster();ray.params.Points.threshold=.05;ray.setFromCamera(new THREE.Vector2((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1),camera);
 const limit=$('hide-overhead').checked?state.data.meta.bounds[0][2]+.100001:Infinity,hit=ray.intersectObjects(content.children.filter(o=>o.userData.surface||o.isPoints),false).find(h=>h.point.z<=limit);if(!hit)return;
 if(hit.object.userData.modelIds){select(hit.object.userData.modelIds[hit.index]);return;}
 if(hit.object.userData.surface&&hit.object.userData.mode==='patch'){select(state.data.patchOwners[hit.face.a]);return;}
 let best=Infinity,id=null;const positions=state.data.positions,candidates=hit.object.userData.surface?[hit.face.a,hit.face.b,hit.face.c]:Array.from({length:state.data.meta.centers},(_,i)=>i);for(const i of candidates){const k=i*3,dist=(hit.point.x-positions[k])**2+(hit.point.y-positions[k+1])**2+(hit.point.z-positions[k+2])**2;if(dist<best){best=dist;id=i;}}select(id);
});
function fail(error){$('loading').hidden=false;$('loading').textContent='加载失败：'+error.message;console.error(error);}
async function loadScene(id){
 const token=++state.token;state.controller?.abort();state.controller=new AbortController();state.ready=false;$('loading').hidden=false;$('loading').textContent='正在读取评价结果…';
 const signal=state.controller.signal,get=async file=>{const r=await fetch(new URL(`${id}/${file}`,BASE),{signal});if(!r.ok)throw Error(`${file}: HTTP ${r.status}`);return file.endsWith('.json')?r.json():r.arrayBuffer();};
 try{
  const [meta,models,patches,faces,owners]=await Promise.all(['data.json','models.bin','patches.bin','patch-faces.bin','patch-owners.bin'].map(get));if(token!==state.token)return;
  const d={meta,models:new Float64Array(models),patches:new Float32Array(patches),patchFaces:new Uint32Array(faces),patchOwners:new Uint32Array(owners),columns:Object.fromEntries(meta.fields.map((f,i)=>[f,i])),get};
  if(d.models.length!==meta.centers*meta.fields.length||d.patches.length!==d.patchOwners.length*3||d.patchFaces.length!==meta.patch_triangles*3)throw Error('评价数据长度不匹配');
  state.data=d;d.positions=new Float32Array(meta.centers*3);for(let i=0;i<meta.centers;i++)d.positions.set([value(i,'x'),value(i,'y'),value(i,'valid')?value(i,'height'):value(i,'z')],i*3);
  d.patchCounts=new Uint32Array(meta.centers);d.patchFaceIds=Array.from({length:meta.centers},()=>[]);
  for(let k=0;k<d.patchFaces.length;k+=3){const i=d.patchOwners[d.patchFaces[k]];d.patchCounts[i]++;d.patchFaceIds[i].push(...d.patchFaces.subarray(k,k+3));}
  d.coverage={insufficient:0,rejected:0,validNoPatch:0,validUnassembled:0};for(let i=0;i<meta.centers;i++){if(!value(i,'valid')){if(value(i,'point_count')<meta.config.minimum_points)d.coverage.insufficient++;else d.coverage.rejected++;}else{if(!d.patchCounts[i])d.coverage.validNoPatch++;if(!value(i,'assembled'))d.coverage.validUnassembled++;}}
  const p=new URLSearchParams(location.search),same=p.get('scene')===id,model=same&&p.has('model')?Number(p.get('model')):NaN;
  state.selected=Number.isInteger(model)&&model>=0&&model<meta.centers?model:null;
  $('scene-select').value=id;const overhead=id.startsWith('corridor-')||id==='table';$('occlusion-option').hidden=!overhead;$('hide-overhead').checked=overhead&&same&&p.get('hide')==='1';
  state.ready=true;await install();if(token!==state.token)return;setAngle(state.angle);$('loading').hidden=true;
 }catch(e){if(e.name!=='AbortError'&&token===state.token)fail(e);}
}
async function start(){
 if(window.metricLegacyRedirect)return;
 const r=await fetch(new URL('catalog.json',BASE));if(!r.ok)throw Error('尚未生成评价场景：HTTP '+r.status);state.catalog=await r.json();
 for(const c of state.catalog.scenes)$('scene-select').append(new Option(c.name,c.id));for(const [key,f]of Object.entries(fields))$('metric-select').append(new Option(f.label,key));
 const params=new URLSearchParams(location.search);state.metric=Object.hasOwn(fields,params.get('metric'))?params.get('metric'):'jump_b_m';$('metric-select').value=state.metric;state.angle=params.get('angle')||'3d';$('mesh-mode').checked=params.get('mesh')==='1';$('cloud-overlay').checked=params.get('points')==='1';$('fit-coverage').checked=params.get('coverage')==='1';
 $('scene-select').onchange=()=>loadScene($('scene-select').value);$('metric-select').onchange=()=>{state.metric=$('metric-select').value;paint();};
 $('fit-coverage').onchange=paint;for(const id of ['mesh-mode','cloud-overlay'])$(id).onchange=()=>install().catch(fail);$('hide-overhead').onchange=()=>install().then(()=>frame(camera.position.clone().sub(controls.target).normalize())).catch(fail);for(const b of document.querySelectorAll('[data-angle]'))b.onclick=()=>setAngle(b.dataset.angle);$('clear-selection').onclick=()=>select(null);
 const requested=params.get('scene');await loadScene(state.catalog.scenes.some(c=>c.id===requested)?requested:'steps');
}
window.metricViewer={state,renderer,camera,controls,content,highlight,fields,value,colorFor,select,loadScene,setAngle,paint};
start().catch(fail);
