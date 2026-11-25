import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const $=id=>document.getElementById(id),BASE=new URL('../convex-assessment/',import.meta.url);
const fields={
 clearance_cost:{label:'上方占用',factor:100,unit:'%',digits:1,scale:1,note:'原始点在函数上方 10–60 cm 的观测列占用。5 cm 列按与本函数凸包的交叠面积计权；没有障碍观测不等于已证明自由空间。'},
 slope_deg:{label:'坡度 · 平均法向',factor:1,unit:'°',digits:2,scale:45,note:'在凸包内独立的 1 cm XY 网格上，对二次函数求导，归一化局部法向后取平均，再归一化；其与竖直方向的夹角为坡度。对称起伏的法向可能抵消，需要同时看平滑性。'},
 rmse_m:{label:'拟合 RMSE',factor:1000,unit:'mm',digits:2,scale:.02,note:'本次参与拟合的全部合格原始点的竖直残差均方根，是训练残差。'},
 std_patch_m:{label:'不确定性 σ',factor:1000,unit:'mm',digits:2,scale:.01,note:'独立内部网格和凸包顶点上的预测均值标准差，取采样最大值；来自参数协方差，尚未标定为实际误差。'},
 normal_spread_deg:{label:'平滑性 · 法向变化',factor:1,unit:'°',digits:2,scale:15,note:'内部网格的导数单位法向相对平均法向的夹角 RMS。越小越平滑，越大法向变化越明显；不混入拟合残差。'},
 max_error_m:{label:'拟合最大误差',factor:1000,unit:'mm',digits:2,scale:.02,note:'本次参与拟合的全部合格原始点的最大绝对竖直残差，是训练残差。'}
};
const state={ready:false,data:null,catalog:null,profiles:[],gapOptions:{},boundaryOptions:{},profile:'qcircle-adaptive',method:'quadratic',metric:'clearance_cost',angle:'3d',selected:null,selectedEdge:null,token:0,renderToken:0};
const viewport=$('viewport'),scene=new THREE.Scene();scene.background=new THREE.Color('#dfe6ed');
const camera=new THREE.PerspectiveCamera(42,1,.005,5000);camera.up.set(0,0,1);
const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;viewport.append(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);controls.minDistance=.03;controls.maxDistance=2000;controls.addEventListener('change',draw);
const content=new THREE.Group(),highlight=new THREE.Group();scene.add(content,highlight);
const number=x=>Number(x).toLocaleString('zh-CN'),coverage=()=>$('fit-coverage').checked;
const format=(x,spec=fields[state.metric])=>Number.isFinite(x)?`${(x*spec.factor).toFixed(spec.digits)} ${spec.unit}`:'未知';
function value(i,key){const d=state.data;return d.models[i*d.meta.fields.length+d.columns[key]];}
function colorFor(x,scale,color=new THREE.Color()){if(!Number.isFinite(x))return color.set('#8d969f');const t=Math.max(0,Math.min(1,x/scale));return color.setRGB(1,1-t,1-t,THREE.SRGBColorSpace);}
function dispose(group){for(const o of [...group.children]){o.traverse(c=>{c.geometry?.dispose();for(const m of Array.isArray(c.material)?c.material:[c.material])m?.dispose();});group.remove(o);}}
function draw(){if(!viewport.clientWidth||!viewport.clientHeight)return;camera.aspect=viewport.clientWidth/viewport.clientHeight;camera.updateProjectionMatrix();renderer.render(scene,camera);}
new ResizeObserver(()=>{renderer.setSize(viewport.clientWidth,viewport.clientHeight);if(state.ready)frame(camera.position.clone().sub(controls.target).normalize());else draw();}).observe(viewport);
function geometry(positions){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(positions,3));return g;}
function cloud(positions,color,size){return new THREE.Points(geometry(positions),new THREE.PointsMaterial({color,size,sizeAttenuation:false}));}
function heightAt(i,x,y){const u=x-value(i,'x'),v=y-value(i,'y');return value(i,'z')+value(i,'c0')+value(i,'c1')*u+value(i,'c2')*v+value(i,'c3')*u*u+value(i,'c4')*u*v+value(i,'c5')*v*v;}
function share(){if(!state.ready)return;const url=new URL(location.href);for(const key of ['mesh','hide','auto_profile','method','model'])url.searchParams.delete(key);url.searchParams.set('scene',state.data.meta.id);url.searchParams.set('profile',state.profile);url.searchParams.set('metric',state.metric);url.searchParams.set('angle',state.angle);url.searchParams.set('coverage',coverage()?'1':'0');url.searchParams.set('points',$('cloud-overlay').checked?'1':'0');if(state.data.gap)url.searchParams.set('gaps',$('small-gap-fill').checked?'1':'0');else url.searchParams.delete('gaps');if(state.data.boundary)url.searchParams.set('boundary',$('boundary-heights').checked?'1':'0');else url.searchParams.delete('boundary');if(state.selected!==null)url.searchParams.set('model',state.selected);history.replaceState(null,'',url);}
function updateBoundaryInfo(){
 const b=state.data.boundary,on=!!b&&$('boundary-heights').checked;$('boundary-info').hidden=!on;if(!on)return;
 const cm=v=>Number.isFinite(v)?(100*v).toFixed(2)+' cm':'暂无';
 $('boundary-info').innerHTML=`边界高度差：0 cm <i class="boundary-scale"></i> ≥80 cm · ${number(b.meta.region_pairs)} 组邻层 · ${number(b.meta.boundary_samples)} 个边界采样 · 平均 ${cm(b.meta.mean_delta_m)} · 最大 ${cm(b.meta.maximum_delta_m)}。色线独立于上方六项评价指标；仅显示 ≥2 cm 高差。<a href="../convex-assessment/boundary-heights/report.md">计算与剖面图 ↗</a><span id="boundary-selected"></span>`;
 const r=b.meta.records[state.selectedEdge];if(r)$('boundary-selected').textContent=` · 已选边界：函数 #${r.left} ↔ #${r.right}，Δz ${cm(r.delta_m)}`;
}
function updateSelection(){
 dispose(highlight);const i=state.selected;$('clear-selection').hidden=i===null;
 if(i===null){$('selection-info').textContent='点击曲面查看局部函数的数值与边界。';return;}
 const d=state.data,valid=!!value(i,'valid'),r=d.radii[i],p=[value(i,'x'),value(i,'y'),value(i,'z')];
 $('selection-info').textContent=`局部函数 #${i} · ${valid?'拟合有效':'未通过拟合'} · ${number(value(i,'point_count'))} 点${Number.isFinite(r)?` · 半径 ${(100*r).toFixed(1)} cm`:''} · 凸包面积 ${(value(i,'domain_area_m2')*10000).toFixed(1)} cm² · ${fields[state.metric].label} ${format(valid?value(i,state.metric):NaN)}${d.meta.config.center_offset_check===false?` · 中心预测偏移 ${(Math.abs(value(i,'c0'))*100).toFixed(2)} cm`:''} · 中心 (${p.map(v=>v.toFixed(2)).join(', ')}) m`;
 const a=d.polygonOffsets[i],b=d.polygonOffsets[i+1];if(b-a<3)return;
 const xyz=[];for(let k=a;k<=b;k++){const j=k===b?a:k,x=d.polygons[j*2]+p[0],y=d.polygons[j*2+1]+p[1];xyz.push(x,y,heightAt(i,x,y));}
 const line=new THREE.Line(geometry(new Float32Array(xyz)),new THREE.LineBasicMaterial({color:'#203f58',depthTest:false}));line.renderOrder=5;line.userData.selectedPatch=i;highlight.add(line);
}
function updateNumbers(){
 const d=state.data,spec=fields[state.metric];let count=0,sum=0,max=-Infinity;
 for(let i=0;i<d.meta.centers;i++)if(value(i,'valid')&&Number.isFinite(value(i,state.metric))){const v=value(i,state.metric);sum+=v;count++;max=Math.max(max,v);}
 state.stats={known:count,valid:d.meta.valid,mean:count?sum/count:NaN,max:count?max:NaN};
 $('mean-value').textContent=format(state.stats.mean);$('max-value').textContent=format(state.stats.max);$('known-value').textContent=d.meta.valid?(100*count/d.meta.valid).toFixed(1)+'%':'暂无拟合';$('known-value').title=`${number(count)} / ${number(d.meta.valid)} 个有效函数拥有指标值；不是地表面积覆盖率。`;
 $('map-title').textContent=`${d.meta.name} · ${d.profile.label} · ${coverage()?'拟合覆盖':spec.label}`+(d.boundary&&$('boundary-heights').checked?' · 边界高度差':'');
 $('metric-legend').hidden=coverage();$('coverage-legend').hidden=!coverage();$('scale-low').textContent=format(0);$('scale-high').textContent='≥ '+format(spec.scale);
 const gapOn=!!d.gap&&$('small-gap-fill').checked;
 $('fit-summary').textContent=`原始点 ${number(d.meta.raw_points)} · 中心 ${number(d.meta.centers)} · 主拟合求解 ${number(d.meta.actual_solve_count)} 次 · 有效函数 ${number(d.meta.valid)} · 显示函数 ${number(d.meta.displayed_functions)}`+(gapOn?` · 小缝显示补全 ${number(d.gap.meta.filled_gaps)} 处（${d.gap.meta.filled_area_m2.toFixed(3)} m²）`:'');
 $('metric-note').textContent=(coverage()?(gapOn?'蓝色包含原凸包拟合范围与小缝显示补全。':'蓝色为二次函数在自身拟合点凸包内的完整显示范围。'):spec.note+' 白色低、红色高；灰色为指标未知。')+' 凸包内不再因附近点不足删除显示面。凸包不能识别内部观测空洞；这里的覆盖是模型显示范围。'+(gapOn?' 小于 9 cm² 的封闭连通间隙按邻近原函数补齐显示，沿用其指标；大空档与外侧敞口保留，不增加原始点或拟合函数。'+(d.gap.meta.unfilled_small_gaps?` 另有 ${number(d.gap.meta.unfilled_small_gaps)} 处小缝未满足邻近函数范围或边界高度条件，保留空白。`:''):'')+(d.meta.config.center_offset_check===false?' 此对照取消中心预测偏移上限，保留选点高度筛选、至少 15 点、秩检查和 RMSE≤8 mm。':'');
 $('source-info').textContent=`${d.meta.name} · 指标在原函数凸包域独立计算；训练残差不是独立测试精度。`;
 $('csv-link').href=new URL(`${d.meta.id}/per-model.csv`,d.base);updateSelection();updateBoundaryInfo();
}
function paint(){if(!state.ready)return;for(const g of content.children.filter(o=>o.userData.surface)){const owners=g.userData.owners,colors=new Float32Array(owners.length*3),c=new THREE.Color();for(let i=0;i<owners.length;i++){if(coverage())c.set('#2586cb');else colorFor(value(owners[i],state.metric),fields[state.metric].scale,c);c.toArray(colors,i*3);}g.geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));g.userData.field=state.metric;g.userData.coverage=coverage();}updateNumbers();draw();share();}
async function install(){
 if(!state.ready)return;const d=state.data,token=++state.renderToken;
 if($('cloud-overlay').checked&&!d.points)d.points=new Float32Array(await d.get('points.bin'));
 if(d.gap&&$('small-gap-fill').checked&&!d.gap.positions){
  const response=await Promise.all(['positions','faces','owners'].map(async key=>{const r=await fetch(new URL(d.gap.meta.files[key],d.gap.base));if(!r.ok)throw Error('小缝文件加载失败');return r.arrayBuffer();}));
  [d.gap.positions,d.gap.faces,d.gap.owners]=[new Float32Array(response[0]),new Uint32Array(response[1]),new Uint32Array(response[2])];
  if(d.gap.positions.length!==d.gap.owners.length*3||d.gap.faces.length!==3*d.gap.meta.overlay_triangles)throw Error('小缝数据长度不匹配');
 }
 if(d.boundary&&$('boundary-heights').checked&&!d.boundary.lines){
  const b=d.boundary,response=await Promise.all(['lines','values','line_records'].map(async key=>{const r=await fetch(new URL(b.meta.files[key],b.base));if(!r.ok)throw Error('边界色线文件加载失败');return r.arrayBuffer();}));
  [b.lines,b.values,b.lineRecords]=[new Float32Array(response[0]),new Float32Array(response[1]),new Uint32Array(response[2])];
  if(b.lines.length!==b.values.length*3||b.values.length!==b.meta.line_segments*2||b.lineRecords.length!==b.meta.line_segments)throw Error('边界色线数据长度不匹配');
 }
 if(d!==state.data||token!==state.renderToken)return;dispose(content);
 const g=geometry(d.patches);g.setIndex(new THREE.BufferAttribute(d.patchFaces,1));const surface=new THREE.Mesh(g,new THREE.MeshBasicMaterial({vertexColors:true,side:THREE.DoubleSide,polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1}));surface.userData={surface:true,mode:'patch',owners:d.patchOwners};content.add(surface);
 if(d.gap&&$('small-gap-fill').checked&&d.gap.faces.length){const h=geometry(d.gap.positions);h.setIndex(new THREE.BufferAttribute(d.gap.faces,1));const fill=new THREE.Mesh(h,new THREE.MeshBasicMaterial({vertexColors:true,side:THREE.DoubleSide,polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1}));fill.userData={surface:true,gapFill:true,owners:d.gap.owners};content.add(fill);}
 if(d.boundary&&$('boundary-heights').checked&&d.boundary.lines.length){const b=d.boundary,g=geometry(b.lines),colors=new Float32Array(b.values.length*3),c=new THREE.Color();for(let k=0;k<b.values.length;k++)colorFor(b.values[k],b.meta.config.color_max_m,c).toArray(colors,k*3);g.setAttribute('color',new THREE.BufferAttribute(colors,3));const lines=new THREE.LineSegments(g,new THREE.LineBasicMaterial({vertexColors:true,depthTest:false,linewidth:2}));lines.renderOrder=6;lines.userData.boundary=true;content.add(lines);}
 const rejected=[];for(let i=0;i<d.meta.centers;i++)if(!value(i,'valid'))rejected.push(i);
 if(rejected.length){const xyz=new Float32Array(rejected.length*3);rejected.forEach((i,k)=>xyz.set([value(i,'x'),value(i,'y'),value(i,'z')],k*3));const o=cloud(xyz,'#c27619',2.2);o.userData.modelIds=rejected;content.add(o);}
 if($('cloud-overlay').checked)content.add(cloud(d.points,'#98a3ae',1.1));paint();state.renderedScene=d.meta.id;state.renderedProfile=d.profile.id;state.renderedMethod='quadratic';
}
function frame(dir){const bounds=state.data.meta.bounds,center=new THREE.Vector3(...bounds[0]).add(new THREE.Vector3(...bounds[1])).multiplyScalar(.5);camera.position.copy(center).add(dir);camera.lookAt(center);const right=new THREE.Vector3(1,0,0).applyQuaternion(camera.quaternion),up=new THREE.Vector3(0,1,0).applyQuaternion(camera.quaternion),tanV=Math.tan(THREE.MathUtils.degToRad(camera.fov/2)),tanH=tanV*viewport.clientWidth/viewport.clientHeight;let distance=.2;for(const x of [bounds[0][0],bounds[1][0]])for(const y of [bounds[0][1],bounds[1][1]])for(const z of [bounds[0][2],bounds[1][2]]){const q=new THREE.Vector3(x,y,z).sub(center);distance=Math.max(distance,q.dot(dir)+Math.abs(q.dot(right))/tanH,q.dot(dir)+Math.abs(q.dot(up))/tanV);}controls.target.copy(center);camera.position.copy(center).addScaledVector(dir,distance*1.08);controls.update();draw();}
function setAngle(angle){state.angle=['3d','top','side'].includes(angle)?angle:'3d';document.querySelectorAll('[data-angle]').forEach(b=>b.classList.toggle('active',b.dataset.angle===state.angle));if(state.ready){frame(new THREE.Vector3(...(state.angle==='top'?[0,-.001,1]:state.angle==='side'?[0,-1,.16]:[.65,-1,.8])).normalize());share();}}
function select(i,edge=null){if(!state.ready)return;state.selected=i===null?null:Math.max(0,Math.min(state.data.meta.centers-1,Math.round(i)));state.selectedEdge=edge;updateSelection();updateBoundaryInfo();share();draw();}
let down=null;renderer.domElement.addEventListener('pointerdown',e=>down=[e.clientX,e.clientY]);renderer.domElement.addEventListener('pointerup',e=>{if(!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>4||!state.ready)return;down=null;const rect=renderer.domElement.getBoundingClientRect(),ray=new THREE.Raycaster();ray.params.Line.threshold=.025;ray.setFromCamera(new THREE.Vector2((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1),camera);const hits=ray.intersectObjects(content.children.filter(o=>o.userData.boundary||o.userData.surface||o.userData.modelIds),false),hit=hits.find(h=>h.object.userData.boundary)||hits[0];if(hit?.object.userData.boundary){const index=state.data.boundary.lineRecords[Math.floor(hit.index/2)],r=state.data.boundary.meta.records[index];select(r.left,index);}else if(hit)select(hit.object.userData.modelIds?hit.object.userData.modelIds[hit.index]:(hit.object.userData.owners||state.data.patchOwners)[hit.face.a]);});
function fail(e){$('loading').hidden=false;$('loading').textContent='加载失败：'+e.message;console.error(e);}
async function loadScene(id,profileId=state.profile){
 const available=state.profiles.filter(p=>!p.scene_ids||p.scene_ids.includes(id));
 const profile=available.find(p=>p.id===profileId)||available.find(p=>p.id==='qcircle-adaptive')||available[0];if(!profile)throw Error('尚未生成拟合结果');
 $('profile-select').replaceChildren(...available.map(p=>new Option(p.label,p.id)));$('profile-select').value=profile.id;
 const keep=state.data?.meta.id===id,base=new URL(profile.path,BASE),token=++state.token;state.controller?.abort();state.controller=new AbortController();const signal=state.controller.signal;
 state.ready=false;$('loading').hidden=false;$('loading').textContent='正在读取凸包曲面…';
 const get=async file=>{const r=await fetch(new URL(`${id}/${file}`,base),{signal});if(!r.ok)throw Error(`${file}: HTTP ${r.status}`);return file.endsWith('.json')?r.json():r.arrayBuffer();};
 try{
  const files=['data.json','models.bin','patches.bin','patch-faces.bin','patch-owners.bin','radii.bin','polygons.bin','polygon-offsets.bin'];const [meta,models,patches,faces,owners,radii,polygons,offsets]=await Promise.all(files.map(get));if(token!==state.token)return;
  const d={meta,profile,base,get,models:new Float64Array(models),patches:new Float32Array(patches),patchFaces:new Uint32Array(faces),patchOwners:new Uint32Array(owners),radii:new Float32Array(radii),polygons:new Float64Array(polygons),polygonOffsets:new Int32Array(offsets),columns:Object.fromEntries(meta.fields.map((k,i)=>[k,i]))};
  const gapPath=state.gapOptions[id]?.[profile.id];
  if(gapPath){const gapBase=new URL(gapPath,BASE),r=await fetch(new URL('data.json',gapBase),{signal});if(!r.ok)throw Error('小缝说明加载失败');const gapMeta=await r.json();if(gapMeta.valid_functions!==meta.valid||gapMeta.raw_points!==meta.raw_points)throw Error('小缝与拟合结果不匹配');d.gap={meta:gapMeta,base:gapBase};}
  const boundaryPath=state.boundaryOptions[id]?.[profile.id];
  if(boundaryPath){const boundaryBase=new URL(boundaryPath,BASE),r=await fetch(new URL('data.json',boundaryBase),{signal});if(!r.ok)throw Error('边界高度差说明加载失败');const boundaryMeta=await r.json();if(boundaryMeta.valid_functions!==meta.valid||boundaryMeta.raw_points!==meta.raw_points)throw Error('边界与拟合结果不匹配');d.boundary={meta:boundaryMeta,base:boundaryBase};}
  if(token!==state.token)return;$('boundary-option').hidden=!d.boundary;state.selectedEdge=null;
  if(token!==state.token)return;$('small-gap-option').hidden=!d.gap;
  if(d.models.length!==meta.centers*meta.fields.length||d.patches.length!==d.patchOwners.length*3||d.patchFaces.length!==meta.patch_triangles*3||d.polygonOffsets.length!==meta.centers+1)throw Error('数据长度不匹配');
  state.data=d;state.profile=profile.id;d.patchCounts=new Uint32Array(meta.centers);for(let k=0;k<d.patchFaces.length;k+=3)d.patchCounts[d.patchOwners[d.patchFaces[k]]]++;
  const params=new URLSearchParams(location.search),model=params.get('scene')===id&&params.get('profile')===profile.id&&params.has('model')?Number(params.get('model')):NaN;state.selected=Number.isInteger(model)&&model>=0&&model<meta.centers?model:null;
  $('scene-select').value=id;$('profile-select').value=profile.id;state.ready=true;await install();if(token!==state.token)return;if(!keep)setAngle(state.angle);else{draw();share();}$('loading').hidden=true;
 }catch(e){if(e.name!=='AbortError'&&token===state.token)fail(e);}
}
async function start(){
 if(window.metricLegacyRedirect)return;const params=new URLSearchParams(location.search);
 if((params.get('method')||'').startsWith('fsgp-')){const old=new URL('../metrics-history/',import.meta.url);old.search=location.search;location.replace(old);return;}
 const responses=await Promise.all(['catalog.json','profiles.json','gap-options.json','boundary-options.json'].map(f=>fetch(new URL(f,BASE))));for(const r of responses)if(!r.ok)throw Error('尚未生成凸包评价结果');const [catalog,profiles,gapOptions,boundaryOptions]=await Promise.all(responses.map(r=>r.json()));state.catalog=catalog;state.profiles=profiles.profiles;state.gapOptions=gapOptions;state.boundaryOptions=boundaryOptions;
 if(params.has('scene')&&!catalog.scenes.some(s=>s.id===params.get('scene'))){const old=new URL('../metrics-history/',import.meta.url);old.search=location.search;location.replace(old);return;}
 for(const s of catalog.scenes)$('scene-select').append(new Option(s.name,s.id));for(const [key,spec]of Object.entries(fields))$('metric-select').append(new Option(spec.label,key));
 state.metric=Object.hasOwn(fields,params.get('metric'))?params.get('metric'):'clearance_cost';$('metric-select').value=state.metric;state.profile=state.profiles.some(p=>p.id===params.get('profile'))?params.get('profile'):profiles.default_profile;state.angle=params.get('angle')||'3d';$('fit-coverage').checked=params.has('coverage')?params.get('coverage')==='1':!params.has('metric');$('cloud-overlay').checked=params.get('points')==='1';
 $('small-gap-fill').checked=params.get('gaps')!=='0';$('small-gap-fill').onchange=()=>install().catch(fail);
 $('boundary-heights').checked=params.get('boundary')==='1';$('boundary-heights').onchange=()=>install().catch(fail);
 $('scene-select').onchange=()=>loadScene($('scene-select').value);$('profile-select').onchange=()=>loadScene($('scene-select').value,$('profile-select').value);$('metric-select').onchange=()=>{state.metric=$('metric-select').value;$('fit-coverage').checked=false;paint();};$('fit-coverage').onchange=paint;$('cloud-overlay').onchange=()=>install().catch(fail);$('clear-selection').onclick=()=>select(null);for(const b of document.querySelectorAll('[data-angle]'))b.onclick=()=>setAngle(b.dataset.angle);
 await loadScene(params.get('scene')||catalog.default_scene);
}
window.metricViewer={state,renderer,camera,controls,content,highlight,fields,value,colorFor,select,loadScene,setAngle,paint};start().catch(fail);
