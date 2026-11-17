import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const $=id=>document.getElementById(id),BASE=new URL('./',import.meta.url),fmt=(v,n=3)=>Number.isFinite(v)?v.toFixed(n):'Unknown',num=v=>Number(v).toLocaleString('zh-CN');
const state={ready:false,data:null,catalog:null,seed:0,panels:new Map(),token:0,view:'overall',syncing:false};
const camera=new THREE.PerspectiveCamera(42,1,.005,1500);camera.up.set(0,0,1);const target=new THREE.Vector3();
const specs={foot_cost:['落足综合代价',1,''],body_cost:['机身综合代价',1,''],max_error_m:['最大绝对训练残差',.02,'m'],rmse_m:['训练 RMSE',.02,'m'],std_center_m:['中心预测 σ',.01,'m'],std_patch_m:['面片内最大预测 σ',.01,'m'],error_plus_2sigma_m:['最大残差 + 2σ',.02,'m'],slope_deg:['中心坡度',45,'°'],normal_change_deg:['角点相对中心的法向变化',15,'°'],roughness_cost:['粗糙度代价',1,''],jump_a_m:['A 单侧二阶预测残差',.12,'m'],jump_b_m:['B 双侧同位置外推',.12,'m'],jump_upper_m:['C 双侧跳变 + 2σΔ',.12,'m'],jump_sigma_m:['跳变标准差 σΔ',.03,'m'],jump_significance:['显著性比值 S（不是危险度）',5,''],clearance_cost:['上方 10–60 cm 的观测占用比例',1,''],body_coverage:['支撑不足比例 1−覆盖率',1,''],body_slope_deg:['四足平面坡度',45,'°'],body_plane_rmse_m:['四足共面残差',.02,'m'],body_support_margin_m:['静态裕量不足代价',1,''],body_max_clearance:['机身范围最大净空占用',1,''],body_max_jump_m:['机身范围最大 C 跳变',.12,'m']};
function value(i,key){return state.data.models[i*state.data.meta.fields.length+state.data.columns[key]];}
function xyz(i){return [value(i,'x'),value(i,'y'),value(i,'valid')?value(i,'height'):value(i,'z')];}
function evaluate(i,x,y){const u=x-value(i,'x'),v=y-value(i,'y');return value(i,'z')+value(i,'c0')+value(i,'c1')*u+value(i,'c2')*v+value(i,'c3')*u*u+value(i,'c4')*u*v+value(i,'c5')*v*v;}
function predictionVariance(i,x,y){const u=x-value(i,'x'),v=y-value(i,'y'),phi=[1,u,v,u*u,u*v,v*v],cov=state.data.covariance;let result=0;for(let a=0;a<6;a++)for(let b=0;b<6;b++)result+=phi[a]*cov[i*36+a*6+b]*phi[b];return Math.max(0,result);}
function fieldFor(type){return ['fit','mesh'].includes(type)?$('main-field').value:type==='clearance'?'clearance_cost':$(`${type}-field`)?.value;}
function scaleFor(type,field){return type==='error'&&$('error-scale').value==='scene'&&state.data.validMax[field]>0?state.data.validMax[field]:specs[field][1];}
function displayValue(i,field){const x=value(i,field);return field==='body_coverage'?1-x:field==='body_support_margin_m'?Math.max(0,Math.min(1,(.10-x)/.10)):x;}
function colorFor(x,scale,color=new THREE.Color()){
 if(!Number.isFinite(x))return color.set('#84918a');const t=Math.max(0,Math.min(1,x/scale));
 const stops=[[25,155,96],[229,201,80],[216,50,54]],a=t<.5?stops[0]:stops[1],b=t<.5?stops[1]:stops[2],s=t<.5?2*t:2*t-1;
 return color.setRGB(...a.map((v,i)=>(v+(b[i]-v)*s)/255),THREE.SRGBColorSpace);
}
function geometry(positions){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(positions,3));return g;}
function points(positions,color='#89988f',size=1.4,opacity=1){return new THREE.Points(geometry(positions),new THREE.PointsMaterial({color,size,sizeAttenuation:false,transparent:opacity<1,opacity,depthWrite:opacity===1}));}
function dispose(group){group.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});group.clear();}
function draw(){for(const p of state.panels.values())if(p.details.open){const w=p.viewport.clientWidth,h=p.viewport.clientHeight;if(w&&h){camera.aspect=w/h;camera.updateProjectionMatrix();p.renderer.render(p.scene,camera);}}}
function initPanel(type){
 if(state.panels.has(type))return state.panels.get(type);
 const viewport=$(`${type}-view`),details=viewport.closest('details'),scene=new THREE.Scene();scene.background=new THREE.Color('#e6ece8');
 const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;viewport.append(renderer.domElement);
 const controls=new OrbitControls(camera,renderer.domElement);controls.target=target;controls.enableDamping=false;controls.minDistance=.04;controls.maxDistance=1200;controls.addEventListener('change',()=>{if(!state.syncing)draw();});
 const content=new THREE.Group(),highlight=new THREE.Group();scene.add(content,highlight);const p={type,viewport,details,scene,renderer,controls,content,highlight};state.panels.set(type,p);
 let down=null;renderer.domElement.addEventListener('pointerdown',e=>down=[e.clientX,e.clientY]);renderer.domElement.addEventListener('pointerup',e=>{if(down&&Math.hypot(e.clientX-down[0],e.clientY-down[1])<4&&state.ready)pick(p,e);down=null;});
 new ResizeObserver(()=>{if(viewport.clientWidth&&viewport.clientHeight){renderer.setSize(viewport.clientWidth,viewport.clientHeight);draw();}}).observe(viewport);
 return p;
}
function surface(type){
 const d=state.data,isMesh=type==='mesh',positions=isMesh?d.meshPositions:d.patches,owners=isMesh?null:d.patchOwners;let indices=isMesh?d.mesh:d.patchFaces;
 if($('only').checked){if(isMesh)indices=indices.filter((_,k)=>{const j=Math.floor(k/3)*3;return d.mesh[j]===state.seed||d.mesh[j+1]===state.seed||d.mesh[j+2]===state.seed;});else indices=indices.filter((_,k)=>owners[d.patchFaces[Math.floor(k/3)*3]]===state.seed);}
 if($('slice-layer').checked){const z=xyz(state.seed)[2],original=indices;indices=original.filter((_,k)=>{const j=Math.floor(k/3)*3;return [0,1,2].every(t=>Math.abs(positions[original[j+t]*3+2]-z)<=.100001);});}
 const field=fieldFor(type),scale=scaleFor(type,field),g=geometry(positions);g.setIndex(new THREE.BufferAttribute(indices,1));const colors=new Float32Array(positions.length),color=new THREE.Color();
 for(let k=0;k<positions.length/3;k++)colorFor(displayValue(owners?owners[k]:k,field),scale,color).toArray(colors,k*3);
 g.setAttribute('color',new THREE.BufferAttribute(colors,3));const material=new THREE.MeshBasicMaterial({vertexColors:true,side:THREE.DoubleSide,polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1});
 const mesh=new THREE.Mesh(g,material);mesh.userData.surface=true;mesh.userData.field=field;return mesh;
}
function centers(type){const d=state.data,ids=Array.from({length:d.meta.centers},(_,i)=>i).filter(i=>!$('slice-layer').checked||Math.abs(d.meshPositions[i*3+2]-xyz(state.seed)[2])<=.100001),positions=new Float32Array(ids.length*3),colors=new Float32Array(ids.length*3),color=new THREE.Color(),field=fieldFor(type);ids.forEach((i,k)=>{positions.set(d.meshPositions.subarray(i*3,i*3+3),k*3);colorFor(value(i,'valid')?displayValue(i,field):NaN,scaleFor(type,field),color).toArray(colors,k*3);});const p=points(positions,0xffffff,2.6);p.material.vertexColors=true;p.geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));p.userData.centers=true;return p;}
function installPanel(p){
 if(!state.ready||!p.details.open)return;dispose(p.content);const d=state.data;
 if(p.type==='cloud')p.content.add(points(d.points));
 else{if($('overlay').checked)p.content.add(points(d.points,'#819087',1.1,.18));p.content.add(surface(p.type));if(!$('only').checked)p.content.add(centers(p.type));
  const field=fieldFor(p.type),[label,,unit]=specs[field],scale=scaleFor(p.type,field),text=scale<.0001?scale.toExponential(2):fmt(scale,5);let note=`${label}：绿 0 → 红 ${text}${unit}，超过色标上限仍为红；灰色为 Unknown。`;
  if(p.type==='error'&&$('error-scale').value==='scene')note+=' 色标使用当前场景有效函数的最大值；全零时使用固定参考门限。';
  if(p.type==='fit')note+=` ${num(d.meta.valid)} 个有效函数；显示三角片 ${num(d.meta.patch_triangles)} 个，片数不是函数数。`;
  if(p.type==='mesh')note+=` 组合保留 ${num(d.meta.triangles)} 个三角形；缺口保持缺失。`;
  if(p.type==='clearance')note+=' 16 个观测列的占用比例；未观测到障碍不等于证明自由空间。';
  if(p.type==='jump')note+=' 各候选方向取最大值；B/C 缺少双侧支撑时留空。';
  if(p.type==='body')note+=' 四足齐全且 25 点覆盖率 ≥80%；颜色为当前位置整个机身的评估。';
  document.querySelector(`[data-caption="${p.type}"]`).textContent=note;
 }highlight(p);draw();
}
function installOpen(){for(const details of document.querySelectorAll('[data-panel]'))if(details.open)installPanel(initPanel(details.dataset.panel));}
function line(positions,color,width=1){return new THREE.Line(geometry(new Float32Array(positions.flat())),new THREE.LineBasicMaterial({color,linewidth:width,depthTest:false}));}
function highlight(p){
 dispose(p.highlight);if(!state.ready)return;const d=state.data,i=state.seed,[x,y,z]=xyz(i),o=d.neighborOffsets,ids=d.neighbors.subarray(o[i],o[i+1]),raw=new Float32Array(ids.length*3);ids.forEach((j,k)=>raw.set(d.points.subarray(3*j,3*j+3),3*k));const selected=points(raw,'#793cc4',3.5);selected.material.depthTest=false;selected.renderOrder=10;p.highlight.add(selected);
 const cross=line([[x-.03,y,z+.003],[x+.03,y,z+.003],[x,y,z+.003],[x,y-.03,z+.003],[x,y+.03,z+.003]],'#6c2fad');cross.renderOrder=11;p.highlight.add(cross);
 if(p.type!=='body')return;
 const [l,w]=[.4,.25],outline=line([[-l,-w],[l,-w],[l,w],[-l,w],[-l,-w]].map(([u,v])=>[x+u,y+v,z+.02]),'#6737a6');outline.renderOrder=12;p.highlight.add(outline);
 const offsets=[[-.35,-.2],[.35,-.2],[.35,.2],[-.35,.2]],feet=[],zs=[];
 for(let j=0;j<4;j++){const h=d.contactZ[i*4+j];if(Number.isFinite(h)){feet.push([x+offsets[j][0],y+offsets[j][1],h+.008]);zs.push([j,h]);}}
 if(feet.length){const fp=points(new Float32Array(feet.flat()),'#efb536',10);fp.material.depthTest=false;fp.renderOrder=15;p.highlight.add(fp);}
 if(feet.length===4){const poly=line([...feet,feet[0]],'#8b6b1b');poly.renderOrder=14;p.highlight.add(poly);const gx=zs.reduce((s,[j,h])=>s+offsets[j][0]*h,0)/(4*.35**2),gy=zs.reduce((s,[j,h])=>s+offsets[j][1]*h,0)/(4*.2**2),norm=Math.sqrt(1+gx*gx+gy*gy),dx=-.3*gx/norm,dy=-.3*gy/norm,mean=zs.reduce((s,[,h])=>s+h,0)/4;const com=points(new Float32Array([x+dx,y+dy,mean+gx*dx+gy*dy+.015]),'#202724',9);com.material.depthTest=false;com.renderOrder=16;p.highlight.add(com);}
}
function setView(view){
 state.view=view;if(!state.ready)return;const b=state.data.meta.bounds,c=new THREE.Vector3(...b[0]).add(new THREE.Vector3(...b[1])).multiplyScalar(.5),extent=new THREE.Vector3(...b[1]).sub(new THREE.Vector3(...b[0]));let radius=Math.max(extent.length()/2,.4);if(view==='seed'){c.set(...xyz(state.seed));radius=.8;}
 target.copy(c);const dir=new THREE.Vector3(...(view==='top'?[0,-.0001,1]:view==='side'?[0,-1,.12]:[.7,-1,.7])).normalize();const aspect=Math.min(1,...[...state.panels.values()].filter(p=>p.details.open).map(p=>p.viewport.clientWidth/p.viewport.clientHeight));const half=Math.atan(Math.tan(THREE.MathUtils.degToRad(camera.fov/2))*aspect);camera.position.copy(target).addScaledVector(dir,1.05*radius/Math.sin(half));state.syncing=true;for(const p of state.panels.values())p.controls.update();state.syncing=false;draw();
}
function pick(p,e){const rect=p.renderer.domElement.getBoundingClientRect(),mouse=new THREE.Vector2((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1),ray=new THREE.Raycaster();ray.params.Points.threshold=.07;ray.setFromCamera(mouse,camera);const hits=ray.intersectObjects(p.content.children,false);if(!hits.length)return;const q=hits[0].point;let best=Infinity,chosen=state.seed;for(let i=0;i<state.data.meta.centers;i++){const pos=xyz(i),cost=(q.x-pos[0])**2+(q.y-pos[1])**2+(q.z-pos[2])**2;if(cost<best){best=cost;chosen=i;}}chooseSeed(chosen);}
function chooseSeed(i){if(!state.ready)return;state.seed=Math.max(0,Math.min(state.data.meta.centers-1,Math.round(Number(i)||0)));$('seed-id').value=state.seed;const url=new URL(location.href);url.searchParams.set('scene',state.data.meta.id);url.searchParams.set('seed',state.seed);history.replaceState(null,'',url);inspect();if($('only').checked||$('slice-layer').checked)installOpen();else{for(const p of state.panels.values())if(p.details.open)highlight(p);draw();}}
function centralGround(){let best=Infinity,id=0;const zmin=state.data.meta.bounds[0][2];for(let i=0;i<state.data.meta.centers;i++)if(value(i,'valid')){const cost=value(i,'x')**2+value(i,'y')**2+.03*(value(i,'height')-zmin);if(cost<best){best=cost;id=i;}}return id;}
function inspect(){
 const i=state.seed,d=state.data,v=key=>value(i,key);$('seed-info').textContent=`#${i} · XYZ (${fmt(v('x'))}, ${fmt(v('y'))}, ${fmt(v('z'))}) m；窗口内 ${v('point_count')} 点。${v('valid')?'有效拟合':Number.isFinite(v('rmse_m'))?'已求解但质量不通过':'邻点不足，未求解'}；${v('assembled')?'参与组合':'未参与组合'}。`;
 $('polynomial').textContent=Number.isFinite(v('c0'))?`z=z_seed+c₀+c₁u+c₂v+c₃u²+c₄uv+c₅v²\nu=x−x_seed，v=y−y_seed\n(c₀…c₅)=(${Array.from({length:6},(_,k)=>v('c'+k).toPrecision(6)).join(', ')})\n梯度=(c₁+2c₃u+c₄v, c₂+c₄u+2c₅v)；Hessian=[[2c₃,c₄],[c₄,2c₅]]（不是直接的主曲率）`:'没有有效的已求解系数。';
 const rows=[['rmse_m','训练 RMSE','只衡量本次拟合点'],['max_error_m','最大绝对训练残差','本次拟合点的最大高度差'],['std_center_m','中心预测 σ','φᵀΣφ 开平方'],['std_patch_m','面片最大预测 σ','实际域内采样点最大值'],['error_plus_2sigma_m','误差 + 2σ','最大残差 + 2×面片最大 σ'],['slope_deg','中心坡度','°；45° 为本实验硬门限'],['normal_change_deg','法向变化','°；四个角相对中心最大值'],['jump_a_m','A：单侧预测残差','m；各候选方向最大值'],['jump_b_m','B：双侧同边界','m；各候选方向最大值'],['jump_sigma_m','σΔ','m；该列独立取方向最大值'],['jump_significance','S：显著性','无量纲；不是危险度'],['jump_upper_m','C：保守跳变','m；每对 B+2σΔ 后取最大值'],['jump_known_directions','已知双侧候选数','未必刚好四个，包含邻点反向选中'],['clearance_cost','10–60 cm 占用','原始点与拟合面证据并集 / 16'],['raw_obstacle_hits','原始障碍点','命中数量，不是独立障碍数'],['surface_obstacle_hits','拟合面障碍采样点','命中实际三角形的顶点数'],['roughness_cost','粗糙度代价','RMSE 与法向变化的较大代价'],['foot_cost','落足综合代价','0–1 工程加权'],['body_cost','机身综合代价','0–1 工程加权；Unknown 不填零'],['body_coverage','机身支持覆盖率','25 个网格点中已知支持比例'],['body_slope_deg','四足平面坡度','°；假定固定足位'],['body_plane_rmse_m','四足共面残差','m；四足高度到支撑平面'],['body_support_margin_m','重力投影静态裕量','m；最近支撑边距离'],['body_max_clearance','机身最大净空占用','包含四足与 25 点的最坏观测'],['body_max_jump_m','机身最大保守跳变','m；窗口内已知值最大值']];
 $('selected-metrics').replaceChildren();for(const [key,label,note]of rows){const tr=document.createElement('tr');for(const text of [label,fmt(v(key),5),note]){const td=document.createElement('td');td.textContent=text;tr.append(td);}$('selected-metrics').append(tr);}
 const cov=d.covariance.subarray(i*36,i*36+36);$('covariance').textContent=Array.from({length:6},(_,r)=>Array.from(cov.subarray(r*6,r*6+6)).map(x=>Number.isFinite(x)?x.toExponential(4):'Unknown').join('  ')).join('\n');$('variance-info').textContent=`中心 σ²=${fmt(predictionVariance(i,v('x'),v('y')),9)} m²；域角 (+10,+10 cm) σ²=${fmt(predictionVariance(i,v('x')+.1,v('y')+.1),9)} m²（域角可能已在显示凸包外）。`;
 $('body-info').textContent=`50×80 cm：覆盖率 ${fmt(v('body_coverage')*100,1)}%；四足模型编号 [${Array.from(d.contacts.subarray(i*4,i*4+4)).join(', ')}]。${Number.isFinite(v('body_cost'))?`机身代价 ${fmt(v('body_cost'))}；静态裕量 ${fmt(v('body_support_margin_m')*100,2)} cm。`:'机身综合代价 Unknown：支撑或所需指标不足。'} 这不是动力学稳定性或实机通过率。`;
}
async function loadScene(id){
 const token=++state.token;state.controller?.abort();state.controller=new AbortController();state.ready=false;$('loading').hidden=false;$('loading').textContent='正在读取完整点云、20 cm 模型与评估结果…';
 const get=async file=>{const r=await fetch(new URL(`${id}/${file}`,BASE),{signal:state.controller.signal});if(!r.ok)throw Error(`${file}: HTTP ${r.status}`);return file.endsWith('.json')?r.json():r.arrayBuffer();};
 try{const [meta,...buffers]=await Promise.all(['data.json','points.bin','models.bin','mesh.bin','patches.bin','patch-faces.bin','patch-owners.bin','covariance.bin','body-contacts.bin','body-contact-z.bin','neighbor-offsets.bin','neighbors.bin'].map(get));if(token!==state.token)return;
  const types=[Float32Array,Float64Array,Uint32Array,Float32Array,Uint32Array,Uint32Array,Float64Array,Int32Array,Float64Array,Int32Array,Int32Array],keys=['points','models','mesh','patches','patchFaces','patchOwners','covariance','contacts','contactZ','neighborOffsets','neighbors'],d={meta,columns:Object.fromEntries(meta.fields.map((v,i)=>[v,i]))};buffers.forEach((b,i)=>d[keys[i]]=new types[i](b));
  if(d.models.length!==meta.centers*meta.fields.length||d.points.length!==meta.raw_points*3||d.covariance.length!==meta.centers*36||d.neighborOffsets.length!==meta.centers+1)throw Error('数据长度不匹配');
  state.data=d;d.validMax={};d.meshPositions=new Float32Array(meta.centers*3);for(let i=0;i<meta.centers;i++){d.meshPositions.set(xyz(i),i*3);if(value(i,'valid'))for(const key of ['max_error_m','rmse_m','std_center_m','std_patch_m','error_plus_2sigma_m'])if(Number.isFinite(value(i,key)))d.validMax[key]=Math.max(d.validMax[key]||0,value(i,key));}state.ready=true;
  const params=new URLSearchParams(location.search),seed=params.get('scene')===id&&params.has('seed')?Number(params.get('seed')):centralGround();$('scene-select').value=id;$('seed-id').max=meta.centers-1;
  $('metrics').replaceChildren();for(const [v,label]of [[num(meta.raw_points),'原始点'],[num(meta.centers),'0.10 m 体素中心'],[num(meta.solved),'主窗口实际求解'],[num(meta.valid),'有效函数'],[num(meta.field_stats.body_cost.known),'机身代价已知'],[fmt(meta.timings.stages_ms.total_assessment_ms,1)+' ms','全部新增计算 · 单次']]){const div=document.createElement('div'),b=document.createElement('b'),s=document.createElement('span');b.textContent=v;s.textContent=label;div.append(b,s);$('metrics').append(div);}
  $('source-note').textContent=meta.source.kind==='analytic_regeneration'?'本场景为解析几何重新生成的 2.5 cm 间距点云，用于严格 20 cm 拟合。历史场景原始文件与数值保持独立；没有把插值点伪装成实测点。':'本场景使用原始场景 XYZ，未为本次 20 cm 拟合补点或加密。点不足与质量不通过保持缺失。';
  if(meta.jump_solve_counts)$('source-note').textContent+=` 主窗口求解 ${num(meta.solved)} 次；B/C 共用的双侧重拟合另有 ${num(meta.jump_solve_counts.actual_side_solves)} 次（含重复支撑集），${num(meta.jump_solve_counts.both_sides_valid_pairs)} 个候选对双侧均有效。`;
  for(const [dom,file]of [['csv-link','per-model.csv'],['pairs-link','jump-pairs.csv'],['npz-link','result.npz'],['meta-link','data.json']])$(dom).href=new URL(`${id}/${file}`,BASE);
  chooseSeed(seed);installOpen();setView(state.view);$('loading').hidden=true;
 }catch(e){if(e.name!=='AbortError'){$('loading').textContent=`加载失败：${e.message}`;console.error(e);}}
}
for(const details of document.querySelectorAll('[data-panel]'))details.addEventListener('toggle',()=>{if(details.open&&state.ready){const p=initPanel(details.dataset.panel);installPanel(p);p.renderer.setSize(p.viewport.clientWidth,p.viewport.clientHeight);draw();}});
for(const id of ['main-field','error-field','error-scale','jump-field','foot-field','body-field','overlay','only','slice-layer'])$(id).addEventListener('change',installOpen);
$('expand-all').onclick=()=>document.querySelectorAll('[data-panel]').forEach(d=>d.open=true);$('collapse-all').onclick=()=>document.querySelectorAll('[data-panel]').forEach(d=>d.open=false);
$('scene-select').onchange=()=>loadScene($('scene-select').value);$('choose-seed').onclick=()=>chooseSeed($('seed-id').value);$('seed-id').addEventListener('keydown',e=>{if(e.key==='Enter')chooseSeed($('seed-id').value);});$('center-ground').onclick=()=>{chooseSeed(centralGround());setView('seed');};$('largest-jump').onclick=()=>{let best=-Infinity,id=state.seed;for(let i=0;i<state.data.meta.centers;i++)if(value(i,'valid')&&value(i,'jump_b_m')>best){best=value(i,'jump_b_m');id=i;}chooseSeed(id);setView('seed');};document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>setView(b.dataset.view));
async function start(){const r=await fetch(new URL('catalog.json',BASE));if(!r.ok)throw Error('catalog.json HTTP '+r.status);state.catalog=await r.json();for(const c of state.catalog.scenes){const option=new Option(c.name,c.id);$('scene-select').append(option);const tr=document.createElement('tr');for(const v of [c.name,num(c.raw_points),num(c.centers),num(c.solved),num(c.valid),num(c.field_stats.body_cost.known),fmt(c.timings.stages_ms.total_assessment_ms,1)]){const td=document.createElement('td');td.textContent=v;tr.append(td);}tr.onclick=()=>loadScene(c.id);$('scene-table').append(tr);}const id=new URLSearchParams(location.search).get('scene');await loadScene(state.catalog.scenes.some(c=>c.id===id)?id:state.catalog.default_scene);}
window.terrainAssessment={state,camera,target,value,evaluate,predictionVariance,colorFor,chooseSeed,setView,loadScene,installOpen};start().catch(e=>{$('loading').textContent=e.message;console.error(e);});
