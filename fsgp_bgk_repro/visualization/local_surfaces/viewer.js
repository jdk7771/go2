import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const $=id=>document.getElementById(id),finite=Number.isFinite,fmt=(x,n=3)=>finite(x)?x.toFixed(n):'无支持';
const state={ready:false,data:null,errors:null,errorsReady:false,panels:[],view:'orbit',seed:0,syncing:false,webglError:null};
const camera=new THREE.PerspectiveCamera(42,1,.01,100),target=new THREE.Vector3();camera.up.set(0,0,1);
const element=(tag,text)=>{const e=document.createElement(tag);if(text!==undefined)e.textContent=text;return e;};
function geometry(positions){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));return g;}
function dispose(group){group.traverse(o=>{o.geometry?.dispose();if(o.material)(Array.isArray(o.material)?o.material:[o.material]).forEach(m=>m.dispose());});group.clear();}
function draw(){for(const p of state.panels){camera.aspect=p.viewport.clientWidth/p.viewport.clientHeight;camera.updateProjectionMatrix();p.renderer.render(p.scene,camera);}}
try{
 for(const [id,type]of [['cloud-view','cloud'],['patch-view','patch'],['mesh-view','mesh']]){
  const viewport=$(id),scene=new THREE.Scene();scene.background=new THREE.Color('#eaf0f6');scene.add(new THREE.HemisphereLight(0xffffff,0x8499a9,1.7));
  const light=new THREE.DirectionalLight(0xffffff,2);light.position.set(-3,-4,7);scene.add(light);
  const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;viewport.append(renderer.domElement);
  const controls=new OrbitControls(camera,renderer.domElement);controls.target=target;controls.enableDamping=false;controls.minDistance=.1;controls.maxDistance=45;controls.addEventListener('change',()=>{if(!state.syncing)draw();});
  const content=new THREE.Group(),highlight=new THREE.Group();scene.add(content,highlight);state.panels.push({viewport,type,scene,renderer,controls,content,highlight});
  new ResizeObserver(()=>{renderer.setSize(viewport.clientWidth,viewport.clientHeight);if(state.ready)setView(state.view);else draw();}).observe(viewport);
 }
}catch(e){state.webglError=e.message;for(const id of ['cloud-view','patch-view','mesh-view'])if(!$(id).querySelector('canvas'))$(id).textContent='三维不可用，可查看下方静态图及剖面。';}
const enabled=i=>$(`component-${i}`)?.checked??true;
function cloud(unknownOnly=false){
 const d=state.data,positions=[],colors=[];
 for(let i=0;i<d.n;i++){
  const label=d.labels[i];if(unknownOnly&&label>=0)continue;
  const color=!unknownOnly&&$('back-color').checked&&label>=0?d.components[label].color:'#8c98a3';
  positions.push(...d.points.slice(i*3,i*3+3));new THREE.Color(color).toArray(colors,colors.length);
 }
 const g=geometry(positions);g.setAttribute('color',new THREE.Float32BufferAttribute(colors,3));return new THREE.Points(g,new THREE.PointsMaterial({vertexColors:true,size:unknownOnly?2.8:2.6,sizeAttenuation:false}));
}
function surface(positions,indices,color,component,vertexColors=null){
 const g=geometry(positions);g.setIndex(indices);g.computeVertexNormals();
 if(vertexColors)g.setAttribute('color',new THREE.Float32BufferAttribute(vertexColors,3));
 const mesh=new THREE.Mesh(g,new THREE.MeshStandardMaterial({color:vertexColors?0xffffff:color,vertexColors:!!vertexColors,side:THREE.DoubleSide,roughness:.86,polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1}));mesh.userData.surface=true;mesh.userData.component=component;
 const wire=new THREE.LineSegments(new THREE.WireframeGeometry(g),new THREE.LineBasicMaterial({color:'#456377',transparent:true,opacity:.22}));wire.visible=$('wire').checked;return [mesh,wire];
}
function install(){
 const d=state.data,vertices=[];
 const errorMetric=state.errorsReady?$('patch-error-color').value:'none';
 const rgb=errorMetric==='none'?null:patchErrorColors(errorMetric);
 for(let i=0;i<d.n;i++)vertices.push(d.points[3*i],d.points[3*i+1],finite(d.seed_height[i])?d.seed_height[i]:d.points[3*i+2]);
 const indicesByComponent=new Map();
 for(let j=0;j<d.patch_triangles.length;j+=3){
  const first=d.patch_triangles[j],seed=d.patch_owners[first],label=rgb?-3:$('back-color').checked?d.labels[seed]:-2;
  if($('only-seed').checked&&seed!==state.seed)continue;
  if(!indicesByComponent.has(label))indicesByComponent.set(label,[]);indicesByComponent.get(label).push(...d.patch_triangles.slice(j,j+3));
 }
 for(const p of state.panels){
  dispose(p.content);
  if(p.type==='cloud')p.content.add(cloud());
  else if($('unknown').checked)p.content.add(cloud(true));
  if(p.type==='patch')for(const [label,indices]of indicesByComponent)surface(d.patch_vertices,indices,label===-2?'#5291bc':label<0?'#8c98a3':d.components[label].color,label,rgb).forEach(o=>p.content.add(o));
  if(p.type==='mesh')for(const component of d.components)if(enabled(component.id))surface(vertices,component.triangles,component.color,component.id).forEach(o=>p.content.add(o));
  const grid=new THREE.GridHelper(6,12,0xb4c8d5,0xd2dfe7);grid.rotation.x=Math.PI/2;grid.position.set(.4,0,-.065);p.content.add(grid);
 }
 $('patch-caption').textContent=rgb?`每片按其 30 邻点的${errorMetric==='fit_rmse_m'?'拟合残差':'解析几何距离'} RMSE 着色；不是组件颜色`:$('only-seed').checked?`只显示种子 #${state.seed} 的一个局部二次函数；红点为它的近邻`:$('back-color').checked?'按最终组件回看着色；实际计算仍然是先独立拟合，再分组':'独立拟合统一蓝色，尚未按组件着色；每片只画 24×24 cm，面片之间没有连网';
 $('error-color-scale').hidden=!rgb;
 if(rgb)$('error-color-label').textContent=`0 — ${fmt(state.errors.color_limits_m[errorMetric]*100,1)} cm；每片一个整体 RMSE`;
 updateSeed();plotProfile();draw();
}
function setView(name){
 state.view=name;if(!state.ready||!state.panels.length)return;
 const aspect=Math.min(...state.panels.map(p=>p.viewport.clientWidth/p.viewport.clientHeight));
 const half=THREE.MathUtils.degToRad(camera.fov/2),fov=Math.min(half,Math.atan(Math.tan(half)*aspect));
 const dir=new THREE.Vector3(...(name==='side'?[0,-1,.035]:name==='under'?[.5,-1,-.15]:name==='top'?[0,-.0001,1]:[.8,-1,.57])).normalize();
 if(name==='seed'){
  $('only-seed').checked=true;install();const d=state.data,i=state.seed;
  target.set(d.points[3*i],d.points[3*i+1],finite(d.seed_height[i])?d.seed_height[i]:d.points[3*i+2]);
 }else target.set(.4,0,.6);
 camera.position.copy(target).addScaledVector(dir,(name==='seed'?.42:4.1)/Math.sin(fov));
 state.syncing=true;state.panels.forEach(p=>p.controls.update());state.syncing=false;
 document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===name));draw();
}
function info(){
 const d=state.data,s=d.summary;$('summary').replaceChildren();
 for(const [value,label]of [[d.n.toLocaleString(),'原始种子'],[30,'邻点 / 拟合'],[s.valid_seed_fits.toLocaleString(),'有效局部拟合'],[d.components.length,'自动主面片'],[s.triangles.toLocaleString(),'组合三角形']]){
  const item=element('span');item.append(element('b',value),label);$('summary').append(item);
 }
 $('component-toggles').replaceChildren();
 for(const c of d.components){const label=element('label'),input=element('input'),dot=element('i');input.type='checkbox';input.id=`component-${c.id}`;input.checked=true;input.addEventListener('change',install);dot.style.background=c.color;label.append(input,dot,`${c.name} · Z中位数 ${fmt(c.median_z)} m`);$('component-toggles').append(label);}
 $('probe').replaceChildren(element('option','手动选择种子编号'));$('probe').firstChild.value='';
 for(const p of d.probes){const option=element('option',`${p.name} #${p.seed}`);option.value=p.seed;$('probe').append(option);}
 const highest=d.components.reduce((a,b)=>a.median_z>b.median_z?a:b);const probe=d.probes[highest.id];
 state.seed=probe.seed;$('probe').value=String(state.seed);$('seed-id').value=state.seed;$('seed-id').max=d.n-1;
 const table=element('table'),head=element('tr');['自动组件','有效种子','Z中位数 m','三角形'].forEach(t=>head.append(element('th',t)));table.append(head);
 for(const c of d.components){const row=element('tr');[c.id+1,c.points,fmt(c.median_z,5),(c.triangles.length/3).toLocaleString()].forEach(t=>row.append(element('td',t)));table.append(row);}
 $('metrics').replaceChildren(table);$('unknown-info').textContent=`${s.insufficient_neighbor_points} 个种子邻点不足，${s.quality_rejected_points} 个拟合质量未通过，${s.small_component_valid_points} 个有效种子属于小碎片；共 ${s.unassigned_points} 个原始点未归入主要组件。`;
}
function updateSeed(){
 const d=state.data,i=state.seed,ids=d.neighbor_ids[i].filter(j=>j>=0),height=d.seed_height[i],label=d.labels[i],p=d.points.slice(3*i,3*i+3);
 $('seed-info').textContent=`种子 #${i} XYZ=(${p.map(v=>fmt(v,4)).join(', ')}) m；${label>=0?d.components[label].name:'未分组'}；半径及高度带内 ${d.candidate_counts[i]} 点，选中 ${ids.length} 点。${finite(height)?`拟合 Z=${fmt(height,5)} m。`:'此处未生成有效曲面。'}`;
 if(ids.length){const z=ids.map(j=>d.points[3*j+2]),maxDelta=Math.max(...z.map(v=>Math.abs(v-p[2])));$('neighbor-info').textContent=`近邻 Z：${fmt(Math.min(...z),4)} ～ ${fmt(Math.max(...z),4)} m；相对种子最大高差 ${fmt(maxDelta*100,2)} cm；拟合 RMSE ${fmt(d.fit_rmse[i]*100,2)} cm。`;}else $('neighbor-info').textContent='候选不足 30 个，未跨高度补点。';
 const c=d.coefficients[i];$('polynomial').textContent=finite(height)?`z−z_seed = c₀+c₁u+c₂v+c₃u²+c₄uv+c₅v²\nu=x−x_seed，v=y−y_seed\n(c₀…c₅) = (${c.map(v=>v.toPrecision(4)).join(', ')})`:'该位置没有通过局部拟合检查。';
 for(const panel of state.panels){dispose(panel.highlight);if(!$('show-neighbors').checked)continue;
  const vertices=ids.flatMap(j=>d.points.slice(3*j,3*j+3));const selected=new THREE.Points(geometry(vertices),new THREE.PointsMaterial({color:'#d92e40',size:5.6,sizeAttenuation:false,depthTest:false}));selected.renderOrder=6;panel.highlight.add(selected);
  const [x,y]=p,z=finite(height)?height:p[2],delta=.055;
  const mark=new THREE.LineSegments(geometry([x-delta,y,z,x+delta,y,z,x,y-delta,z,x,y+delta,z,x,y,z-delta,x,y,z+delta]),new THREE.LineBasicMaterial({color:'#b42b3b',depthTest:false}));mark.renderOrder=7;panel.highlight.add(mark);
 }draw();if(state.errorsReady)updateErrorDetails();
}
const NS='http://www.w3.org/2000/svg';
function svgNode(tag,attrs={},text){const n=document.createElementNS(NS,tag);Object.entries(attrs).forEach(([k,v])=>n.setAttribute(k,v));if(text!==undefined)n.textContent=text;return n;}
function segments(field){const output=[];let run=[];field.forEach((v,i)=>{if(finite(v))run.push(i);else if(run.length){output.push(run);run=[];}});if(run.length)output.push(run);return output;}
function plotProfile(){
 if(!state.ready)return;
 const d=state.data,n=d.x.length,row=Number($('slice').value),y=d.y[row],fields=d.components.map(c=>c.profile_height.slice(row*n,(row+1)*n));$('slice-value').textContent=`${fmt(y,2)} m`;
 const width=Math.max($('profile').clientWidth,270),height=width<600?280:325,left=47,right=16,top=22,bottom=38,w=width-left-right,h=height-top-bottom;
 const lo=-.09,hi=1.34,sx=x=>left+(x+2.5)/5*w,sy=z=>top+(hi-z)/(hi-lo)*h;
 const svg=svgNode('svg',{viewBox:`0 0 ${width} ${height}`});
 for(const z of [0,.3,.6,.9,1.1,1.2]){svg.append(svgNode('line',{x1:left,x2:left+w,y1:sy(z),y2:sy(z),stroke:'#e5ecf2'}));svg.append(svgNode('text',{x:left-6,y:sy(z)+4,'text-anchor':'end','font-size':11,fill:'#7b8c9b'},z.toFixed(1)));}
 for(const x of [-2,-1,0,1,2]){svg.append(svgNode('line',{x1:sx(x),x2:sx(x),y1:top,y2:top+h,stroke:'#edf1f5'}));svg.append(svgNode('text',{x:sx(x),y:height-17,'text-anchor':'middle','font-size':11,fill:'#7b8c9b'},x));}
 svg.append(svgNode('text',{x:5,y:14,'font-size':11,fill:'#617b8f'},'Z (m)'));svg.append(svgNode('text',{x:width-8,y:height-2,'text-anchor':'end','font-size':11,fill:'#617b8f'},'X (m)'));
 for(let i=0;i<d.n;i++)if(Math.abs(d.points[3*i+1]-y)<=.1&&Math.abs(d.points[3*i])<=2.5)svg.append(svgNode('circle',{cx:sx(d.points[3*i]),cy:sy(d.points[3*i+2]),r:1.9,fill:'#8996a4',opacity:.65}));
 d.components.forEach((c,j)=>{if(enabled(j))for(const run of segments(fields[j]))svg.append(svgNode('polyline',{points:run.map(i=>`${sx(d.x[i])},${sy(fields[j][i])}`).join(' '),fill:'none',stroke:c.color,'stroke-width':2.5,'data-component':j}));});
 const read=i=>{$('profile-value').textContent=`X=${fmt(d.x[i],2)} m；${d.components.map((c,j)=>`${c.name} ${fmt(fields[j][i],4)}${finite(fields[j][i])?' m':''}`).join('；')}`;};
 const hair=svgNode('line',{x1:sx(0),x2:sx(0),y1:top,y2:top+h,stroke:'#bdc9d3','stroke-dasharray':'2 3'});svg.append(hair);
 svg.addEventListener('pointermove',event=>{const p=svg.createSVGPoint();p.x=event.clientX;p.y=event.clientY;const q=p.matrixTransform(svg.getScreenCTM().inverse());const i=Math.round(Math.min(1,Math.max(0,(q.x-left)/w))*(n-1));hair.setAttribute('x1',sx(d.x[i]));hair.setAttribute('x2',sx(d.x[i]));read(i);});
 $('profile').replaceChildren(svg);read(50);
}
const errorStops=['#440154','#3b528b','#21918c','#5ec962','#fde725'].map(c=>new THREE.Color(c));
function patchErrorColors(metric){
 const max=state.errors.color_limits_m[metric],rgb=[];
 for(const seed of state.data.patch_owners){
  const value=state.errors.records[seed][metric],t=Math.max(0,Math.min(1,finite(value)?value/max:0))*4,index=Math.min(3,Math.floor(t));
  errorStops[index].clone().lerp(errorStops[index+1],t-index).toArray(rgb,rgb.length);
 }return rgb;
}
function errorSubset(){
 if(!state.errorsReady)return [];
 const metric=$('error-metric').value,filter=$('error-filter').value;
 return state.errors.records.filter(r=>finite(r[metric])&&(filter==='solved'||filter==='valid'&&r.valid||filter==='rejected'&&r.solved&&!r.valid||filter.startsWith('component:')&&r.component===Number(filter.split(':')[1])));
}
function plotErrorChart(){
 if(!state.errorsReady)return;
 const metric=$('error-metric').value,records=errorSubset(),width=Math.max($('error-plot').clientWidth,270),height=310,left=46,right=15,top=19,bottom=39,w=width-left-right,h=height-top-bottom;
 const limit=Math.max(.01,...records.map(r=>r[metric]))*1.08,sx=x=>left+x/(state.errors.records.length-1)*w,sy=z=>top+(1-z/limit)*h;
 const svg=svgNode('svg',{viewBox:`0 0 ${width} ${height}`,'aria-label':`逐个局部函数的${metric==='fit_rmse_m'?'拟合残差':'解析几何距离'} RMSE`});
 for(let i=0;i<=4;i++){
  const z=limit*i/4;svg.append(svgNode('line',{x1:left,x2:left+w,y1:sy(z),y2:sy(z),stroke:'#e4ebf1'}));
  svg.append(svgNode('text',{x:left-6,y:sy(z)+4,'text-anchor':'end','font-size':11,fill:'#788d9d'},(z*100).toFixed(1)));
 }
 for(const i of [0,500,1000,1500,2000])svg.append(svgNode('text',{x:sx(i),y:height-16,'text-anchor':'middle','font-size':11,fill:'#788d9d'},i));
 svg.append(svgNode('text',{x:3,y:13,'font-size':11,fill:'#617b8f'},'RMSE (cm)'));
 svg.append(svgNode('text',{x:width-4,y:height-1,'text-anchor':'end','font-size':11,fill:'#617b8f'},'种子编号'));
 if(metric==='fit_rmse_m'&&limit>=.02)svg.append(svgNode('line',{x1:left,x2:left+w,y1:sy(.02),y2:sy(.02),stroke:'#d89b4f','stroke-dasharray':'5 4'}));
 for(const r of records){
  const x=sx(r.seed),y=sy(r[metric]),selected=r.seed===state.seed;
  const node=svgNode(r.valid?'circle':'g',{'class':'curve-error-point','data-seed':r.seed,'data-valid':r.valid});
  if(r.valid){Object.entries({cx:x,cy:y,r:selected?4.5:1.9,fill:'#367caa',stroke:selected?'#1c4055':'none','stroke-width':2}).forEach(([k,v])=>node.setAttribute(k,v));}
  else{node.append(svgNode('circle',{cx:x,cy:y,r:5,fill:'transparent'}));node.append(svgNode('path',{d:`M${x-3},${y-3}L${x+3},${y+3}M${x-3},${y+3}L${x+3},${y-3}`,stroke:'#d24742','stroke-width':selected?2.5:1.5}));}
  node.append(svgNode('title',{},`种子 #${r.seed}：${(r[metric]*100).toFixed(4)} cm，${r.valid?'通过检查':'求解后未通过'}`));
  node.addEventListener('click',()=>changeSeed(r.seed));svg.append(node);
 }
 if(!records.length)svg.append(svgNode('text',{x:width/2,y:height/2,'text-anchor':'middle',fill:'#7b8c9b'},'此筛选范围没有误差记录'));
 $('error-plot').replaceChildren(svg);
 $('error-definition').textContent=metric==='fit_rmse_m'?'拟合 RMSE = sqrt(mean((函数预测高度−30个原始观测高度)²))。这是训练残差，不是独立测试准确率。':'解析几何 RMSE：函数在30个邻点XY处的预测点到最近无噪声暴露表面的欧氏距离，再求RMSE。只作事后评价，未参与拟合；不是对应面高度误差或独立新扫描测试。';
}
function plotNeighborErrors(record){
 const holder=$('neighbor-error-plot');
 if(!record.solved){holder.replaceChildren(element('p','邻点不足，未求解，没有残差值。'));return;}
 const values=record.neighbor_residuals_m,width=Math.max(holder.clientWidth,260),height=205,left=42,right=10,top=15,bottom=30,w=width-left-right,h=height-top-bottom;
 const bound=Math.max(.01,...values.map(v=>Math.abs(v)))*1.13,sy=z=>top+(bound-z)/(2*bound)*h;
 const svg=svgNode('svg',{viewBox:`0 0 ${width} ${height}`});
 for(const z of [-bound,0,bound]){svg.append(svgNode('line',{x1:left,x2:left+w,y1:sy(z),y2:sy(z),stroke:z===0?'#b1c3ce':'#e7edf2'}));svg.append(svgNode('text',{x:left-5,y:sy(z)+4,'text-anchor':'end','font-size':10,fill:'#7a8e9d'},(z*100).toFixed(1)));}
 const ids=state.data.neighbor_ids[record.seed];
 values.forEach((v,i)=>{const x=left+(i+.15)*w/values.length,zero=sy(0),end=sy(v),bar=svgNode('rect',{x,y:Math.min(zero,end),width:w/values.length*.7,height:Math.max(.7,Math.abs(end-zero)),fill:'#4b8aaf','data-neighbor':ids[i]});bar.append(svgNode('title',{},`第${i+1}点，原始ID #${ids[i]}：${(v*100).toFixed(4)} cm`));svg.append(bar);});
 for(const j of [1,5,10,15,20,25,30])svg.append(svgNode('text',{x:left+(j-.5)*w/30,y:height-12,'text-anchor':'middle','font-size':10,fill:'#7a8e9d'},j));
 svg.append(svgNode('text',{x:4,y:11,'font-size':10,fill:'#617b8f'},'残差 cm'));holder.replaceChildren(svg);
}
function updateErrorDetails(){
 if(!state.errorsReady)return;
 const r=state.errors.records[state.seed];$('selected-error-title').textContent=`当前种子 #${r.seed} 的局部函数`;
 if(!r.solved){$('selected-error-info').textContent=`候选只有 ${r.candidate_count} 点，不足30，未求解；这里不将误差计为0。`;$('selected-range-info').textContent='';}
 else{
  $('selected-error-info').textContent=`${r.valid?'已求解并通过局部检查':'已求解但未通过检查'}；拟合 RMSE ${fmt(r.fit_rmse_m*100,3)} cm；解析几何距离 RMSE ${fmt(r.truth_neighbor_rmse_m*100,3)} cm；中心预测点到真实表面距离 ${fmt(r.truth_center_distance_m*100,3)} cm。`;
  $('selected-range-info').textContent=`本次30点（含种子自身）：XY最远半径 ${fmt(r.radius_xy_m)} m，3D最远半径 ${fmt(r.radius_3d_m)} m；XY包围框 ${fmt(r.width_x_m)}×${fmt(r.width_y_m)} m；最大点对距离 ${fmt(r.diameter_xy_m)} m；凸包面积 ${fmt(r.hull_area_m2)} m²。`;
 }
 const s=state.errors.summary.valid_statistics;
 $('range-summary').textContent=`全部有效函数的邻域统计：XY最远半径中位数 ${fmt(s.radius_xy_m.median)} m，P95 ${fmt(s.radius_xy_m.p95)} m；最大点对距离中位数 ${fmt(s.diameter_xy_m.median)} m。24×24 cm 只是曲面的显示范围。`;
 plotNeighborErrors(r);plotErrorChart();
}
async function loadErrors(){
 try{
  const response=await fetch('./errors/data.json');if(!response.ok)throw Error(`HTTP ${response.status}`);
  state.errors=await response.json();state.errorsReady=true;
  const s=state.errors.summary;$('error-counts').textContent=`${s.seeds} 个原始种子 → ${s.solved} 次实际求解（${s.valid} 个通过、${s.rejected} 个剔除）；${s.insufficient} 个不足30点而未求解。这里每个点代表一个局部函数，不是网格三角形。`;
  for(const c of state.data.components){const option=element('option',`${c.name} 的函数`);option.value=`component:${c.id}`;$('error-filter').append(option);}
  const ex=state.errors.triangle_example;
  $('triangle-example-values').textContent=`真实例子：三角形顶点 #${ex.triangle_ids.join('、#')}。在 A=#${ex.a} 的同一个位置，A自己的模型预测 ${fmt(ex.at_a.a_model*100,2)} cm，B=#${ex.b} 的模型预测 ${fmt(ex.at_a.b_model*100,2)} cm，相差 ${fmt(ex.worst_disagreement_m*100,2)} cm，所以该候选三角形没有保留。这不表示真实地面有洞。`;
  install();
 }catch(error){$('error-counts').textContent=`逐曲面误差未加载：${error.message}；可查看静态误差图或先运行误差分析脚本。`;}
}
function changeSeed(i){state.seed=Math.min(state.data.n-1,Math.max(0,Math.round(Number(i)||0)));$('seed-id').value=state.seed;$('probe').value=String(state.seed);if($('only-seed').checked)install();else updateSeed();}
for(const id of ['wire','unknown','only-seed'])$(id).addEventListener('change',install);
$('back-color').addEventListener('change',()=>{if($('back-color').checked)$('patch-error-color').value='none';install();});
$('patch-error-color').addEventListener('change',()=>{if($('patch-error-color').value!=='none')$('back-color').checked=false;install();});
for(const id of ['error-metric','error-filter'])$(id).addEventListener('change',plotErrorChart);
$('worst-error').addEventListener('click',()=>{const records=errorSubset(),metric=$('error-metric').value;if(records.length)changeSeed(records.reduce((a,b)=>a[metric]>b[metric]?a:b).seed);});
$('show-neighbors').addEventListener('change',updateSeed);$('probe').addEventListener('change',()=>{if($('probe').value!=='')changeSeed($('probe').value);});$('seed-id').addEventListener('change',()=>changeSeed($('seed-id').value));$('slice').addEventListener('input',plotProfile);
document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>setView(b.dataset.view)));
new ResizeObserver(()=>{if(state.ready)plotProfile();}).observe($('profile'));
new ResizeObserver(()=>{if(state.errorsReady)updateErrorDetails();}).observe($('error-plot'));
window.tableLocal={state,camera,target,install,setView,plotProfile,changeSeed,draw};
try{const response=await fetch('./data.json');if(!response.ok)throw Error(`HTTP ${response.status}`);state.data=await response.json();state.ready=true;info();install();setView('orbit');$('status').textContent=`已显示 ${state.data.n} 点 · ${state.data.components.length} 个自动主面片${state.webglError?' · 三维不可用，剖面可查看':''}`;await loadErrors();}catch(e){$('status').textContent=`读取失败：${e.message}`;$('status').classList.add('error');}
