import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const BASE=new URL('./',import.meta.url),$=id=>document.getElementById(id),fmt=(v,n=3)=>Number.isFinite(v)?Number(v).toFixed(n):'无值',number=v=>v.toLocaleString('zh-CN');
const state={ready:false,catalog:null,data:null,panels:[],seed:0,view:'overall',syncing:false,loadToken:0,controller:null};
const camera=new THREE.PerspectiveCamera(42,1,.008,1200),target=new THREE.Vector3();camera.up.set(0,0,1);
const palette=['#328f79','#4c83c5','#dd9c40','#9975b8','#b96883','#58a6a7','#ada248'];
function geo(positions){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(positions,3));return g;}
function dispose(group){group.traverse(o=>{o.geometry?.dispose();if(o.material)(Array.isArray(o.material)?o.material:[o.material]).forEach(m=>m.dispose());});group.clear();}
function draw(){for(const p of state.panels){camera.aspect=p.viewport.clientWidth/p.viewport.clientHeight;camera.updateProjectionMatrix();p.renderer.render(p.scene,camera);}}
for(const [id,type]of [['cloud-view','cloud'],['fit-view','fit']]){
 const viewport=$(id),scene=new THREE.Scene();scene.background=new THREE.Color('#eaf0f5');scene.add(new THREE.HemisphereLight(0xffffff,0x829aaa,1.9));
 const light=new THREE.DirectionalLight(0xffffff,2.2);light.position.set(-5,-8,12);scene.add(light);
 const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;viewport.append(renderer.domElement);
 const controls=new OrbitControls(camera,renderer.domElement);controls.target=target;controls.enableDamping=false;controls.minDistance=.05;controls.maxDistance=1000;controls.addEventListener('change',()=>{if(!state.syncing)draw();});
 const content=new THREE.Group(),highlight=new THREE.Group();scene.add(content,highlight);const panel={viewport,type,scene,renderer,controls,content,highlight};state.panels.push(panel);
 let down=null;renderer.domElement.addEventListener('pointerdown',e=>{down=[e.clientX,e.clientY];});
 renderer.domElement.addEventListener('pointerup',e=>{if(down&&Math.hypot(e.clientX-down[0],e.clientY-down[1])<4&&state.ready)pick(panel,e);down=null;});
 new ResizeObserver(()=>{renderer.setSize(viewport.clientWidth,viewport.clientHeight);if(state.ready)setView(state.view);else draw();}).observe(viewport);
}
function seedPoint(i){const d=state.data.seeds;return new THREE.Vector3(d[i*14],d[i*14+1],d[i*14+2]);}
function value(i,column){return state.data.seeds[i*14+column];}
function isStatusMode(){return ['fit-status','assembly-status'].includes($('color-mode').value);}
function statusBlue(i){return $('color-mode').value==='fit-status'?!!value(i,12):!!state.data.assembled[i];}
function statusCenters(only){
 const d=state.data,n=only?1:d.meta.seed_count,positions=new Float32Array(n*3),colors=new Float32Array(n*3),color=new THREE.Color();
 for(let k=0;k<n;k++){const i=only?state.seed:k;positions.set(d.meshPositions.subarray(i*3,i*3+3),k*3);color.set(statusBlue(i)?'#568eb6':'#ffffff').toArray(colors,k*3);}
 const points=pointObject(positions,0xffffff,3.2);points.geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));points.material.vertexColors=true;points.userData.statusCenters=true;return points;
}
async function ensureStatusGeometry(d){
 if(d.statusGeometry)return;if(d.statusLoading)return d.statusLoading;
 d.statusLoading=(async()=>{const files=['status-patches.bin','status-patch-faces.bin','status-patch-owners.bin'];const buffers=await Promise.all(files.map(async file=>{const r=await fetch(new URL(`${d.meta.id}/${file}`,BASE),{signal:state.controller.signal});if(!r.ok)throw Error(`${file}: HTTP ${r.status}`);return r.arrayBuffer();}));
  const g={positions:new Float32Array(buffers[0]),indices:new Uint32Array(buffers[1]),owners:new Uint32Array(buffers[2])};if(g.positions.length!==d.meta.status_patches.vertices*3||g.owners.length!==d.meta.status_patches.vertices||g.indices.length!==d.meta.status_patches.triangles*3)throw Error('全部面片状态数据长度不匹配');d.statusGeometry=g;
 })();return d.statusLoading;
}
function maxErrorColor(error,color=new THREE.Color()){
 if(!Number.isFinite(error))return color.set('#8595a1');
 const t=Math.min(1,Math.max(0,error/state.data.meta.max_abs_error.color_scale_max_m));
 return color.setRGB(1,1-t,1-t,THREE.SRGBColorSpace);
}
function errorCM(error){return Number.isFinite(error)?`${error!==0&&Math.abs(error*100)<.001?(error*100).toExponential(2):fmt(error*100,3)} cm`:'无值';}
function evaluate(i,x,y){const s=state.data.seeds,o=i*14,u=x-s[o],v=y-s[o+1];return s[o+2]+s[o+3]+s[o+4]*u+s[o+5]*v+s[o+6]*u*u+s[o+7]*u*v+s[o+8]*v*v;}
function seedNeighbors(i){const ids=state.data.neighbors.subarray(i*30,(i+1)*30);return ids[0]>=0?Array.from(ids):[];}
function pointObject(positions,color,size=1.7,opacity=1){return new THREE.Points(geo(positions),new THREE.PointsMaterial({color,size,sizeAttenuation:false,transparent:opacity<1,opacity,depthWrite:opacity===1}));}
function surface(positions,indices,owners=null){
 const geometry=geo(positions);geometry.setIndex(new THREE.BufferAttribute(indices,1));geometry.computeVertexNormals();
 const mode=$('color-mode').value,color=new THREE.Color(),rgb=mode==='uniform'?null:new Float32Array(positions.length);
 if(rgb)for(let i=0;i<positions.length/3;i++){
  const seed=owners?owners[i]:i;
  if(isStatusMode())color.set(statusBlue(seed)?'#568eb6':'#ffffff');
  else if(mode==='max-error')maxErrorColor(state.data.maxErrors[seed],color);
  else if(mode==='component'){const component=value(seed,11);color.set(component>=0?palette[component%palette.length]:'#8c9da9');}
  else{const t=Math.min(1,Math.max(0,value(seed,10)/.02));color.setHSL(.60-.56*t,.65,.48);}
  color.toArray(rgb,i*3);
 }
 if(rgb)geometry.setAttribute('color',new THREE.BufferAttribute(rgb,3));
 const materialOptions={color:rgb?0xffffff:'#568eb6',vertexColors:!!rgb,side:THREE.DoubleSide,polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1};
 const material=mode==='max-error'||isStatusMode()?new THREE.MeshBasicMaterial(materialOptions):new THREE.MeshStandardMaterial({...materialOptions,roughness:.9});
 const mesh=new THREE.Mesh(geometry,material);mesh.userData.surface=true;
 const result=[mesh];if($('wire').checked)result.push(new THREE.LineSegments(new THREE.WireframeGeometry(geometry),new THREE.LineBasicMaterial({color:'#305a74',transparent:true,opacity:.3})));return result;
}
function hull2d(points){
 const sorted=points.slice().sort((a,b)=>a[0]-b[0]||a[1]-b[1]),unique=sorted.filter((p,i)=>i===0||p[0]!==sorted[i-1][0]||p[1]!==sorted[i-1][1]);
 const cross=(o,a,b)=>(a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0]);if(unique.length<3)return [];
 const lo=[],hi=[];for(const p of unique){while(lo.length>1&&cross(lo.at(-2),lo.at(-1),p)<=0)lo.pop();lo.push(p);}for(const p of unique.slice().reverse()){while(hi.length>1&&cross(hi.at(-2),hi.at(-1),p)<=0)hi.pop();hi.push(p);}return lo.slice(0,-1).concat(hi.slice(0,-1));
}
function singlePatch(seed){
 const ids=seedNeighbors(seed),raw=state.data.points,center=seedPoint(seed);if(!value(seed,12)||ids.length!==30)return {positions:new Float32Array(),indices:new Uint32Array()};
 const local=ids.map(i=>[raw[3*i]-center.x,raw[3*i+1]-center.y]),hull=hull2d(local);if(hull.length<3)return {positions:new Float32Array(),indices:new Uint32Array()};
 const half=[0,1].map(a=>Math.min(.12,Math.max(1e-5,...local.map(p=>Math.abs(p[a])))*.9)),positions=[],inside=[];
 for(let y=0;y<9;y++)for(let x=0;x<9;x++){
  const u=(x/8*2-1)*half[0],v=(y/8*2-1)*half[1],z=evaluate(seed,center.x+u,center.y+v);positions.push(center.x+u,center.y+v,z);
  inside.push(hull.every((a,i)=>{const b=hull[(i+1)%hull.length];return (b[0]-a[0])*(v-a[1])-(b[1]-a[1])*(u-a[0])>=-1e-10;})&&Math.abs(z-center.z)<=.04+1e-10);
 }
 const faces=[];for(let y=0;y<8;y++)for(let x=0;x<8;x++){const a=y*9+x;for(const f of [[a,a+1,a+9],[a+1,a+10,a+9]])if(f.every(i=>inside[i]))faces.push(...f);}
 return {positions:new Float32Array(positions),indices:new Uint32Array(faces),owners:new Uint32Array(81).fill(seed)};
}
function install(){
 if(!state.ready)return;const d=state.data,status=isStatusMode(),only=$('only-current').checked;
 if(status&&!state.statusActive)state.previousSurfaceMode=$('surface-mode').value;
 if(!status&&state.statusActive)$('surface-mode').value=state.previousSurfaceMode||'mesh';state.statusActive=status;
 $('surface-mode').disabled=status;if(status)$('surface-mode').value='patch';const mode=$('surface-mode').value;
 if(status&&!d.statusGeometry){$('loading').hidden=false;$('loading').textContent='正在加载全部独立二次面片，准备拟合／组合状态对照…';ensureStatusGeometry(d).then(()=>{if(d===state.data&&state.ready){$('loading').hidden=true;install();}}).catch(error=>{if(error.name!=='AbortError'&&d===state.data){$('loading').textContent=`加载失败：${error.message}`;console.error(error);}});return;}
 $('loading').hidden=true;
 for(const p of state.panels){
  dispose(p.content);
  if(p.type==='cloud')p.content.add(pointObject(d.points,'#7f939f',1.7));
  else{
   p.scene.background.set($('color-mode').value==='max-error'||status?'#cbd5df':'#eaf0f5');
   if($('raw-overlay').checked)p.content.add(pointObject(d.points,'#8193a0',1.4,.22));
   if(only){const g=singlePatch(state.seed);if(g.indices.length)surface(g.positions,g.indices,g.owners).forEach(o=>p.content.add(o));}
   else if(status){const g=d.statusGeometry;surface(g.positions,g.indices,g.owners).forEach(o=>p.content.add(o));}
   else if(mode==='patch')surface(d.patches,d.patchFaces,d.patchOwners).forEach(o=>p.content.add(o));
   else surface(d.meshPositions,d.mesh).forEach(o=>p.content.add(o));
   if(status)p.content.add(statusCenters(only));
  }
  const size=Math.max(d.meta.bounds[1][0]-d.meta.bounds[0][0],d.meta.bounds[1][1]-d.meta.bounds[0][1]),grid=new THREE.GridHelper(size,Math.min(50,Math.max(8,Math.round(size))),0xbacbd6,0xd7e1e8);grid.rotation.x=Math.PI/2;grid.position.set((d.meta.bounds[0][0]+d.meta.bounds[1][0])/2,(d.meta.bounds[0][1]+d.meta.bounds[1][1])/2,d.meta.bounds[0][2]-.025);p.content.add(grid);
 }
 $('result-title').textContent=only?'当前函数的真实二次曲面':mode==='patch'?'独立局部二次面片（统一采样展示）':'局部二次函数组合曲面';
 $('result-caption').textContent=only?`当前 #${state.seed} 函数；显示范围由其邻点 XY 凸包限制`:`实际有效 ${number(d.meta.valid)} 个函数；${mode==='patch'?`展示其中 ${number(d.meta.independent_patches_shown)} 个独立面片`:`组合得到 ${number(d.meta.triangles)} 个三角形，三角形数量不是函数数量`}。`;
 $('status-legend').hidden=!status;if(status){const blue=$('color-mode').value==='fit-status'?d.meta.valid:d.assembledCount;$('result-title').textContent=only?'当前二次函数的状态':$('color-mode').value==='fit-status'?'拟合成功为蓝，其余为白':'参与最终组合为蓝，其余为白';$('status-counts').textContent=`${number(d.meta.seed_count)} 个中心：蓝色 ${number(blue)} 个，白色 ${number(d.meta.seed_count-blue)} 个。拟合有效但未参与任何最终三角形：${number(d.meta.valid-d.assembledCount)} 个。`;document.querySelectorAll('[data-status-mode]').forEach(b=>b.classList.toggle('active',b.dataset.statusMode===$('color-mode').value));$('result-caption').textContent=`状态判据按局部函数判断。包含全部 ${number(d.meta.valid)} 个有效函数；${number(d.meta.status_patches.functions_with_display_triangles)} 个有可显示面片，其余用状态中心点表示。${only?`当前 #${state.seed}：${statusBlue(state.seed)?'蓝色':'白色'}。`:''}`;}
 if($('color-mode').value==='error')$('result-caption').textContent+=' 残差颜色：蓝色 0 cm → 红色 2 cm。';
 const maximum=$('color-mode').value==='max-error';$('max-error-legend').hidden=!maximum;
 if(maximum){
  $('result-caption').textContent+=' 最大绝对训练残差：白色低、红色高；独立面片每片单色，组合网格按拟合中心的值插值。';
  $('max-error-scale').textContent=errorCM(d.meta.max_abs_error.color_scale_max_m);
  $('max-error-scale-note').textContent=`当前场景统一线性色标，包含全部 ${number(d.meta.solved)} 个已求解函数；灰蓝背景表示没有曲面。全景独立面片仍为采样展示，图表和 CSV 包含全部 ${number(d.meta.seed_count)} 个中心。红色大点是选中的 30 个邻点。`;
 }
 const url=new URL(location.href);url.searchParams.set('color',$('color-mode').value);url.searchParams.set('surface',mode);url.searchParams.set('only',only?'1':'0');url.searchParams.set('seed',state.seed);url.searchParams.set('overlay',$('raw-overlay').checked?'1':'0');url.searchParams.set('neighbors',$('neighbors').checked?'1':'0');history.replaceState(null,'',url);
 updateSeed();plotProfile();draw();
}
function setView(name){
 state.view=name;if(!state.ready)return;const bounds=state.data.meta.bounds,center=new THREE.Vector3(...bounds[0]).add(new THREE.Vector3(...bounds[1])).multiplyScalar(.5),extent=new THREE.Vector3(...bounds[1]).sub(new THREE.Vector3(...bounds[0]));
 const aspect=Math.min(...state.panels.map(p=>p.viewport.clientWidth/p.viewport.clientHeight)),half=THREE.MathUtils.degToRad(camera.fov/2),fov=Math.min(half,Math.atan(Math.tan(half)*aspect));
 const dir=new THREE.Vector3(...(name==='side'?[0,-1,.035]:name==='top'?[0,-.0001,1]:name==='stairs'?[.15,-1,.5]:[.8,-1,.6])).normalize();
 let radius=Math.max(extent.length()/2,.5);target.copy(center);
 if(name==='local'){target.set(0,0,Math.max(.3,Math.min(1,center.z)));radius=3.2;}
 if(name==='stairs'&&state.data.meta.scene_geometry){const g=state.data.meta.scene_geometry;target.set((g.risers[0].x_m+g.risers.at(-1).x_m)/2,0,g.maximum_height_m*.45);radius=2.3;}
 if(name==='seed'){target.copy(seedPoint(state.seed));radius=Math.max(.2,Math.min(.8,value(state.seed,13)||.25));}
 camera.position.copy(target).addScaledVector(dir,radius*1.1/Math.sin(fov));
 state.syncing=true;state.panels.forEach(p=>p.controls.update());state.syncing=false;document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===name));draw();
 const url=new URL(location.href);url.searchParams.set('view',name);history.replaceState(null,'',url);
}
function updateSeed(){
 const i=state.seed,d=state.data,ids=seedNeighbors(i),p=seedPoint(i),valid=!!value(i,12),solved=Number.isFinite(value(i,3));
 $('seed-id').value=i;$('seed-info').textContent=`中心 #${i}，XYZ=(${fmt(p.x,4)}, ${fmt(p.y,4)}, ${fmt(p.z,4)}) m；${valid?'有效函数':solved?'已求解，未通过质量检查':'邻点不足，未求解'}。${solved?`最大绝对训练残差 ${errorCM(d.maxErrors[i])}；训练 RMSE ${errorCM(value(i,10))}。`:''}`;
 $('seed-info').textContent+=` 组合状态：${d.assembled[i]?'已参与最终三角形':valid?'拟合有效，但没有参与最终三角形':'没有有效函数参与组合'}。`;
 if(ids.length){const selected=ids.map(j=>[d.points[3*j],d.points[3*j+1],d.points[3*j+2]]),span=[0,1].map(k=>Math.max(...selected.map(v=>v[k]))-Math.min(...selected.map(v=>v[k])));$('neighbor-info').textContent=`选中 ${ids.length} 点（含中心）；XY 包围框 ${fmt(span[0])} × ${fmt(span[1])} m，最远 XY 半径 ${fmt(value(i,13))} m。`;}
 else $('neighbor-info').textContent='半径与高度约束内不足 30 点，不借用其他高度的点补足。';
 if(solved&&ids.length){let worst=-1,max=-1,prediction=NaN;for(const j of ids){const z=evaluate(i,d.points[3*j],d.points[3*j+1]),e=Math.abs(z-d.points[3*j+2]);if(e>max){max=e;worst=j;prediction=z;}}
  $('neighbor-info').textContent+=` 最大误差对应原始点 #${worst}：观测 Z=${fmt(d.points[3*worst+2],5)} m，预测 Z=${fmt(prediction,5)} m。`;
 }
 $('polynomial').textContent=solved?`z=z_seed+c₀+c₁u+c₂v+c₃u²+c₄uv+c₅v²\nu=x−x_seed，v=y−y_seed\n(c₀…c₅)=(${Array.from(d.seeds.subarray(i*14+3,i*14+9)).map(v=>v.toPrecision(5)).join(', ')})`:'没有足够邻点，没有函数系数；误差留空。';
 for(const panel of state.panels){dispose(panel.highlight);if(!$('neighbors').checked)continue;
  const xyz=new Float32Array(ids.length*3);ids.forEach((j,k)=>xyz.set(d.points.subarray(j*3,j*3+3),k*3));
  const selected=pointObject(xyz,'#d93445',5.5);selected.material.depthTest=false;selected.renderOrder=5;panel.highlight.add(selected);
  const z=valid?value(i,9):p.z,tick=Math.max(.02,Math.min(.08,value(i,13)*.2||.04));const mark=new THREE.LineSegments(geo(new Float32Array([p.x-tick,p.y,z,p.x+tick,p.y,z,p.x,p.y-tick,z,p.x,p.y+tick,z,p.x,p.y,z-tick,p.x,p.y,z+tick])),new THREE.LineBasicMaterial({color:'#ba293c',depthTest:false}));mark.renderOrder=6;panel.highlight.add(mark);
 }plotErrors();draw();
}
function chooseSeed(i){state.seed=Math.max(0,Math.min(state.data.meta.seed_count-1,Math.round(Number(i)||0)));const url=new URL(location.href);url.searchParams.set('seed',state.seed);history.replaceState(null,'',url);if($('only-current').checked){$('slice').value=(value(state.seed,1)-state.data.meta.bounds[0][1])/(state.data.meta.bounds[1][1]-state.data.meta.bounds[0][1])*100;install();}else{updateSeed();plotProfile();}}
function pick(panel,event){
 const rect=panel.renderer.domElement.getBoundingClientRect(),pointer=new THREE.Vector2((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1),ray=new THREE.Raycaster();
 camera.aspect=rect.width/rect.height;camera.updateProjectionMatrix();ray.params.Points.threshold=.07;ray.setFromCamera(pointer,camera);const hit=ray.intersectObjects(panel.content.children,false)[0];if(!hit)return;
 let best=0,min=Infinity;for(let i=0;i<state.data.meta.seed_count;i++){const distance=seedPoint(i).distanceToSquared(hit.point);if(distance<min){min=distance;best=i;}}chooseSeed(best);
}
function plotErrors(){
 if(!state.ready)return;const canvas=$('error-chart'),width=canvas.clientWidth,height=205,scale=Math.min(devicePixelRatio,2);canvas.width=Math.round(width*scale);canvas.height=height*scale;const ctx=canvas.getContext('2d');ctx.scale(scale,scale);ctx.clearRect(0,0,width,height);
 const d=state.data,maximum=$('color-mode').value!=='error',metric=i=>maximum?d.maxErrors[i]:value(i,10),left=42,top=18,w=width-57,h=height-48,n=d.meta.seed_count;let max=maximum?d.meta.max_abs_error.color_scale_max_m:.02;for(let i=0;i<n;i++)if(Number.isFinite(metric(i)))max=Math.max(max,metric(i));max*=1.08;
 const title=maximum?'每个局部函数的最大绝对训练残差':'每个局部函数的训练 RMSE';$('error-chart-title').textContent=title;canvas.setAttribute('aria-label',title);
 const stats=maximum?d.meta.max_abs_error.valid:d.meta.training_residual_valid;
 $('error-summary').textContent=`全部 ${number(n)} 个中心：${number(d.meta.solved)} 个已求解，${number(d.meta.valid)} 个通过、${number(d.meta.rejected)} 个剔除，${number(d.meta.insufficient)} 个未求解。${stats?`有效函数${maximum?'最大绝对残差':' RMSE'}中位数 ${errorCM(stats.median_m)}；最大 ${errorCM(stats.max_m)}。`:''}`;
 $('error-chart-hint').textContent=maximum?'横轴是全部拟合中心编号，纵轴是各函数在 30 个拟合点上的最大绝对残差。白→红表示误差递增，灰框表示未通过检查的已求解函数。点击图表可选函数；未求解的误差留空。':'横轴是全部拟合中心编号，纵轴是训练 RMSE。蓝点通过检查，红点未通过。点击图表可选函数；未求解的误差留空。';
 ctx.fillStyle='#e5ebf0';ctx.fillRect(left,top,w,h);
 ctx.font='10px sans-serif';for(let j=0;j<4;j++){const y=top+j*h/3;ctx.strokeStyle='#e3ebf1';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(left+w,y);ctx.stroke();ctx.fillStyle='#8194a2';ctx.textAlign='right';ctx.fillText((max*(1-j/3)*100).toFixed(1),left-5,y+3);}
 for(let i=0;i<n;i++){const e=metric(i);if(!Number.isFinite(e))continue;const px=left+i/Math.max(1,n-1)*w,py=top+(1-e/max)*h;ctx.fillStyle=maximum?`#${maxErrorColor(e).getHexString()}`:value(i,12)?'#3b80ac':'#d85c55';ctx.fillRect(px,py,1.7,1.7);if(maximum&&!value(i,12)){ctx.strokeStyle='#687a87';ctx.strokeRect(px-.5,py-.5,2.7,2.7);}}
 ctx.strokeStyle='#d69a44';const x=left+state.seed/Math.max(1,n-1)*w;ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,top+h);ctx.stroke();ctx.fillStyle='#8194a2';ctx.textAlign='left';ctx.fillText(maximum?'max |Δz| / cm':'RMSE / cm',2,10);ctx.fillText('0',left,height-9);ctx.textAlign='right';ctx.fillText(`${number(n-1)}  中心编号`,width-5,height-9);canvas.onclick=e=>chooseSeed((e.offsetX-left)/w*(n-1));
}
const NS='http://www.w3.org/2000/svg';function node(tag,attrs={},text){const n=document.createElementNS(NS,tag);for(const [k,v]of Object.entries(attrs))n.setAttribute(k,v);if(text!==undefined)n.textContent=text;return n;}
function plotProfile(){
 if(!state.ready)return;const d=state.data,global=d.meta.bounds,y=global[0][1]+Number($('slice').value)/100*(global[1][1]-global[0][1]),ids=seedNeighbors(state.seed),local=$('only-current').checked&&ids.length===30&&value(state.seed,12);let b=global;
 if(local){const xx=ids.map(i=>d.points[i*3]),zz=ids.map(i=>d.points[i*3+2]),span=Math.max(.02,Math.max(...xx)-Math.min(...xx)),zspan=Math.max(.02,Math.max(...zz)-Math.min(...zz));b=[[Math.min(...xx)-.05*span,global[0][1],Math.min(...zz)-.12*zspan],[Math.max(...xx)+.05*span,global[1][1],Math.max(...zz)+.12*zspan]];}
 $('profile-title').textContent=local?`当前 #${state.seed} 函数的局部剖面 · 橙线为二次函数`:'曲面剖面 · 同一 XY 可显示多个高度';$('slice-value').textContent=`${fmt(y,2)} m`;
 const width=Math.max(280,$('profile').clientWidth),height=260,left=47,top=20,w=width-65,h=height-57,zspan=Math.max(local ? .02 : .3,b[1][2]-b[0][2]),lo=b[0][2]-.05*zspan,hi=b[1][2]+.08*zspan;
 const sx=x=>left+(x-b[0][0])/(b[1][0]-b[0][0])*w,sy=z=>top+(hi-z)/(hi-lo)*h,svg=node('svg',{viewBox:`0 0 ${width} ${height}`});
 const defs=node('defs'),clip=node('clipPath',{id:'profile-clip'});clip.append(node('rect',{x:left,y:top,width:w,height:h}));defs.append(clip);svg.append(defs);
 for(let i=0;i<5;i++){const z=lo+(hi-lo)*i/4;svg.append(node('line',{x1:left,x2:left+w,y1:sy(z),y2:sy(z),stroke:'#e7edf2'}),node('text',{x:left-6,y:sy(z)+3,'text-anchor':'end','font-size':10,fill:'#8194a2'},fmt(z,local?3:2)));const x=b[0][0]+(b[1][0]-b[0][0])*i/4;svg.append(node('text',{x:sx(x),y:height-16,'text-anchor':'middle','font-size':10,fill:'#8194a2'},fmt(x,local?2:1)));}
 const band=Math.min(.1,(b[1][1]-b[0][1])/60);const raw=[];for(let i=0;i<d.points.length;i+=3)if(d.points[i]>=b[0][0]&&d.points[i]<=b[1][0]&&Math.abs(d.points[i+1]-y)<band)raw.push(i);const step=Math.max(1,Math.ceil(raw.length/1800));
 for(let k=0;k<raw.length;k+=step){const i=raw[k];svg.append(node('circle',{cx:sx(d.points[i]),cy:sy(d.points[i+2]),r:1.3,fill:'#9aa8b2',opacity:.5}));}
 const segments=[];
 for(let i=0;i<d.mesh.length;i+=3){const hits=[];for(const [a,c]of [[0,1],[1,2],[2,0]]){const ia=d.mesh[i+a],ic=d.mesh[i+c],ya=value(ia,1),yc=value(ic,1);if(ya===yc)continue;if((y>=Math.min(ya,yc))&&(y<Math.max(ya,yc))){const t=(y-ya)/(yc-ya);hits.push([value(ia,0)+(value(ic,0)-value(ia,0))*t,value(ia,9)+(value(ic,9)-value(ia,9))*t]);}}if(hits.length===2)segments.push(`M${sx(hits[0][0]).toFixed(2)},${sy(hits[0][1]).toFixed(2)}L${sx(hits[1][0]).toFixed(2)},${sy(hits[1][1]).toFixed(2)}`);}
 svg.append(node('path',{d:segments.join(''),stroke:'#337ca7','stroke-width':1.8,fill:'none','clip-path':'url(#profile-clip)'}));
 $('ramp-profile-note').hidden=d.meta.id!=='flat_ramp';if(d.meta.id==='flat_ramp'){const xx=[b[0][0],...[-1,1].filter(x=>x>b[0][0]&&x<b[1][0]),b[1][0]];svg.append(node('polyline',{'data-role':'analytic-reference',points:xx.map(x=>`${sx(x)},${sy(.3*Math.min(2,Math.max(0,x+1)))}`).join(' '),stroke:'#303b43','stroke-width':1.4,'stroke-dasharray':'5 4',fill:'none','clip-path':'url(#profile-clip)'}));}
 const staircase=d.meta.scene_geometry;$('steps-profile-note').hidden=!staircase;if(staircase){const path=[];if(Math.abs(y)<=staircase.stair_width_m/2){for(const part of staircase.profile_segments){const a=Math.max(b[0][0],part.x_min_m),c=Math.min(b[1][0],part.x_max_m);if(c<a)continue;path.push(`${path.length?'L':'M'}${sx(a)},${sy(part.height_m)}L${sx(c)},${sy(part.height_m)}`);}}else path.push(`M${sx(b[0][0])},${sy(0)}L${sx(b[1][0])},${sy(0)}`);svg.append(node('path',{'data-role':'stairs-reference',d:path.join(''),stroke:'#303b43','stroke-width':1.4,'stroke-dasharray':'5 4',fill:'none','clip-path':'url(#profile-clip)'}));}
 if(ids.length&&value(state.seed,12)){
  const xx=ids.map(i=>d.points[i*3]),yy=ids.map(i=>d.points[i*3+1]),hull=hull2d(ids.map(i=>[d.points[i*3],d.points[i*3+1]]));
  if(y>=Math.min(...yy)&&y<=Math.max(...yy)&&hull.length>=3){
   const min=Math.min(...xx),max=Math.max(...xx);let run=[];
   const flush=()=>{if(run.length>1)svg.append(node('polyline',{'data-role':'quadratic-profile',points:run.join(' '),stroke:'#df9b3f','stroke-width':2.2,fill:'none','clip-path':'url(#profile-clip)'}));run=[];};
   for(let i=0;i<81;i++){const x=min+(max-min)*i/80,z=evaluate(state.seed,x,y),inside=hull.every((a,j)=>{const b=hull[(j+1)%hull.length];return (b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0])>=-1e-10;});if(inside&&Math.abs(z-value(state.seed,2))<=.04+1e-10)run.push(`${sx(x)},${sy(z)}`);else flush();}flush();
  }
 }
 svg.append(node('text',{x:3,y:12,'font-size':10,fill:'#8194a2'},'Z / m'),node('text',{x:width-2,y:height-1,'text-anchor':'end','font-size':10,fill:'#8194a2'},'X / m'));$('profile').replaceChildren(svg);
}
async function loadScene(id){
 const requestedParams=new URLSearchParams(location.search),sameScene=requestedParams.get('scene')===id,requestedSeed=sameScene?requestedParams.get('seed'):null,requestedOnly=sameScene&&requestedParams.get('only')==='1';
 const token=++state.loadToken;state.controller?.abort();const controller=new AbortController();state.controller=controller;state.ready=false;$('loading').hidden=false;$('loading').textContent='正在加载完整点云和实际拟合结果…';
 const fetchFile=async name=>{const r=await fetch(new URL(`${id}/${name}`,BASE),{signal:controller.signal});if(!r.ok)throw Error(`${name}: HTTP ${r.status}`);return name.endsWith('.json')?r.json():r.arrayBuffer();};
 try{
  const [meta,points,seeds,neighbors,mesh,patches,patchFaces,patchOwners,maxErrors]=await Promise.all(['data.json','points.bin','seeds.bin','neighbors.bin','mesh.bin','patches.bin','patch-faces.bin','patch-owners.bin','max-error.bin'].map(fetchFile));if(token!==state.loadToken)return;
  const d={meta,points:new Float32Array(points),seeds:new Float64Array(seeds),neighbors:new Int32Array(neighbors),mesh:new Uint32Array(mesh),patches:new Float32Array(patches),patchFaces:new Uint32Array(patchFaces),patchOwners:new Uint32Array(patchOwners),maxErrors:new Float64Array(maxErrors)};
  if(d.points.length!==meta.raw_points*3||d.seeds.length!==meta.seed_count*14||d.neighbors.length!==meta.seed_count*30||d.maxErrors.length!==meta.seed_count)throw Error('场景数据长度不匹配');
  d.meshPositions=new Float32Array(meta.seed_count*3);for(let i=0;i<meta.seed_count;i++)d.meshPositions.set([d.seeds[i*14],d.seeds[i*14+1],Number.isFinite(d.seeds[i*14+9])?d.seeds[i*14+9]:d.seeds[i*14+2]],i*3);
  d.assembled=new Uint8Array(meta.seed_count);for(const i of d.mesh)d.assembled[i]=1;d.assembledCount=d.assembled.reduce((sum,v)=>sum+v,0);
  state.data=d;state.ready=true;state.seed=0;let best=Infinity;
  const assessmentURL=new URL(`../assessment/details.html?scene=${encodeURIComponent(id)}`,BASE);$('assessment-link').href=assessmentURL.href;if($('assessment-inline').open)$('assessment-frame').src=assessmentURL.href;
  for(let i=0;i<meta.seed_count;i++)if(value(i,12)){const score=value(i,0)**2+value(i,1)**2-.02*value(i,2);if(score<best){best=score;state.seed=i;}}
  if(requestedSeed!==null&&Number.isInteger(Number(requestedSeed))&&Number(requestedSeed)>=0&&Number(requestedSeed)<meta.seed_count)state.seed=Number(requestedSeed);
  $('seed-id').max=meta.seed_count-1;$('only-current').checked=requestedOnly;$('scene-select').value=id;$('cloud-caption').textContent=`完整 ${number(meta.raw_points)} 点`;$('ramp-probes').hidden=id!=='flat_ramp';if(requestedOnly)$('slice').value=(value(state.seed,1)-meta.bounds[0][1])/(meta.bounds[1][1]-meta.bounds[0][1])*100;
  $('steps-summary').hidden=!meta.scene_geometry;if(meta.scene_geometry){const g=meta.scene_geometry,up=g.risers.filter(r=>r.signed_height_m>0).map(r=>r.height_cm),down=g.risers.filter(r=>r.signed_height_m<0).map(r=>r.height_cm);$('steps-description').textContent=`场景 ${g.ground_size_m.join(' × ')} m，台阶宽 ${g.stair_width_m} m、最高 ${g.maximum_height_m} m。上升高差依次 ${up.join('、')} cm；下降高差依次 ${down.join('、')} cm。点云包含踏面、立面、侧壁与两侧地面。`;}
  $('metrics').replaceChildren();for(const [v,label]of [[number(meta.raw_points),'原始点 · 完整查询支撑'],[number(meta.seed_count),'拟合中心 · 0.12 m 体素'],[number(meta.solved),'实际求解次数'],[number(meta.valid),'通过检查的函数'],[`${fmt(meta.timings.medians.xyz_to_mesh_ms,1)} ms`,'XYZ → 曲面 · 3 次中位数']]){const item=document.createElement('div');const strong=document.createElement('b');strong.textContent=v;const span=document.createElement('span');span.textContent=label;item.append(strong,span);$('metrics').append(item);}
  const residual=meta.training_residual_valid;$('error-summary').textContent=`${number(meta.solved)} 个已求解函数：${number(meta.valid)} 个通过、${number(meta.rejected)} 个剔除；${number(meta.insufficient)} 个邻点不足。${residual?`有效函数 RMSE 中位数 ${fmt(residual.median_m*100,4)} cm。`:''}`;
  $('probe-select').replaceChildren();for(const c of meta.components.slice(0,16)){const option=document.createElement('option');option.value=c.id;option.textContent=`组件 ${c.id+1} · Z中位数 ${fmt(c.median_z,2)} m`;$('probe-select').append(option);}
  for(const [dom,file]of [['csv-link','per-fit.csv'],['npz-link','results.npz'],['audit-link','audit.json'],['max-error-csv-link','max-error.csv'],['max-error-audit-link','max-error-audit.json']])$(dom).href=new URL(`${id}/${file}`,BASE);
  const rawLinks={table:'large-table',combined:'combined-scene','corridor-040':'corridor-040','corridor-060':'corridor-060','corridor-080':'corridor-080'};$('raw-scene-link').hidden=!rawLinks[id];if(rawLinks[id])$('raw-scene-link').href=new URL(`../${rawLinks[id]}/`,BASE);
  document.querySelectorAll('#scene-table tr').forEach(row=>row.classList.toggle('active',row.dataset.scene===id));
  const url=new URL(location.href);url.searchParams.set('scene',id);history.replaceState(null,'',url);
  const requestedView=new URLSearchParams(location.search).get('view');install();setView(['overall','local','side','top','seed','stairs'].includes(requestedView)?requestedView:meta.scene_geometry?'stairs':['table','combined','corridor-040','corridor-060','corridor-080'].includes(id)?'local':'overall');$('loading').hidden=isStatusMode()?!d.statusLoading||!!d.statusGeometry:true;
 }catch(error){if(error.name==='AbortError')return;$('loading').textContent=`加载失败：${error.message}`;console.error(error);}
}
$('scene-select').addEventListener('change',()=>loadScene($('scene-select').value));
$('assessment-inline').addEventListener('toggle',()=>{if($('assessment-inline').open&&$('assessment-frame').src!==$('assessment-link').href)$('assessment-frame').src=$('assessment-link').href;});
for(const id of ['surface-mode','color-mode','raw-overlay','wire','only-current'])$(id).addEventListener('change',install);
document.querySelectorAll('[data-status-mode]').forEach(button=>button.addEventListener('click',()=>{$('color-mode').value=button.dataset.statusMode;install();}));$('show-final-mesh').addEventListener('click',()=>{state.previousSurfaceMode='mesh';$('color-mode').value='uniform';$('surface-mode').value='mesh';install();});
$('neighbors').addEventListener('change',()=>{const url=new URL(location.href);url.searchParams.set('neighbors',$('neighbors').checked?'1':'0');history.replaceState(null,'',url);if(state.ready)updateSeed();});$('seed-id').addEventListener('change',()=>chooseSeed($('seed-id').value));
$('largest-error').addEventListener('click',()=>{if(!state.ready)return;let seed=-1,error=-1;for(let i=0;i<state.data.meta.seed_count;i++){const e=$('color-mode').value==='error'?value(i,10):state.data.maxErrors[i];if(Number.isFinite(e)&&e>error){seed=i;error=e;}}if(seed>=0){chooseSeed(seed);setView('seed');}});
$('steps-overview').addEventListener('click',()=>setView('stairs'));$('steps-side').addEventListener('click',()=>setView('side'));
$('probe-select').addEventListener('change',()=>{const component=Number($('probe-select').value);let best=-1,score=Infinity;for(let i=0;i<state.data.meta.seed_count;i++)if(value(i,11)===component){const d=value(i,0)**2+value(i,1)**2;if(d<score){score=d;best=i;}}if(best>=0)chooseSeed(best);});
const focus=()=>{if(!state.ready)return;$('only-current').checked=true;$('slice').value=(value(state.seed,1)-state.data.meta.bounds[0][1])/(state.data.meta.bounds[1][1]-state.data.meta.bounds[0][1])*100;install();setView('seed');};$('focus-seed').addEventListener('click',focus);
document.querySelectorAll('[data-ramp-x]').forEach(button=>button.addEventListener('click',()=>{if(!state.ready)return;let best=-1,distance=Infinity;const x=Number(button.dataset.rampX);for(let i=0;i<state.data.meta.seed_count;i++)if(value(i,12)){const d=(value(i,0)-x)**2+value(i,1)**2;if(d<distance){distance=d;best=i;}}if(best>=0){$('surface-mode').value='patch';$('neighbors').checked=true;chooseSeed(best);focus();}}));
document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>b.dataset.view==='seed'?focus():setView(b.dataset.view)));
$('slice').addEventListener('input',plotProfile);new ResizeObserver(()=>{if(state.ready){plotErrors();plotProfile();}}).observe($('error-chart-holder'));
async function init(){
 const params=new URLSearchParams(location.search);for(const [key,id]of [['color','color-mode'],['surface','surface-mode']]){const requested=params.get(key);if(Array.from($(id).options).some(o=>o.value===requested))$(id).value=requested;}if(params.get('overlay')==='0')$('raw-overlay').checked=false;if(params.get('neighbors')==='0')$('neighbors').checked=false;
 const response=await fetch(new URL('catalog.json',BASE));if(!response.ok)throw Error('无法读取场景目录');state.catalog=await response.json();
 for(const c of state.catalog.scenes){const option=document.createElement('option');option.value=c.id;option.textContent=c.name;$('scene-select').append(option);const row=document.createElement('tr');row.dataset.scene=c.id;const residual=c.training_residual_valid;for(const v of [c.name,number(c.raw_points),number(c.seed_count),number(c.solved),number(c.valid),residual?fmt(residual.median_m*100,4):'无值',fmt(c.timings.medians.xyz_to_mesh_ms,1)]){const td=document.createElement('td');td.textContent=v;row.append(td);}const td=document.createElement('td'),button=document.createElement('button');button.textContent='查看';button.addEventListener('click',()=>{loadScene(c.id);$('scene-select').scrollIntoView({behavior:'smooth'});});td.append(button);row.append(td);$('scene-table').append(row);}
 $('scene-count').textContent=`${state.catalog.scenes.length} 个场景 · 1 个拼接大场景，3 个独立通道 · 端口 42482`;
 const requested=new URLSearchParams(location.search).get('scene');await loadScene(state.catalog.scenes.some(c=>c.id===requested)?requested:state.catalog.default_scene);
}
window.sceneFitCatalog={state,camera,target,draw,setView,loadScene,chooseSeed,singlePatch,maxErrorColor};init().catch(error=>{$('loading').textContent=error.message;console.error(error);});
