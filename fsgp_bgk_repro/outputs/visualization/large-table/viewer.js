import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

const viewport = document.querySelector('#viewport');
const status = document.querySelector('#status');
const label = document.querySelector('#view-label');
const colors = [0xc4cdbb, 0xbc8651, 0x356d60, 0x356d60, 0x356d60, 0x356d60];
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xedf1e9);
const camera = new THREE.PerspectiveCamera(40, 1, 0.025, 100);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({antialias:true, preserveDrawingBuffer:true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewport.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.minDistance = 0.25;
controls.maxDistance = 18;
// A small positive camera elevation allows a view into the clear space while
// preventing the orbit from showing the underside of the ground plane.
controls.maxPolarAngle = Math.PI / 2 - 0.008;
scene.add(new THREE.HemisphereLight(0xffffff, 0x899d82, 2.3));
const sunlight = new THREE.DirectionalLight(0xfff6e1, 2.5);
sunlight.position.set(-2, -3, 7);
scene.add(sunlight);
const fill = new THREE.DirectionalLight(0xe6eeff, 1);
fill.position.set(3, 4, 2);
scene.add(fill);
const groups = {floor:new THREE.Group(), tabletop:new THREE.Group(), legs:new THREE.Group()};
Object.values(groups).forEach(group => scene.add(group));
const guides = new THREE.Group();
scene.add(guides);

function line(points, color=0x879582, group=guides) {
  const geometry = new THREE.BufferGeometry().setFromPoints(points.map(p => new THREE.Vector3(...p)));
  const result = new THREE.Line(geometry, new THREE.LineBasicMaterial({color, transparent:true, opacity:.8}));
  group.add(result);
}

function textSprite(text, position) {
  const canvas = document.createElement('canvas');
  canvas.width = 512; canvas.height = 96;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = '#ffffffee';
  ctx.beginPath();ctx.roundRect(4, 4, 504, 88, 15);ctx.fill();
  ctx.font = '500 39px "Noto Sans CJK SC", sans-serif';
  ctx.fillStyle = '#3d5b4c';ctx.textAlign = 'center';ctx.textBaseline = 'middle';
  ctx.fillText(text, 256, 49);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({map:texture, depthTest:false, transparent:true}));
  sprite.position.set(...position);sprite.scale.set(1.23, .23, 1);sprite.renderOrder = 10;
  guides.add(sprite);
}

function dimension(a, b, text, labelPosition) {
  line([a, b]);
  const direction = new THREE.Vector3(...b).sub(new THREE.Vector3(...a)).normalize();
  const vertical = Math.abs(direction.z) > .5;
  const tick = vertical ? [0, .065, 0] : [0, 0, .065];
  for(const p of [a, b]) line([p.map((v,i) => v-tick[i]), p.map((v,i) => v+tick[i])]);
  textSprite(text, labelPosition);
}

const presets = {
  overall:{position:[5.2,-6.4,4.0], target:[0,0,.52], name:'整体视角'},
  under:{position:[3.5,-4.6,.52], target:[0,0,.48], name:'桌底视角 · 桌下净空 1.10 m'},
  side:{position:[0,-7.3,1.0], target:[0,0,.6], name:'侧面视角'},
  top:{position:[0,-.01,8.5], target:[0,0,0], name:'俯视视角'},
};
function setView(name) {
  const p = presets[name];
  camera.position.set(...p.position);controls.target.set(...p.target);controls.update();
  label.textContent = p.name;
  document.querySelectorAll('[data-view]').forEach(b => b.classList.toggle('active', b.dataset.view===name));
}
document.querySelectorAll('[data-view]').forEach(b => b.addEventListener('click', () => setView(b.dataset.view)));
for(const [id, group] of Object.entries({...groups, guides})) {
  document.getElementById(id).addEventListener('change', event => {group.visible = event.target.checked;});
}
function setMode() {
  const mode = document.querySelector('#mode').value;
  Object.values(groups).forEach(group => group.traverse(object => {
    if(object.userData.surface) object.visible = mode !== 'points';
    if(object.isPoints) object.visible = mode !== 'surface-only';
  }));
}
document.querySelector('#mode').addEventListener('change', setMode);

function surface(box) {
  const lo=box.min, hi=box.max;
  const group=groups[box.label===0?'floor':box.label===1?'tabletop':'legs'];
  const geometry = box.label===0 ? new THREE.PlaneGeometry(hi[0]-lo[0], hi[1]-lo[1])
    : new THREE.BoxGeometry(...hi.map((v,i) => v-lo[i]));
  const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({
    color:colors[box.label], roughness:.87, metalness:0, side:THREE.DoubleSide,
    polygonOffset:true, polygonOffsetFactor:1, polygonOffsetUnits:1,
  }));
  mesh.position.set(...hi.map((v,i) => (v+lo[i])/2));mesh.userData.surface=true;group.add(mesh);
  if(box.label!==0) {
    const edge=new THREE.LineSegments(new THREE.EdgesGeometry(geometry),
      new THREE.LineBasicMaterial({color:box.label===1?0x8b623d:0x284f47}));
    edge.position.copy(mesh.position);edge.userData.surface=true;group.add(edge);
  }
}

async function load() {
  const [metaResponse, pointsResponse] = await Promise.all([fetch('./scene.json'),fetch('./points.bin')]);
  if(!metaResponse.ok || !pointsResponse.ok) throw new Error('场景文件读取失败');
  const meta = await metaResponse.json();
  const values = new Float32Array(await pointsResponse.arrayBuffer());
  if(values.length !== meta.point_count*4) throw new Error('点云数据长度与记录不一致');
  [meta.floor,meta.tabletop,...meta.legs].forEach(surface);
  const positions=Array.from({length:6},()=>[]);
  for(let i=0;i<values.length;i+=4) positions[values[i+3]].push(values[i],values[i+1],values[i+2]);
  positions.forEach((xyz,id) => {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.Float32BufferAttribute(xyz,3));
    const points=new THREE.Points(geometry,new THREE.PointsMaterial({
      color:id===0?0x95a88b:id===1?0x855024:0x163f36,size:.014,sizeAttenuation:true,
    }));
    groups[id===0?'floor':id===1?'tabletop':'legs'].add(points);
  });
  const grid = new THREE.GridHelper(6,12,0xa4b199,0xb7c3ad);
  grid.rotation.x = Math.PI/2;grid.position.z=.001;grid.userData.surface=true;groups.floor.add(grid);
  dimension([-1.5,-1.25,1.2],[1.5,-1.25,1.2],'长 3.00 m',[0,-1.38,1.45]);
  dimension([1.76,-1,1.2],[1.76,1,1.2],'宽 2.00 m',[2.05,0,1.42]);
  dimension([-1.87,-1.0,0],[-1.87,-1.0,1.1],'净空 1.10 m',[-2.1,-1.0,.6]);
  const origin = [-2.55,-2.55,.015];
  line([origin,[-2.05,-2.55,.015]],0xad594d);
  line([origin,[-2.55,-2.05,.015]],0x52825f);
  line([origin,[-2.55,-2.55,.515]],0x4c73a8);
  document.querySelector('#point-count').textContent = `${meta.point_count.toLocaleString('zh-CN')} 点 · 间距 ≤ ${(meta.max_surface_spacing_m*100).toFixed(1)} cm`;
  status.hidden = true;
  window.tableScene = {ready:true, metadata:meta, scene, camera, controls, renderer,
    groups, guides, setView, setMode};
}
function resize() {
  const {width,height}=viewport.getBoundingClientRect();
  camera.aspect=width/height;camera.updateProjectionMatrix();renderer.setSize(width,height);
}
new ResizeObserver(resize).observe(viewport);resize();setView('overall');
renderer.setAnimationLoop(() => {controls.update();renderer.render(scene,camera);});
load().catch(error => {status.hidden=false;status.textContent=`加载失败：${error.message}。请通过本地 HTTP 地址打开。`;console.error(error);});
