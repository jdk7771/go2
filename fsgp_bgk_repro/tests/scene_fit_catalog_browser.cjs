const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const assert=require('assert/strict'),fs=require('fs'),path=require('path');
const output=path.resolve(__dirname,'../outputs/visualization/scene-fits');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1480,height:1100}}),errors=[],failed=[],results=[];
  page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push({url:r.url(),status:r.status()});});
  assert.equal((await page.goto('http://127.0.0.1:42482/scene-fits/',{waitUntil:'networkidle',timeout:60000})).status(),200);
  await page.waitForFunction(()=>window.sceneFitCatalog?.state.ready,{timeout:60000});
  const sceneCount=await page.evaluate(()=>window.sceneFitCatalog.state.catalog.scenes.length);
  assert.equal(await page.locator('#scene-select option').count(),sceneCount);assert.equal(await page.locator('#scene-table tr').count(),sceneCount);
  await page.screenshot({path:path.join(output,'browser-home.png'),fullPage:true});
  const ids=await page.locator('#scene-select option').evaluateAll(options=>options.map(o=>o.value));
  for(const id of ids){
   await page.selectOption('#scene-select',id);
   await page.waitForFunction(id=>window.sceneFitCatalog.state.ready&&window.sceneFitCatalog.state.data.meta.id===id,id,{timeout:60000});
   const rendered=await page.evaluate(()=>{
    const s=window.sceneFitCatalog.state,d=s.data,cloud=s.panels.find(p=>p.type==='cloud').content.children.find(o=>o.isPoints),mesh=s.panels.find(p=>p.type==='fit').content.children.find(o=>o.userData.surface);
    return {id:d.meta.id,raw:d.meta.raw_points,displayedRaw:cloud.geometry.attributes.position.count,valid:d.meta.valid,triangles:d.meta.triangles,displayedTriangles:mesh.geometry.index.count/3,solved:d.meta.solved};
   });assert.equal(rendered.displayedRaw,rendered.raw);assert.equal(rendered.displayedTriangles,rendered.triangles);assert(rendered.valid>0);results.push(rendered);
   assert.equal(await page.locator('#profile svg').count(),1);
   if(id.startsWith('corridor-')||id==='table'||id==='forest')await page.locator('.workspace').screenshot({path:path.join(output,`browser-${id}.png`)});
   console.log(JSON.stringify({checked:id,raw:rendered.raw,valid:rendered.valid}));
  }
  await page.selectOption('#scene-select','corridor-040');await page.waitForFunction(()=>window.sceneFitCatalog.state.ready&&window.sceneFitCatalog.state.data.meta.id==='corridor-040');
  await page.selectOption('#surface-mode','patch');
  assert.equal(await page.evaluate(()=>window.sceneFitCatalog.state.panels[1].content.children.find(o=>o.userData.surface).geometry.index.count/3),await page.evaluate(()=>window.sceneFitCatalog.state.data.patchFaces.length/3));
  await page.selectOption('#color-mode','error');assert(await page.evaluate(()=>window.sceneFitCatalog.state.panels[1].content.children.find(o=>o.userData.surface).material.vertexColors));
  await page.click('#focus-seed');assert(await page.isChecked('#only-current'));
  assert((await page.evaluate(()=>window.sceneFitCatalog.state.panels[1].content.children.find(o=>o.userData.surface).geometry.index.count))>0);
  await page.locator('.workspace').screenshot({path:path.join(output,'browser-selected-function.png')});
  assert.match(await page.locator('#neighbor-info').innerText(),/选中 30 点/);
  assert.match(await page.locator('#polynomial').innerText(),/c₀/);
  const missing=await page.evaluate(()=>{const d=window.sceneFitCatalog.state.data;for(let i=0;i<d.meta.seed_count;i++)if(!Number.isFinite(d.seeds[i*14+3]))return i;return -1;});
  if(missing>=0){await page.fill('#seed-id',String(missing));await page.locator('#seed-id').dispatchEvent('change');assert.match(await page.locator('#seed-info').innerText(),/未求解/);assert.match(await page.locator('#polynomial').innerText(),/误差留空/);}
  await page.uncheck('#only-current');await page.selectOption('#surface-mode','mesh');
  for(const asset of ['catalog.json','summary.csv','corridor-040/per-fit.csv','corridor-060/results.npz','corridor-080/audit.json'])assert.equal((await page.request.head('http://127.0.0.1:42482/scene-fits/'+asset)).status(),200);
  for(const old of ['legacy-results.html','table-local/','table-layers/','fsgp-fit/','quadric-fit/','quadric-lidar/','corridor-040/','corridor-060/','corridor-080/','combined-scene/'])assert.equal((await page.request.head('http://127.0.0.1:42482/'+old)).status(),200);
  await page.setViewportSize({width:390,height:844});
  await page.waitForFunction(()=>window.sceneFitCatalog.state.panels.every(p=>p.renderer.domElement.width===p.viewport.clientWidth));
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);await page.screenshot({path:path.join(output,'browser-mobile.png'),fullPage:true});
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  const result={passed:true,rootPort:42482,sceneCount,scenes:results,independentPatches:true,selectedFunction:true,exactNeighborsDisplayed:30,rmseColors:true,unknownNotZero:true,legacyLinks:true,downloads:true,mobileOverflow:false,errors,failed};fs.writeFileSync(path.join(output,'browser-audit.json'),JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
