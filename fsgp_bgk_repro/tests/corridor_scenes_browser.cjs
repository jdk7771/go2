const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const assert=require('assert/strict'),fs=require('fs'),path=require('path');
const web=path.resolve(__dirname,'../outputs/visualization');
const variants=[['040',.4,30761],['060',.6,32841],['080',.8,34921]];
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1460,height:1100}}),errors=[],failed=[],results=[];
  page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push({url:r.url(),status:r.status()});});
  for(const [id,height,count]of variants){
   const base=`http://127.0.0.1:42482/corridor-${id}/`,output=path.join(web,`corridor-${id}`);
   assert.equal((await page.goto(base,{waitUntil:'networkidle'})).status(),200);
   await page.waitForFunction(()=>window.corridorScene?.state.ready);
   const loaded=await page.evaluate(()=>{const s=window.corridorScene;s.draw();return {loaded:s.state.meta.point_count,rendered:s.renderer.info.render.points,clearance:s.state.meta.corridor.clear_height_m,scope:s.state.meta.scope,nominalSize:s.state.meta.nominal_size_m};});
   assert.equal(loaded.loaded,count);assert.equal(loaded.rendered,count);assert.equal(loaded.clearance,height);
   assert.equal(loaded.scope,'standalone');assert.deepEqual(loaded.nominalSize,[6,4]);
   assert.equal(await page.locator('.scene-tabs a[aria-current="page"]').getAttribute('data-clearance'),String(height));
   assert.equal(await page.locator('#layers input').count(),3);
   await page.screenshot({path:path.join(output,'browser-local.png'),fullPage:true});
   const views={};
   for(const view of ['entrance','inside','overall','top']){
    await page.click(`[data-view="${view}"]`);assert.equal(await page.evaluate(()=>window.corridorScene.state.view),view);
    views[view]=await page.evaluate(()=>({camera:window.corridorScene.camera.position.toArray(),target:window.corridorScene.controls.target.toArray()}));
    if(view==='inside')assert(Math.abs(views[view].camera[2]-height/2)<1e-8);
    if(view==='entrance'||view==='overall')await page.locator('.viewer-shell').screenshot({path:path.join(output,`browser-${view}.png`)});
   }
   const roof=await page.evaluate(()=>window.corridorScene.state.meta.layers[1].count);
   await page.uncheck('#layer-1');assert.equal(await page.evaluate(()=>window.corridorScene.renderer.info.render.points),count-roof);
   await page.check('#layer-1');
   await page.selectOption('#color-mode','height');assert(await page.locator('#height-key').isVisible());
   await page.selectOption('#color-mode','source');await page.click('[data-view="local"]');
   for(const asset of ['scene.json','points.bin','corridor_scene.pcd','corridor_scene.npz','audit.json','README.md','clearance-comparison.png'])assert.equal((await page.request.head(base+asset)).status(),200);
   // Load the actual comparison image and verify the variant links resolve.
   await page.locator('.comparison img').scrollIntoViewIfNeeded();await page.waitForFunction(()=>document.querySelector('.comparison img').complete&&document.querySelector('.comparison img').naturalWidth>0);
   for(const [other]of variants)assert.equal((await page.request.head(`http://127.0.0.1:42482/corridor-${other}/`)).status(),200);
   results.push({id,passed:true,...loaded,allViews:true,views,layerToggle:true,allDownloads200:true});
  }
  assert.deepEqual(results.map(r=>r.views.entrance),Array(3).fill(results[0].views.entrance));
  await page.setViewportSize({width:390,height:844});await page.waitForFunction(()=>window.corridorScene.renderer.domElement.width===document.querySelector('#viewport').clientWidth);
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
  await page.screenshot({path:path.join(web,'corridor-080/browser-mobile.png'),fullPage:true});
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  for(const r of results)fs.writeFileSync(path.join(web,`corridor-${r.id}/browser-audit.json`),JSON.stringify({...r,sameEntranceCameraAcrossAllVariants:true,mobileOverflow:false,errors,failed},null,2)+'\n');
  console.log(JSON.stringify({passed:true,scenes:results.map(({views,...r})=>r),sameEntranceCamera:true,mobileOverflow:false,errors,failed}));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
