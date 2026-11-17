// Open the actual HTTP viewer, verify all points render and exercise controls.
const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const assert=require('assert/strict'),fs=require('fs'),path=require('path');
const output=path.resolve(__dirname,'../outputs/visualization/combined-scene');
const base='http://127.0.0.1:42482/combined-scene/';
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1460,height:1100}}),errors=[],failed=[];
  page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push({url:r.url(),status:r.status()});});
  assert.equal((await page.goto(base,{waitUntil:'networkidle'})).status(),200);
  await page.waitForFunction(()=>window.combinedScene?.state.ready);
  const rendered=await page.evaluate(()=>{
   const s=window.combinedScene;s.draw();
   return {loaded:s.state.meta.point_count,rendered:s.renderer.info.render.points,layers:s.groups.map(g=>g.children[0].geometry.attributes.position.count)};
  });
  assert.equal(rendered.loaded,439501);assert.equal(rendered.rendered,rendered.loaded);
  assert.equal(await page.locator('#layers input').count(),7);
  await page.screenshot({path:path.join(output,'browser-overall.png'),fullPage:true});
  for(const view of ['hill','forest','table','under','top']){
   await page.click(`[data-view="${view}"]`);
   assert.equal(await page.evaluate(()=>window.combinedScene.state.view),view);
   if(view==='table'||view==='under')await page.locator('.viewer-shell').screenshot({path:path.join(output,`browser-${view}.png`)});
  }
  await page.uncheck('#layer-1');
  assert.equal(await page.evaluate(()=>window.combinedScene.renderer.info.render.points),439501-141068);
  await page.check('#layer-1');
  await page.selectOption('#color-mode','height');
  assert(await page.locator('#height-key').isVisible());
  const colors=await page.evaluate(()=>{
   const rgb=window.combinedScene.groups[0].children[0].geometry.attributes.color.array,set=new Set();
   for(let i=0;i<rgb.length;i+=30)set.add([rgb[i],rgb[i+1],rgb[i+2]].map(v=>v.toFixed(3)).join(','));
   return set.size;
  });assert(colors>100);
  await page.check('#isolate-table');await page.click('[data-view="table"]');
  assert.equal(await page.evaluate(()=>window.combinedScene.renderer.info.render.points),45409+8310+1614);
  await page.uncheck('#layer-4');assert.equal(await page.evaluate(()=>window.combinedScene.renderer.info.render.points),45409+1614);
  await page.check('#layer-4');await page.uncheck('#isolate-table');
  await page.selectOption('#color-mode','source');await page.click('[data-view="overall"]');
  for(const asset of ['scene.json','points.bin','combined_scene.pcd','combined_scene.npz','overview.png','README.md'])assert.equal((await page.request.head(base+asset)).status(),200);
  await page.setViewportSize({width:390,height:844});
  await page.waitForFunction(()=>window.combinedScene.renderer.domElement.width===document.querySelector('#viewport').clientWidth);
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
  await page.screenshot({path:path.join(output,'browser-mobile.png'),fullPage:true});
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  const result={passed:true,...rendered,allViews:true,layerToggle:true,heightColorDistinctSamples:colors,tableIsolation:true,allDownloads200:true,mobileOverflow:false,errors,failed};
  fs.writeFileSync(path.join(output,'browser-audit.json'),JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
