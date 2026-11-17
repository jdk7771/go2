const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const assert=require('assert/strict'),fs=require('fs'),path=require('path');
const output=path.resolve(__dirname,'../outputs/visualization/mixed-steps');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1480,height:1100}}),errors=[],failed=[];
  page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push({url:r.url(),status:r.status()});});
  await page.goto('http://127.0.0.1:42482/?scene=steps&color=uniform&surface=mesh&view=local&only=0&seed=2358&overlay=1&neighbors=1',{waitUntil:'networkidle'});
  await page.waitForFunction(()=>window.sceneFitCatalog?.state.ready&&window.sceneFitCatalog.state.data.meta.id==='steps');
  assert(await page.isVisible('#steps-summary'));assert.match(await page.innerText('#steps-description'),/8、18、12、30、10、22/);
  assert.match(await page.innerText('#steps-description'),/25、10、40、25/);
  const meta=await page.evaluate(()=>{
   const d=window.sceneFitCatalog.state.data,s=window.sceneFitCatalog.state,p=s.panels.find(p=>p.type==='cloud').content.children.find(o=>o.isPoints),m=s.panels.find(p=>p.type==='fit').content.children.find(o=>o.userData.surface);
   return {scene:d.meta.id,raw:d.meta.raw_points,displayed:p.geometry.attributes.position.count,triangles:d.meta.triangles,renderedTriangles:m.geometry.index.count/3,geometry:d.meta.scene_geometry,centers:d.meta.seed_count,solved:d.meta.solved,valid:d.meta.valid};
  });assert.equal(meta.raw,39417);assert.equal(meta.raw,meta.displayed);assert.equal(meta.triangles,meta.renderedTriangles);assert.equal(meta.geometry.riser_count,10);assert.equal(meta.geometry.maximum_height_m,1);assert.equal(meta.geometry.ascent_count,6);assert.equal(meta.geometry.descent_count,4);
  await page.click('#steps-overview');assert.equal(await page.evaluate(()=>window.sceneFitCatalog.state.view),'stairs');await page.locator('.workspace').screenshot({path:path.join(output,'overview.png')});await page.locator('.profile-card').screenshot({path:path.join(output,'profile.png')});assert.equal(await page.locator('[data-role="stairs-reference"]').count(),1);
  await page.click('#steps-side');assert.equal(await page.evaluate(()=>window.sceneFitCatalog.state.view),'side');await page.locator('.workspace').screenshot({path:path.join(output,'side.png')});
  await page.selectOption('#color-mode','max-error');assert(await page.isVisible('#max-error-legend'));await page.click('#steps-overview');await page.locator('.workspace').screenshot({path:path.join(output,'max-error.png')});
  const csv=await page.request.get(await page.getAttribute('#max-error-csv-link','href'));assert.equal(csv.status(),200);assert.equal((await csv.text()).trim().split(/\r?\n/).length-1,meta.centers);
  await page.selectOption('#surface-mode','patch');assert((await page.evaluate(()=>window.sceneFitCatalog.state.panels[1].content.children.find(o=>o.userData.surface).geometry.index.count))>0);
  await page.evaluate(()=>{const a=window.sceneFitCatalog,d=a.state.data;let best=0,score=Infinity;for(let i=0;i<d.meta.seed_count;i++)if(d.seeds[i*14+12]){const q=d.seeds[i*14]**2+d.seeds[i*14+1]**2+(d.seeds[i*14+2]-1)**2;if(q<score){score=q;best=i;}}a.chooseSeed(best);});
  await page.click('#focus-seed');assert(await page.isChecked('#only-current'));assert.equal(await page.evaluate(()=>window.sceneFitCatalog.state.panels[1].content.children.find(o=>o.userData.surface).geometry.attributes.position.count),81);assert.match(await page.innerText('#neighbor-info'),/选中 30 点/);
  for(const asset of ['mixed-steps/mixed_steps.pcd','mixed-steps/mixed_steps.npz','scene-fits/steps-single/results.npz'])assert.equal((await page.request.head('http://127.0.0.1:42482/'+asset)).status(),200);
  await page.selectOption('#scene-select','steps-single');await page.waitForFunction(()=>window.sceneFitCatalog.state.ready&&window.sceneFitCatalog.state.data.meta.id==='steps-single');assert.equal(await page.evaluate(()=>window.sceneFitCatalog.state.data.meta.raw_points),10201);assert(!(await page.isVisible('#steps-summary')));
  await page.selectOption('#scene-select','steps');await page.waitForFunction(()=>window.sceneFitCatalog.state.ready&&window.sceneFitCatalog.state.data.meta.id==='steps');await page.setViewportSize({width:390,height:844});await page.waitForFunction(()=>window.sceneFitCatalog.state.panels.every(p=>p.renderer.domElement.width===p.viewport.clientWidth));assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);await page.screenshot({path:path.join(output,'mobile.png'),fullPage:true});
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);const result={passed:true,port:42482,meta,profile:true,sideView:true,allStepsView:true,maxErrors:true,independentPatches:true,selectedPolynomial:true,downloads:true,legacySingleStep:true,mobileOverflow:false,errors,failed};fs.writeFileSync(path.join(output,'browser-audit.json'),JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
