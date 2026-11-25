const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const base='http://127.0.0.1:42482/',out=path.resolve(__dirname,'../outputs/visualization/convex-assessment');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 let page=await browser.newPage({viewport:{width:1440,height:1100}});const errors=[],failed=[],records=[],requests=[];
 const watch=()=>{page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push([r.status(),r.url()]);});page.on('request',r=>requests.push(r.url()));};watch();
 const wait=(scene,profile)=>page.waitForFunction(({scene,profile})=>{const s=window.metricViewer?.state;return s?.ready&&s.renderedScene===scene&&s.renderedProfile===profile&&document.getElementById('loading').hidden;},{scene,profile},{timeout:90000});
 try{
  await page.goto(base,{waitUntil:'networkidle'});await wait('hill','qcircle-adaptive');
  const scenes=await page.locator('#scene-select option').evaluateAll(o=>o.map(x=>x.value));assert.deepEqual(scenes,['hill','forest','flat','desert','mountain','snow','vocano','steps','flat-corridor-platforms','surface-route-test','table','table-low-040','table-low-042','table-low-060','layer-choice','platform-underpass-070','platform-underpass-042','four-platforms']);
  const profiles=await page.locator('#profile-select option').evaluateAll(o=>o.map(x=>x.value));assert.equal(profiles.length,4);
  const fields=await page.locator('#metric-select option').evaluateAll(o=>o.map(x=>x.value));assert.deepEqual(fields,['clearance_cost','slope_deg','rmse_m','std_patch_m','normal_spread_deg','max_error_m']);
  assert.equal(await page.locator('#mesh-option,#mesh-mode,#method-select,#hide-overhead,#plan-controls,#plan-option').count(),0);assert.equal(await page.locator('canvas').count(),1);
  assert(await page.isChecked('#fit-coverage'));
  for(const profile of profiles){
   // Release the preceding setting's large snow-scene WebGL resources.
   // Scene and setting switches are also covered by the checks below.
   if(profile!==profiles[0]){await page.close();page=await browser.newPage({viewport:{width:1440,height:1100}});watch();await page.goto(base,{waitUntil:'networkidle'});await wait('hill','qcircle-adaptive');}
   const current=await page.locator('#scene-select').inputValue();await page.selectOption('#profile-select',profile);await wait(current,profile);
   for(const scene of scenes){
    if(scene!==scenes[0]){await page.close();page=await browser.newPage({viewport:{width:1440,height:1100}});watch();await page.goto(base+`?scene=${scene}&profile=${profile}`,{waitUntil:'networkidle'});}
    await page.selectOption('#scene-select',scene);await wait(scene,profile);
    const record=await page.evaluate(()=>{const a=window.metricViewer,d=a.state.data,g=a.content.children.find(o=>o.userData.surface);if(!g||g.geometry.index.count!==3*d.meta.patch_triangles)throw Error('Incorrect display geometry');if(d.meta.valid!==d.meta.displayed_functions||d.patchCounts.filter(n=>n>0).length!==d.meta.valid)throw Error('An entire valid hull is missing');if(d.meta.geometry.observation_face_deletion||d.meta.geometry.cross_function_assembly)throw Error('Old clipping/assembly enabled');return {scene:d.meta.id,profile:a.state.profile,valid:d.meta.valid,shown:d.meta.displayed_functions,triangles:d.meta.patch_triangles};});
    const checked=[];
    for(const field of fields){
     await page.selectOption('#metric-select',field);assert(!await page.isChecked('#fit-coverage'));
     checked.push(await page.evaluate(field=>{
      const a=window.metricViewer,d=a.state.data,spec=a.fields[field],g=a.content.children.find(o=>o.userData.surface);let n=0,sum=0,max=-Infinity;
      for(let i=0;i<d.meta.centers;i++)if(a.value(i,'valid')&&Number.isFinite(a.value(i,field))){const v=a.value(i,field);n++;sum+=v;max=Math.max(max,v);}
      const format=v=>Number.isFinite(v)?(v*spec.factor).toFixed(spec.digits)+' '+spec.unit:'未知';
      if(document.getElementById('mean-value').textContent!==format(n?sum/n:NaN)||document.getElementById('max-value').textContent!==format(n?max:NaN)||a.state.stats.known!==n)throw Error('Readout mismatch');
      if(n!==d.meta.valid)throw Error('Missing requested metric on valid functions');
      const color=g.geometry.attributes.color.array;for(const k of [0,Math.floor(d.patchOwners.length/2),d.patchOwners.length-1]){const expected=a.colorFor(a.value(d.patchOwners[k],field),spec.scale).toArray();for(let j=0;j<3;j++)if(Math.abs(color[3*k+j]-expected[j])>1e-6)throw Error('Color mismatch');}
      const gl=a.renderer.getContext();gl.finish();if(gl.isContextLost())throw Error('WebGL context lost');return {field,known:n,mean:n?sum/n:null,max:n?max:null};
     },field));
    }
    records.push({...record,metrics:checked});console.log('Checked',profile,scene,6,'metrics');
    if(profile==='qcircle-adaptive'){
     await page.check('#fit-coverage');await page.screenshot({path:path.join(out,`${scene}-coverage.png`),fullPage:true});
     await page.selectOption('#metric-select','normal_spread_deg');await page.screenshot({path:path.join(out,`${scene}-smoothness.png`),fullPage:true});
    }
   }
  }
  assert(!requests.some(u=>/\/mesh\.bin|\/fsgp\//.test(u)));
  await page.selectOption('#scene-select','forest');await wait('forest','q20-n30');await page.selectOption('#profile-select','qcircle-adaptive');await wait('forest','qcircle-adaptive');
  await page.evaluate(()=>{const a=window.metricViewer;let i=0;while(!a.state.data.patchCounts[i])i++;a.select(i);});
  assert.match(await page.locator('#selection-info').innerText(),/局部函数 #\d+.*凸包面积/);assert(await page.evaluate(()=>window.metricViewer.highlight.children.some(o=>o.userData.selectedPatch===window.metricViewer.state.selected)));
  const camera=await page.evaluate(()=>window.metricViewer.camera.position.toArray());await page.selectOption('#metric-select','slope_deg');assert.deepEqual(await page.evaluate(()=>window.metricViewer.camera.position.toArray()),camera);
  const shared=page.url();await page.reload({waitUntil:'networkidle'});await wait('forest','qcircle-adaptive');assert.match(await page.locator('#selection-info').innerText(),/局部函数 #\d+/);
  await page.click('#clear-selection');await page.check('#cloud-overlay');await page.waitForFunction(()=>{const a=window.metricViewer;return a.content.children.some(o=>o.isPoints&&o.geometry.attributes.position.count===a.state.data.meta.raw_points);});await page.uncheck('#cloud-overlay');
  assert.equal((await page.request.get(await page.locator('#csv-link').getAttribute('href'))).status(),200);assert.equal((await page.request.get(await page.locator('#details-link').evaluate(a=>a.href))).status(),200);
  await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.screenshot({path:path.join(out,'mobile.png'),fullPage:true});
  await page.goto(base+'?scene=forest&profile=qcircle-r07&metric=roughness_cost&mesh=1&coverage=1',{waitUntil:'networkidle'});await wait('forest','qcircle-r07');assert.equal(await page.locator('#mesh-mode').count(),0);assert.equal(await page.locator('#metric-select').inputValue(),'clearance_cost');assert(!new URL(page.url()).searchParams.has('mesh'));
  await page.goto(base+'?scene=table&profile=qcircle-r07&coverage=1',{waitUntil:'networkidle'});await page.waitForFunction(()=>window.metricViewer?.state.ready&&window.metricViewer.state.renderedScene==='table'&&document.getElementById('loading').hidden);assert.equal(await page.locator('#scene-select').inputValue(),'table');await page.goto(base+'metrics-history/?scene=table&profile=qcircle-r07&coverage=1',{waitUntil:'networkidle'});assert.equal(new URL(page.url()).pathname,'/metrics-history/');
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  fs.writeFileSync(path.join(out,'browser-audit.json'),JSON.stringify({passed:true,scenes,profiles,fields,metric_views:records.length*6,records,one_canvas:true,no_assembly_or_fsgp_controls:true,all_valid_hulls_displayed:true,no_assembled_mesh_requests:true,raw_toggle:true,selection_and_share:true,mobile:true,legacy_links:true,errors,failed},null,2)+'\n');
  console.log(`PASS: ${scenes.length} scenes × ${profiles.length} profiles × ${fields.length} metrics; whole convex domains, values, colors, mobile and legacy links`);
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
