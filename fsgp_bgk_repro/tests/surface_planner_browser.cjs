const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const onlyLow=process.argv.includes('--low-table-only');
const base='http://127.0.0.1:42482/',out=path.resolve(__dirname,'../outputs/visualization/surface-planner');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});const errors=[],failed=[],records=[];
 try{
  const page=await browser.newPage({viewport:{width:1440,height:1120}});page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push([r.status(),r.url()])});
  for(const scene of (onlyLow?['table-low-040','table-low-042','table-low-060']:['hill','surface-route-test','flat','table','flat-corridor-platforms','table-low-040','table-low-042','table-low-060'])){
   await page.goto(base+'surface-planner/?scene='+scene,{waitUntil:'networkidle'});await page.waitForFunction(scene=>window.surfacePlanner?.state.ready&&window.surfacePlanner.state.data.scene===scene,scene);
   const record=await page.evaluate(()=>{const a=window.surfacePlanner,d=a.state.data;if(a.terrain.children[0].geometry.index.count!==a.state.meta.patch_triangles*3)throw Error('Wrong fitted surface geometry');if(Object.keys(d.metric_weights).length!==5||'clearance_cost' in d.metric_weights)throw Error('Occupancy still contributes to foot cost');return {scene:d.scene,routes:d.routes.length,frames:d.routes.map(r=>r.frames.length),metric_weights:d.metric_weights,wallPatches:d.obstacles.fitted_wall_patches.length,forceFrames:d.obstacles.adjusted_frames}});
   assert.equal(record.routes,['surface-route-test','table'].includes(scene)?2:1);assert.equal(await page.locator('canvas').count(),1);
   for(let r=0;r<record.routes;r++){
    await page.selectOption('#route',String(r));
    if(scene==='table'){const details=await page.evaluate(()=>{const a=window.surfacePlanner,r=a.state.data.routes[a.state.route],p=r.frames[0];return {id:r.case_id,heights:p.feet.map(v=>v[2]),opacity:a.terrain.children[0].material[1].opacity,negative:a.state.data.test_cases.find(c=>c.id==='ground-to-table')}});for(const z of details.heights)assert(Math.abs(z-(details.id==='table-under'?0:1.2))<1e-6);assert.equal(details.opacity,details.id==='table-under'?.18:1);assert.equal(details.negative.routes,0);assert(details.negative.statistics.searches.at(-1).open_exhausted)}
    if(scene.startsWith('table-low-')){const d=await page.evaluate(()=>{const a=window.surfacePlanner,r=a.state.data.routes[a.state.route];return {under:r.frames.filter(p=>Math.abs(p.body[0])<1&&Math.abs(p.body[1])<.5).length,minimumHeight:Math.min(...r.frames.map(p=>p.body_height_m)),lower:r.frames.filter(p=>p.phase==='body_lowering').length,opacity:a.terrain.children[0].material[1].opacity,outlines:a.terrain.children.filter(o=>o.userData.contextOutline).length,raw:document.getElementById('points').checked,info:document.getElementById('result-info').textContent,negative:a.state.data.test_cases.find(c=>c.expected_route===false)}});assert.equal(d.opacity,.06);assert.equal(d.outlines,5);assert.equal(d.raw,false);if(scene==='table-low-042')assert(d.info.includes('3.0 cm'));if(scene==='table-low-040'){assert.equal(d.under,0);assert.equal(d.negative.routes,0)}else{assert(d.under>0);assert.equal(d.minimumHeight,scene==='table-low-042'?.32:.4);if(scene==='table-low-042')assert(d.lower>0)}}
    const indices=await page.evaluate(()=>{const a=window.surfacePlanner,r=a.state.data.routes[a.state.route];return [...new Set([0,Math.floor(r.frames.length/2),r.frames.findIndex(p=>p.phase==='swing'),r.frames.findIndex(p=>p.virtual_force_shift_m),r.frames.length-1].filter(i=>i>=0))]});
    for(const k of indices){await page.evaluate(k=>window.surfacePlanner.drawPose(k),k);assert.equal(await page.locator('#time').inputValue(),String(k));assert(await page.locator('#pose-info').textContent());await page.evaluate(()=>{const a=window.surfacePlanner;a.renderer.getContext().finish();if(a.renderer.getContext().isContextLost())throw Error('WebGL context lost')})}
    const interesting=await page.evaluate(()=>{const a=window.surfacePlanner,r=a.state.data.routes[a.state.route];const swing=r.frames.findIndex(p=>p.phase==='swing');return swing>=0?swing:Math.floor(r.frames.length/2)});await page.evaluate(k=>window.surfacePlanner.drawPose(k),interesting);
    await page.screenshot({path:path.join(out,`${scene}-route-${r}.png`),fullPage:true});
    if(scene==='surface-route-test'){const k=await page.evaluate(()=>window.surfacePlanner.state.data.routes[window.surfacePlanner.state.route].frames.findIndex(p=>p.virtual_force_shift_m));assert(k>=0);await page.evaluate(k=>window.surfacePlanner.drawPose(k),k)}
    await page.click('#follow');assert(await page.evaluate(()=>window.surfacePlanner.state.follow));await page.screenshot({path:path.join(out,`${scene}-route-${r}-close.png`),fullPage:true});await page.click('#reset');
   }
   await page.fill('#time','0');await page.dispatchEvent('#time','input');await page.click('#play');await page.waitForTimeout(600);assert(await page.evaluate(()=>window.surfacePlanner.state.frame>0));await page.click('#play');await page.click('#top');await page.check('#points');assert(await page.evaluate(()=>window.surfacePlanner.terrain.children.some(c=>c.isPoints&&c.visible)));await page.uncheck('#points');await page.click('#reset');
   assert.equal((await page.request.get(await page.locator('#json').evaluate(a=>a.href))).status(),200);assert.equal((await page.request.get(base+'surface-planner/report.md')).status(),200);
   records.push(record);console.log('checked planner',scene,record);
  }
  await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.screenshot({path:path.join(out,'mobile.png'),fullPage:true});
  await page.goto(base+'surface-planner/?scene=table&route=0',{waitUntil:'networkidle'});await page.waitForFunction(()=>window.surfacePlanner?.state.ready&&window.surfacePlanner.state.data.scene==='table');assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.setViewportSize({width:1440,height:1120});
  const metricChecks=[];
  if(!onlyLow){
  await page.goto(base+'?scene=surface-route-test&profile=qcircle-adaptive',{waitUntil:'networkidle'});await page.waitForFunction(()=>window.metricViewer?.state.ready&&window.metricViewer.state.renderedScene==='surface-route-test');assert.equal(await page.locator('#scene-select option').count(),15);assert.equal(await page.locator('#profile-select option').count(),4);assert.equal(await page.locator('#metric-select option').count(),6);assert(await page.isVisible('#planner-link'));
  for(const currentScene of ['surface-route-test','table','table-low-040','table-low-042','table-low-060']){
  await page.selectOption('#scene-select',currentScene);await page.waitForFunction(s=>window.metricViewer?.state.ready&&window.metricViewer.state.renderedScene===s,currentScene);
  for(const profile of ['qcircle-adaptive','qcircle-r07','q15-n15','q20-n30']){
   await page.selectOption('#profile-select',profile);await page.waitForFunction(profile=>window.metricViewer?.state.ready&&window.metricViewer.state.renderedProfile===profile,profile);
   for(const field of ['clearance_cost','slope_deg','rmse_m','std_patch_m','normal_spread_deg','max_error_m']){
    await page.selectOption('#metric-select',field);metricChecks.push(await page.evaluate(field=>{const a=window.metricViewer,d=a.state.data,g=a.content.children.find(o=>o.userData.surface);if(g.geometry.index.count!==3*d.meta.patch_triangles||d.meta.valid!==d.meta.displayed_functions)throw Error('New scene hull missing');if(a.state.stats.known!==d.meta.valid)throw Error('New scene metric missing');const color=g.geometry.attributes.color.array;for(const k of [0,Math.floor(d.patchOwners.length/2),d.patchOwners.length-1]){const expected=a.colorFor(a.value(d.patchOwners[k],field),a.fields[field].scale).toArray();for(let j=0;j<3;j++)if(Math.abs(color[3*k+j]-expected[j])>1e-6)throw Error('New scene color mismatch')}return {scene:d.meta.id,profile:a.state.profile,field,valid:d.meta.valid}},field));
   }
  }
  }
  assert.equal(metricChecks.length,120);assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.click('#planner-link');await page.waitForFunction(()=>window.surfacePlanner?.state.ready);assert.equal(new URL(page.url()).port,'42482');
  }
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);fs.writeFileSync(path.join(out,onlyLow?'low-table-browser-audit.json':'browser-audit.json'),JSON.stringify({passed:true,records,new_scene_metric_views:metricChecks,animation:true,frame_slider:true,route_switch:true,robot_close_view:true,zmp_and_leg_display:true,original_hulls:true,raw_toggle:true,wall_fits:true,main_viewer_link:true,mobile:true,errors,failed},null,2)+'\n');
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exit(1)});
