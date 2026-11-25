const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const base='http://127.0.0.1:42482/',out=path.resolve(__dirname,'../outputs/visualization/surface-planner');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 const errors=[],failed=[],records=[],metrics=[];
 try{
  const page=await browser.newPage({viewport:{width:1440,height:1120}});page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push([r.status(),r.url()])});
  await page.goto(base+'surface-planner/?scene=layer-choice',{waitUntil:'networkidle'});await page.waitForFunction(()=>window.surfacePlanner?.state.ready);
  assert.equal(await page.locator('#route option').count(),2);
  for(let k=0;k<2;k++){
   await page.selectOption('#route',String(k));
   const record=await page.evaluate(()=>{
    const a=window.surfacePlanner,r=a.state.data.routes[a.state.route];if(a.terrain.children[0].geometry.index.count!==a.state.meta.patch_triangles*3)throw Error('Original fitted faces changed');if(Object.keys(a.state.data.metric_weights).length!==5||'clearance_cost' in a.state.data.metric_weights)throw Error('Occupancy still contributes to foot cost');
    const order=r.frames.filter(p=>p.phase==='prepare'&&p.change_flag?.some(Boolean)).map(p=>p.swing_leg);
    for(const p of r.frames)if(p.change_flag.filter(Boolean).length>1)throw Error('Simultaneous change flags');
    const index=r.case_id==='duck-under'?r.frames.findIndex(p=>Math.abs(p.body[0])<.1):r.frames.findIndex(p=>p.phase==='swing'&&p.change_flag?.some(Boolean)&&p.body[2]>.65);
    return {id:r.case_id,cost:r.cost,frames:r.frames.length,minimumHeight:Math.min(...r.frames.map(p=>p.body_height_m)),maximumSupport:Math.max(...r.frames.flatMap(p=>p.feet.map((v,k)=>p.support[k]?v[2]:0))),order,index,opacity:a.terrain.children[0].material[1].opacity};
   });
   if(record.id==='duck-under'){assert.equal(record.minimumHeight,.32);assert(record.maximumSupport<.015);assert.equal(record.opacity,.06);assert.equal(record.order.length,0)}
   else{assert(record.maximumSupport>.47);assert(record.order.length>0);for(let n=0;n<record.order.length;n++)assert.equal(record.order[n],n%4);assert.equal(record.opacity,1)}
   assert(record.index>=0);await page.evaluate(k=>window.surfacePlanner.drawPose(k),record.index);
   if(record.id==='climb-over')assert((await page.locator('#pose-info').textContent()).includes('换层 change：左前')||(await page.locator('#pose-info').textContent()).match(/换层 change：(右前|左后|右后)/));
   await page.screenshot({path:path.join(out,`layer-choice-${record.id}.png`),fullPage:true});await page.click('#follow');await page.screenshot({path:path.join(out,`layer-choice-${record.id}-close.png`),fullPage:true});await page.click('#reset');
   await page.check('#headroom');assert(await page.evaluate(()=>window.surfacePlanner.headroom.visible));
   const colors=await page.evaluate(()=>{const c=window.surfacePlanner.headroom.children[0].geometry.attributes.color.array;let red=0,green=0;for(let i=0;i<c.length;i+=3){if(c[i]>.8)red++;else green++}return {red,green}});assert(colors.red>0&&colors.green>0);await page.uncheck('#headroom');
   await page.evaluate(()=>window.surfacePlanner.drawPose(0));await page.click('#play');await page.waitForTimeout(700);assert(await page.evaluate(()=>window.surfacePlanner.state.frame>0));await page.click('#play');records.push(record);
  }
  assert.equal((await page.request.get(base+'surface-planner/results/layer-choice/layer-tree.json')).status(),200);
  assert.equal((await page.request.get(base+'surface-planner/workflow.md')).status(),200);
  await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.screenshot({path:path.join(out,'layer-choice-mobile.png'),fullPage:true});await page.setViewportSize({width:1440,height:1120});
  await page.goto(base+'?scene=layer-choice&profile=qcircle-adaptive',{waitUntil:'networkidle'});await page.waitForFunction(()=>window.metricViewer?.state.ready&&window.metricViewer.state.renderedScene==='layer-choice');assert.equal(await page.locator('#scene-select option').count(),15);
  for(const profile of ['qcircle-adaptive','qcircle-r07','q15-n15','q20-n30']){
   await page.selectOption('#profile-select',profile);await page.waitForFunction(p=>window.metricViewer?.state.ready&&window.metricViewer.state.renderedProfile===p,profile);
   for(const field of ['clearance_cost','slope_deg','rmse_m','std_patch_m','normal_spread_deg','max_error_m']){
    await page.selectOption('#metric-select',field);metrics.push(await page.evaluate(field=>{const a=window.metricViewer,d=a.state.data,g=a.content.children.find(o=>o.userData.surface);if(d.meta.valid!==d.meta.displayed_functions||a.state.stats.known!==d.meta.valid||g.geometry.index.count!==3*d.meta.patch_triangles)throw Error('Missing hulls or metrics');for(const k of [0,Math.floor(d.patchOwners.length/2),d.patchOwners.length-1]){const expected=a.colorFor(a.value(d.patchOwners[k],field),a.fields[field].scale).toArray(),c=g.geometry.attributes.color.array;for(let j=0;j<3;j++)if(Math.abs(c[3*k+j]-expected[j])>1e-6)throw Error('Wrong metric colors')}return {profile:a.state.profile,field,valid:d.meta.valid}},field));
   }
  }
  assert.equal(metrics.length,24);assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  fs.writeFileSync(path.join(out,'layer-choice-browser-audit.json'),JSON.stringify({passed:true,records,metrics,binary_headroom:true,layer_tree:true,sequential_change_flags:true,playback:true,route_switch:true,original_fitted_surfaces:true,mobile:true,errors,failed},null,2)+'\n');console.log('PASS layer choice: both routes, sequential flags, 0/1, 24 metric views');
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exit(1)});
