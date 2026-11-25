const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const base='http://127.0.0.1:42482/',out=path.resolve(__dirname,'../outputs/visualization/surface-planner');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 const errors=[],failed=[],records=[];
 try{
  const page=await browser.newPage({viewport:{width:1440,height:1080}});page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push([r.status(),r.url()])});
  for(const scene of ['flat-corridor-platforms','hill','flat','table','table-low-040','table-low-042','table-low-060','surface-route-test','layer-choice','platform-underpass-070','platform-underpass-042','four-platforms']){
   await page.goto(base+'surface-planner/?scene='+scene,{waitUntil:'networkidle'});await page.waitForFunction(s=>window.surfacePlanner?.state.ready&&window.surfacePlanner.state.data.scene===s,scene);
   if(scene==='four-platforms'){
    assert.equal(await page.locator('#platform-summary tbody tr').count(),5);
    assert.equal(await page.locator('#platform-summary button').count(),0);
    const checked=await page.evaluate(()=>{const a=window.surfacePlanner,d=a.state.data,c=a.state.context;
     if(d.test_cases.length!==1||d.test_cases[0].id!=='all-platforms')throw Error('Local variants retained');
     if(a.terrain.children.filter(p=>p.userData.platformLabel).length!==5)throw Error('Missing platform labels');
     for(const k of [1,2,3])if(c.platforms[k].tabletop.min[1]!==-4||c.platforms[k].tabletop.max[1]!==4)throw Error('Side bypass retained');
     if(c.fifth_bypass.width_m!==1.2)throw Error('Missing final narrow path');
     return {routeCount:d.routes.length,values:a.state.evaluation.platforms.map(p=>[p.platform,p.overhead_passable,p.direct_climb_candidate]),status:d.status};});
    assert.deepEqual(checked.values,[[1,null,0],[2,1,0],[3,1,0],[4,0,1],[5,0,0]]);
    if(!checked.routeCount){
     assert(await page.isDisabled('#play'));assert(await page.isDisabled('#route'));
     assert(await page.evaluate(()=>{const a=window.surfacePlanner;return a.robot.children.length>0&&a.paths.children.length===2&&a.paths.children.every(p=>p.userData.endpoint)&&a.terrain.children[0].geometry.index.count===3*a.state.meta.patch_triangles}));
     assert((await page.textContent('#result-info')).includes('没有返回'));
     await page.click('#top');await page.screenshot({path:path.join(out,'five-platform-chain-overview.png'),fullPage:true});await page.click('#reset');
     await page.check('#upper-transparent');await page.screenshot({path:path.join(out,'five-platform-chain-transparent.png'),fullPage:true});await page.uncheck('#upper-transparent');
     records.push({scene,status:checked.status,routes:0,failed_maps:2});console.log('Checked',scene,'no full route, actual failure displayed');continue;
    }
    assert.equal(await page.evaluate(()=>{const a=window.surfacePlanner;return a.state.data.routes[a.state.route].case_id}),'all-platforms');
   }
   const count=await page.locator('#route option').count();
   for(let k=0;k<count;k++){
    await page.selectOption('#route',String(k));
    const record=await page.evaluate(()=>{const a=window.surfacePlanner,d=a.state.data,r=d.routes[a.state.route];
     if(d.planner!=='layer-projection-astar'||Object.keys(d.metric_weights).length!==5||'clearance_cost' in d.metric_weights)throw Error('Wrong planning method');
     if(a.terrain.children[0].geometry.index.count!==a.state.meta.patch_triangles*3)throw Error('Fitted faces changed');
     for(const p of r.frames)if(p.lift_flag.some(Boolean)||p.change_flag.some(Boolean)||p.swing_leg!==null||p.support.some(v=>!v))throw Error('Unexpected generated gait');
     if(r.map_mode==='base'&&r.events.length)throw Error('Base layer crosses another layer');
     for(const p of d.layer_graph.portals)if(!(p.delta_m<.4&&p.boundary_xy_gap_m<=.025+1e-9))throw Error('Bad boundary gate');
     const group=a.paths.children[a.state.route];if(group.children.filter(o=>o.userData.layerCrossing).length!==r.events.length)throw Error('Crossing visualization missing');
     return {scene:d.scene,map_id:r.map_id,mode:r.map_mode,frames:r.frames.length,crossings:r.events.length,cost:r.cost,maxFootZ:Math.max(...r.frames.flatMap(p=>p.feet.map(q=>q[2])))};
    });
    if(scene==='flat-corridor-platforms'&&record.mode==='overlay'){assert(record.crossings>0);assert(Math.abs(record.maxFootZ-.15)<1e-6)}
    if(scene==='hill'&&record.mode==='base')assert.equal(record.crossings,0);
    if(scene==='four-platforms'){
     record.case_id=await page.evaluate(()=>window.surfacePlanner.state.data.routes[window.surfacePlanner.state.route].case_id);
    }
    if(scene.startsWith('platform-underpass-')){
     const underpass=await page.evaluate(()=>{const a=window.surfacePlanner,r=a.state.data.routes[a.state.route],frames=a.underPlatform(r);
      if(!frames.length)throw Error('Did not enter under platform');
      if(a.terrain.children[0].material[1].transparent||a.terrain.children[0].material[1].opacity!==1)throw Error('Fitted roof hidden by default');
      const box=a.state.context.tabletop;if(box.min[1]!==-4||box.min[0]!==1||box.max[0]!==2.6)throw Error('Platform did not extend to robot right');
      if(a.terrain.children.filter(p=>p.userData.contextOutline).length!==5)throw Error('Platform/leg outlines missing');
      if(!document.getElementById('result-info').textContent.includes('平台下核查'))throw Error('Underpass explanation missing');
      return {minimum_body_height_m:Math.min(...frames.map(p=>p.body_height_m)),frames:frames.length,frame_index:r.frames.indexOf(frames[Math.floor(frames.length/2)])};
     });
     assert(Math.abs(underpass.minimum_body_height_m-(scene.endsWith('042')?.32:.40))<1e-6);record.underpass=underpass;
    }
    await page.check('#projection');assert(await page.evaluate(()=>{const a=window.surfacePlanner,g=a.projection.children[0].geometry.attributes.position.array;const z=g[2];for(let k=2;k<g.length;k+=3)if(Math.abs(g[k]-z)>1e-6)throw Error('Projection kept vertical layers');return a.projection.visible&&!a.terrain.visible&&!a.robot.visible}));
    if(scene==='flat-corridor-platforms')await page.screenshot({path:path.join(out,`layer-projection-${record.mode}-xy.png`),fullPage:true});
    await page.uncheck('#projection');await page.click('#reset');
    const index=await page.evaluate(()=>{const a=window.surfacePlanner,r=a.state.data.routes[a.state.route];return r.events.length?r.events[0].frame:Math.floor(r.frames.length/2)});await page.evaluate(k=>window.surfacePlanner.drawPose(k),index);
    if(scene==='flat-corridor-platforms'||scene==='hill')await page.screenshot({path:path.join(out,`layer-projection-${scene}-${record.mode}.png`),fullPage:true});
    if(record.underpass){await page.evaluate(k=>window.surfacePlanner.drawPose(k),record.underpass.frame_index);await page.screenshot({path:path.join(out,`${scene}-${record.mode}-overview.png`),fullPage:true});await page.check('#upper-transparent');assert(await page.evaluate(()=>window.surfacePlanner.terrain.children[0].material[1].transparent&&window.surfacePlanner.terrain.children[0].material[1].opacity===.18));await page.click('#follow');await page.screenshot({path:path.join(out,`${scene}-${record.mode}-underpass.png`),fullPage:true});await page.click('#reset');await page.uncheck('#upper-transparent');}
    if(scene==='four-platforms'){
     await page.click('#top');await page.screenshot({path:path.join(out,`five-platform-chain-${record.mode}.png`),fullPage:true});await page.click('#reset');
     await page.check('#upper-transparent');await page.click('#top');await page.screenshot({path:path.join(out,'five-platform-chain-overview.png'),fullPage:true});await page.uncheck('#upper-transparent');await page.click('#reset');
     const lowered=await page.evaluate(()=>{const a=window.surfacePlanner,r=a.state.data.routes[a.state.route];return r.frames.findIndex(p=>Math.abs(p.body[0])<.4&&p.body_height_m<.33)});
     if(lowered>=0){await page.check('#upper-transparent');await page.evaluate(k=>window.surfacePlanner.drawPose(k),lowered);await page.click('#follow');await page.screenshot({path:path.join(out,'five-platform-chain-underpass.png'),fullPage:true});await page.uncheck('#upper-transparent');await page.click('#reset');}
    }
    records.push(record);
   }
   await page.fill('#time','0');await page.dispatchEvent('#time','input');await page.click('#play');await page.waitForTimeout(250);assert(await page.evaluate(()=>window.surfacePlanner.state.frame>0));await page.click('#play');await page.click('#follow');await page.check('#headroom');await page.uncheck('#headroom');
   console.log('Checked',scene,count,'routes');
  }
  await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.screenshot({path:path.join(out,'layer-projection-mobile.png'),fullPage:true});
  // A failed search must still show the scene and supported start, never a path.
  await page.route('**/results/four-platforms/result.json',async route=>{const r=JSON.parse(fs.readFileSync(path.join(out,'results/four-platforms/result.json')));r.routes=[];r.recommended_map_id=null;r.status='no_validated_route';for(const m of r.projection_maps)m.status='no_validated_route';await route.fulfill({json:r});});
  await page.goto(base+'surface-planner/?scene=four-platforms',{waitUntil:'networkidle'});await page.waitForFunction(()=>window.surfacePlanner?.state.ready);
  assert(await page.isDisabled('#play'));assert((await page.textContent('#result-info')).includes('没有返回'));
  assert(await page.evaluate(()=>{const a=window.surfacePlanner;return a.robot.children.length>0&&a.paths.children.length===2&&a.paths.children.every(p=>p.userData.endpoint)}));
  await page.unroute('**/results/four-platforms/result.json');
  await page.goto(base+'surface-planner/history/before-layer-projection-20261123/index.html?scene=flat-corridor-platforms',{waitUntil:'networkidle'});await page.waitForFunction(()=>window.surfacePlanner?.state.ready);assert.equal(await page.evaluate(()=>window.surfacePlanner.state.data.schema_version||1),1);
  await page.goto(base+'surface-planner/history/before-right-side-correction-20261124/?scene=platform-underpass-042',{waitUntil:'networkidle'});await page.waitForFunction(()=>window.surfacePlanner?.state.ready);assert.equal(await page.locator('#scene option').count(),2);assert.equal(await page.evaluate(()=>window.surfacePlanner.state.data.scene),'platform-underpass-042');
  await page.goto(base+'surface-planner/history/before-five-platform-chain-20261124/?scene=four-platforms',{waitUntil:'networkidle'});await page.waitForFunction(()=>window.surfacePlanner?.state.ready);assert.equal(await page.locator('#scene option').count(),1);assert.equal(await page.locator('#platform-summary tbody tr').count(),4);assert.equal(await page.evaluate(()=>window.surfacePlanner.state.data.routes.length),10);
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  const audit={passed:true,records,projection_toggle:true,real_height_preserved:true,no_endpoint_height_lift_rule:true,no_generated_swings:true,crossing_candidates:true,failed_search_display:true,mobile:true,historical_viewer:true,four_platforms_archive:true,errors,failed};
  fs.writeFileSync(path.join(out,'layer-projection-browser-audit.json'),JSON.stringify(audit,null,2)+'\n');console.log('PASS layer projection browser');
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exit(1)});
