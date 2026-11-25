const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const base='http://127.0.0.1:42482/',out=path.resolve(__dirname,'../outputs/visualization/convex-assessment/forest-center-offset');
const variant='qcircle-adaptive-no-offset';
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 const page=await browser.newPage({viewport:{width:1440,height:1100}}),errors=[],failed=[],metrics=[];
 page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push([r.status(),r.url()]);});
 const wait=(scene,profile)=>page.waitForFunction(({scene,profile})=>{const a=window.metricViewer;return a?.state.ready&&a.state.renderedScene===scene&&a.state.renderedProfile===profile&&document.getElementById('loading').hidden;},{scene,profile},{timeout:90000});
 try{
  await page.goto(base+`?scene=forest&profile=${variant}&coverage=1`,{waitUntil:'networkidle'});await wait('forest',variant);
  assert.equal(await page.locator('#profile-select option').count(),5);assert.equal(await page.locator('canvas').count(),1);
  assert.match(await page.locator('#metric-note').innerText(),/取消中心预测偏移上限.*RMSE≤8 mm/);
  const result=await page.evaluate(()=>{const a=window.metricViewer,d=a.state.data;if(d.meta.config.center_offset_check!==false||d.meta.config.height_gate_m!==.04||d.meta.config.max_rmse_m!==.008)throw Error('Wrong ablation');let count=0,max=0,id=-1;for(let i=0;i<d.meta.centers;i++)if(a.value(i,'valid')){const x=Math.abs(a.value(i,'c0'));if(x>.04)count++;if(x>max){max=x;id=i;}if(a.value(i,'rmse_m')>.008)throw Error('RMSE gate removed');}return {valid:d.meta.valid,displayed:d.meta.displayed_functions,above4cm:count,maximumOffset:max,example:id};});
  assert(result.valid>4666&&result.valid===result.displayed&&result.above4cm>0);
  await page.screenshot({path:path.join(out,'forest-no-offset-coverage.png'),fullPage:true});
  const fields=await page.locator('#metric-select option').evaluateAll(o=>o.map(x=>x.value));assert.equal(fields.length,6);
  for(const field of fields){
   await page.selectOption('#metric-select',field);
   metrics.push(await page.evaluate(field=>{const a=window.metricViewer,d=a.state.data,spec=a.fields[field],g=a.content.children.find(o=>o.userData.surface);let count=0,sum=0,max=-Infinity;for(let i=0;i<d.meta.centers;i++)if(a.value(i,'valid')&&Number.isFinite(a.value(i,field))){const v=a.value(i,field);count++;sum+=v;max=Math.max(max,v);}if(count!==d.meta.valid)throw Error('Missing metric');const format=v=>(v*spec.factor).toFixed(spec.digits)+' '+spec.unit;if(document.getElementById('mean-value').textContent!==format(sum/count)||document.getElementById('max-value').textContent!==format(max))throw Error('Readout mismatch');const c=g.geometry.attributes.color.array;for(const k of [0,Math.floor(d.patchOwners.length/2),d.patchOwners.length-1]){const expected=a.colorFor(a.value(d.patchOwners[k],field),spec.scale).toArray();for(let j=0;j<3;j++)if(Math.abs(c[3*k+j]-expected[j])>1e-6)throw Error('Color mismatch');}return {field,count,mean:sum/count,max};},field));
  }
  await page.selectOption('#metric-select','std_patch_m');await page.screenshot({path:path.join(out,'forest-no-offset-uncertainty.png'),fullPage:true});
  await page.evaluate(i=>window.metricViewer.select(i),result.example);assert.match(await page.locator('#selection-info').innerText(),/中心预测偏移/);
  const shared=page.url();await page.reload({waitUntil:'networkidle'});await wait('forest',variant);assert.match(await page.locator('#selection-info').innerText(),/中心预测偏移/);
  await page.click('#clear-selection');await page.selectOption('#metric-select','clearance_cost');await page.check('#fit-coverage');const camera=await page.evaluate(()=>window.metricViewer.camera.position.toArray());
  await page.selectOption('#profile-select','qcircle-adaptive');await wait('forest','qcircle-adaptive');assert.deepEqual(await page.evaluate(()=>window.metricViewer.camera.position.toArray()),camera);
  await page.screenshot({path:path.join(out,'forest-original-coverage.png'),fullPage:true});
  await page.selectOption('#profile-select',variant);await wait('forest',variant);await page.check('#cloud-overlay');await page.waitForFunction(()=>window.metricViewer.content.children.some(o=>o.isPoints&&o.geometry.attributes.position.count===141068));await page.uncheck('#cloud-overlay');
  assert.equal((await page.request.get(await page.locator('#csv-link').getAttribute('href'))).status(),200);
  await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.screenshot({path:path.join(out,'forest-no-offset-mobile.png'),fullPage:true});
  await page.selectOption('#scene-select','flat');await wait('flat','qcircle-adaptive');assert.equal(await page.locator('#profile-select option').count(),4);assert.equal(await page.locator(`#profile-select option[value="${variant}"]`).count(),0);
  await page.goto(base+`?scene=flat&profile=${variant}&coverage=1`,{waitUntil:'networkidle'});await wait('flat','qcircle-adaptive');
  await page.selectOption('#scene-select','forest');await wait('forest','qcircle-adaptive');assert.equal(await page.locator('#profile-select option').count(),5);
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  fs.writeFileSync(path.join(out,'browser-audit.json'),JSON.stringify({passed:true,metrics,result,metric_views:metrics.length,forest_only:true,scene_switch_fallback:true,raw_overlay:true,selection_and_share:true,mobile:true,shared,errors,failed},null,2)+'\n');
  console.log('PASS: forest ablation, six metrics, selection, share, raw points, mobile and scene-scoped options');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
