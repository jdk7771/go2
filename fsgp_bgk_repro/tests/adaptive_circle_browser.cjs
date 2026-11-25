const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const assert=require('assert/strict'),fs=require('fs'),path=require('path');
const base='http://127.0.0.1:42482/',out=path.resolve(__dirname,'../outputs/visualization/assessment/adaptive-circle');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 const page=await browser.newPage({viewport:{width:1440,height:1100}}),errors=[],failed=[],records=[];
 page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push({url:r.url(),status:r.status()});});
 const wait=(scene,profile)=>page.waitForFunction(({scene,profile})=>{const s=window.metricViewer?.state;return s?.ready&&s.data.meta.id===scene&&s.renderedScene===scene&&s.renderedProfile===profile&&document.getElementById('loading').hidden;},{scene,profile},{timeout:90000});
 try{
  for(const scene of ['desert','mountain','snow','vocano']){
   await page.goto(base+`?scene=${scene}&profile=qcircle-adaptive&coverage=1`,{waitUntil:'networkidle'});await wait(scene,'qcircle-adaptive');
   assert.equal(await page.locator('canvas').count(),1);assert.equal(await page.locator('details,iframe,#plan-option,#plan-controls').count(),0);
   assert.equal(await page.locator('#scene-select option').count(),17);assert(await page.locator('#mesh-option').isHidden());
   assert.match(await page.locator('#source-info').innerText(),/自适应半径.*至少 15 点/);
   const fields=await page.locator('#metric-select option').evaluateAll(options=>options.map(o=>o.value));assert.equal(fields.length,11);
   const initial=await page.evaluate(()=>{const a=window.metricViewer,d=a.state.data,g=a.content.children.find(o=>o.userData.surface);if(!g||g.geometry.index.count!==d.meta.patch_triangles*3)throw Error('Missing fitted surface');if(d.radii.length!==d.meta.centers)throw Error('Missing per-function radius');if(d.meta.displayed_functions!==d.patchCounts.filter(n=>n>0).length)throw Error('Shown count differs');return {centers:d.meta.centers,valid:d.meta.valid,shown:d.meta.displayed_functions,triangles:d.meta.patch_triangles};});
   await page.evaluate(()=>{const a=window.metricViewer;let i=0;while(!a.state.data.patchCounts[i])i++;a.select(i);});
   assert.match(await page.locator('#selection-info').innerText(),/半径 [\d.]+ cm/);await page.click('#clear-selection');
   await page.screenshot({path:path.join(out,`${scene}-coverage.png`),fullPage:true});
   const checked=[];
   for(const field of fields){
    await page.selectOption('#metric-select',field);assert(!await page.isChecked('#fit-coverage'));
    checked.push(await page.evaluate(field=>{
     const a=window.metricViewer,d=a.state.data,spec=a.fields[field],g=a.content.children.find(o=>o.userData.surface);
     let sum=0,n=0,max=-Infinity;for(let i=0;i<d.meta.centers;i++)if(a.value(i,'valid')&&Number.isFinite(a.value(i,field))){const v=a.value(i,field);sum+=v;n++;max=Math.max(max,v);}
     const format=v=>Number.isFinite(v)?(v*spec.factor).toFixed(spec.digits)+(spec.unit?' '+spec.unit:''):'未知';
     if(document.getElementById('mean-value').textContent!==format(n?sum/n:NaN)||document.getElementById('max-value').textContent!==format(n?max:NaN)||a.state.stats.known!==n)throw Error('Readout mismatch');
     const colors=g.geometry.attributes.color.array;for(const k of [0,Math.floor(d.patchOwners.length/2),d.patchOwners.length-1]){const expected=a.colorFor(a.value(d.patchOwners[k],field),spec.scale).toArray();for(let j=0;j<3;j++)if(Math.abs(colors[3*k+j]-expected[j])>1e-6)throw Error('Metric color mismatch');}
     const gl=a.renderer.getContext();gl.finish();if(gl.isContextLost())throw Error('WebGL context lost');
     return {field,known:n,mean:n?sum/n:null,max:n?max:null};
    },field));
   }
   await page.selectOption('#metric-select','max_error_m');await page.screenshot({path:path.join(out,`${scene}-error.png`),fullPage:true});
   await page.check('#cloud-overlay');await page.waitForFunction(()=>{const a=window.metricViewer;return a.content.children.some(o=>o.isPoints&&o.geometry.attributes.position.count===a.state.data.meta.raw_points);});await page.uncheck('#cloud-overlay');
   assert.equal((await page.request.get(await page.locator('#csv-link').getAttribute('href'))).status(),200);
   assert.equal((await page.request.get(await page.locator('#details-link').getAttribute('href'))).status(),200);
   records.push({scene,...initial,metrics:checked});console.log('Checked adaptive',scene,fields.length,'metrics');
  }
  await page.selectOption('#scene-select','table');await wait('table','qcircle-r07');
  assert(await page.locator('#profile-select option[value=qcircle-adaptive]').evaluate(o=>o.disabled));
  for(const [profile,pattern]of [['q15-n15',/15×15 cm/],['q20-n30',/20×20 cm/],['qcircle-r07',/半径 7 cm/]]){
   await page.selectOption('#profile-select',profile);await wait('table',profile);assert.match(await page.locator('#source-info').innerText(),pattern);await page.selectOption('#metric-select','slope_deg');
  }
  await page.goto(base+'?scene=steps&profile=qcircle-adaptive&coverage=1',{waitUntil:'networkidle'});await wait('steps','qcircle-r07');
  await page.goto(base+'?scene=desert&profile=qcircle-adaptive&coverage=1',{waitUntil:'networkidle'});await wait('desert','qcircle-adaptive');
  await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.screenshot({path:path.join(out,'mobile.png'),fullPage:true});
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  fs.writeFileSync(path.join(out,'browser-audit.json'),JSON.stringify({records,errors,failed,metric_views:44,scene_count:17,old_profiles_verified:3,partial_profile_fallback:true,mobile:true},null,2)+'\n');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1);});
