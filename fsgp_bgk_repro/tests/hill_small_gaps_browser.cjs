const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const base='http://127.0.0.1:42482/',out=path.resolve(__dirname,'../outputs/visualization/convex-assessment/hill-small-gaps');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 const page=await browser.newPage({viewport:{width:1440,height:1100}}),errors=[],failed=[],records=[];
 page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push([r.status(),r.url()]);});
 const wait=(scene,profile)=>page.waitForFunction(({scene,profile})=>{const a=window.metricViewer;return a?.state.ready&&a.state.renderedScene===scene&&a.state.renderedProfile===profile&&document.getElementById('loading').hidden;},{scene,profile},{timeout:90000});
 const fill=()=>page.waitForFunction(()=>window.metricViewer.content.children.some(o=>o.userData.gapFill));
 try{
  await page.goto(base+'?scene=hill&profile=qcircle-adaptive&coverage=1&gaps=0',{waitUntil:'networkidle'});await wait('hill','qcircle-adaptive');
  assert(await page.isVisible('#small-gap-option'));assert(!await page.isChecked('#small-gap-fill'));assert.equal(await page.locator('canvas').count(),1);
  await page.screenshot({path:path.join(out,'hill-before.png'),fullPage:true});const camera=await page.evaluate(()=>window.metricViewer.camera.position.toArray());
  await page.check('#small-gap-fill');await fill();assert.deepEqual(await page.evaluate(()=>window.metricViewer.camera.position.toArray()),camera);
  await page.screenshot({path:path.join(out,'hill-after.png'),fullPage:true});assert.match(await page.locator('#fit-summary').innerText(),/小缝显示补全/);
  const profiles=await page.locator('#profile-select option').evaluateAll(o=>o.map(x=>x.value));assert.equal(profiles.length,4);
  const fields=await page.locator('#metric-select option').evaluateAll(o=>o.map(x=>x.value));assert.equal(fields.length,6);
  for(const profile of profiles){
   await page.selectOption('#profile-select',profile);await wait('hill',profile);await fill();
   const info=await page.evaluate(async()=>{const a=window.metricViewer,d=a.state.data,g=d.gap;const digest=await crypto.subtle.digest('SHA-256',d.models.buffer);const hash=Array.from(new Uint8Array(digest),b=>b.toString(16).padStart(2,'0')).join('');if(hash!==g.meta.original_files_sha256['models.bin'])throw Error('Base models changed');if(g.meta.threshold_cm2!==9||!g.meta.strictly_less||g.meta.new_fits!==0||g.meta.filled_gaps+g.meta.unfilled_small_gaps!==g.meta.small_gaps_before||!g.meta.geometry.large_gaps_unchanged)throw Error('Gap policy mismatch');const mesh=a.content.children.find(o=>o.userData.gapFill);if(mesh.geometry.index.count!==g.meta.overlay_triangles*3)throw Error('Overlay not displayed');for(let k=0;k<g.owners.length;k++){const id=g.owners[k];if(!a.value(id,'valid'))throw Error('Gap refers to invalid fit');}return {profile:a.state.profile,valid:d.meta.valid,small:g.meta.filled_gaps,retainedLarge:g.meta.large_gaps_retained,area:g.meta.filled_area_m2};});
   for(const field of fields){
    await page.selectOption('#metric-select',field);
    await page.evaluate(field=>{const a=window.metricViewer,d=a.state.data,spec=a.fields[field];let n=0,sum=0,max=-Infinity;for(let i=0;i<d.meta.centers;i++)if(a.value(i,'valid')&&Number.isFinite(a.value(i,field))){const v=a.value(i,field);n++;sum+=v;max=Math.max(max,v);}const format=v=>(v*spec.factor).toFixed(spec.digits)+' '+spec.unit;if(document.getElementById('mean-value').textContent!==format(sum/n)||document.getElementById('max-value').textContent!==format(max))throw Error('Metrics changed');for(const g of a.content.children.filter(o=>o.userData.surface)){const owners=g.userData.owners,c=g.geometry.attributes.color.array;for(const k of [0,Math.floor(owners.length/2),owners.length-1]){const expected=a.colorFor(a.value(owners[k],field),spec.scale).toArray();for(let j=0;j<3;j++)if(Math.abs(c[3*k+j]-expected[j])>1e-6)throw Error('Parent metric color mismatch');}}},field);
   }
   records.push(info);console.log('Checked hill small gaps',profile,6,'metrics');
   await page.uncheck('#small-gap-fill');await page.waitForFunction(()=>!window.metricViewer.content.children.some(o=>o.userData.gapFill));
   assert(!new URL(page.url()).searchParams.get('gaps')||new URL(page.url()).searchParams.get('gaps')==='0');
   await page.check('#small-gap-fill');await fill();
  }
  await page.selectOption('#profile-select','qcircle-adaptive');await wait('hill','qcircle-adaptive');await fill();await page.check('#fit-coverage');
  await page.click('[data-angle="top"]');await page.screenshot({path:path.join(out,'hill-top-after.png'),fullPage:true});
  const shared=page.url();await page.reload({waitUntil:'networkidle'});await wait('hill','qcircle-adaptive');await fill();assert(await page.isChecked('#small-gap-fill'));
  await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.screenshot({path:path.join(out,'hill-mobile.png'),fullPage:true});
  await page.selectOption('#scene-select','forest');await wait('forest','qcircle-adaptive');assert(await page.isVisible('#small-gap-option'));assert(await page.evaluate(()=>window.metricViewer.state.data.gap.meta.scene==='forest'));
  await page.selectOption('#profile-select','qcircle-adaptive-no-offset');await wait('forest','qcircle-adaptive-no-offset');assert.match(await page.locator('#metric-note').innerText(),/取消中心预测偏移上限/);
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  fs.writeFileSync(path.join(out,'browser-audit.json'),JSON.stringify({passed:true,metric_views:records.length*6,records,one_canvas:true,base_models_unchanged:true,overlay_toggle:true,share:true,mobile:true,forest_unaffected:true,shared,errors,failed},null,2)+'\n');
  console.log('PASS: hill small-gap completion across four settings and six metrics, toggle, sharing, mobile and forest compatibility');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
