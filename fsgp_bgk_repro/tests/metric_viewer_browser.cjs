const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const base='http://127.0.0.1:42482/',out=path.resolve(__dirname,'../outputs/visualization/metrics');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 const page=await browser.newPage({viewport:{width:1440,height:1100}}),errors=[],failed=[],records=[];
 page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push({url:r.url(),status:r.status()});});
 const wait=(id,profile='q20-n30')=>page.waitForFunction(({id,profile})=>window.metricViewer?.state.ready&&window.metricViewer.state.data.meta.id===id&&window.metricViewer.state.renderedScene===id&&window.metricViewer.state.renderedProfile===profile&&document.getElementById('loading').hidden,{id,profile},{timeout:90000});
 try{
  await page.goto(base,{waitUntil:'networkidle'});await wait('steps');
  assert.equal(await page.locator('canvas').count(),1);assert.equal(await page.locator('details,iframe').count(),0);
  const scenes=await page.locator('#scene-select option').evaluateAll(options=>options.map(o=>o.value));assert.equal(scenes.length,17);
  const fields=await page.locator('#metric-select option').evaluateAll(options=>options.map(o=>o.value));assert.equal(fields.length,11);
  const profiles=await page.locator('#profile-select option').evaluateAll(options=>options.map(o=>o.value));assert.deepEqual(profiles,['q20-n30','q15-n15']);
  for(const profile of profiles){
   const current=await page.locator('#scene-select').inputValue();await page.selectOption('#profile-select',profile);await wait(current,profile);
  for(const id of scenes){
   await page.selectOption('#scene-select',id);await wait(id,profile);
   assert.match(await page.locator('#source-info').innerText(),profile==='q15-n15'?/15×15 cm，至少 15 点/:/20×20 cm，至少 30 点/);
   const checked=[];
   for(const field of fields){
    await page.selectOption('#metric-select',field);
    const result=await page.evaluate(field=>{
     const a=window.metricViewer,s=a.state,d=s.data,g=a.content.children.find(o=>o.userData.surface),f=a.fields[field];
     let n=0,sum=0,max=-Infinity;for(let i=0;i<d.meta.centers;i++)if(a.value(i,'valid')&&Number.isFinite(a.value(i,field))){n++;sum+=a.value(i,field);max=Math.max(max,a.value(i,field));}
     const values=state=>Number.isFinite(state)?(state*f.factor).toFixed(f.digits)+(f.unit?' '+f.unit:''):'未知';
     if(document.getElementById('mean-value').textContent!==values(n?sum/n:NaN))throw Error('mean does not match stored values');
     if(document.getElementById('max-value').textContent!==values(n?max:NaN))throw Error('max does not match stored values');
     if(s.stats.known!==n)throw Error('unknown values counted incorrectly');
     if(d.meta.valid&&(!g||g.userData.field!==field||g.geometry.index.count!==d.patchFaces.length))throw Error('incorrect patch geometry or field');
     if(g){const colors=g.geometry.attributes.color.array;for(const k of [0,Math.floor(d.patchOwners.length/2),d.patchOwners.length-1]){if(k<0)continue;const v=a.value(d.patchOwners[k],field),expected=a.colorFor(v,f.scale).toArray();for(let j=0;j<3;j++)if(Math.abs(colors[k*3+j]-expected[j])>1e-6)throw Error('surface colors do not match owner metric');}}
     const gl=a.renderer.getContext();gl.finish();if(gl.isContextLost())throw Error('WebGL context lost');
     return {field,known:n,valid:d.meta.valid,mean:n?sum/n:null,max:n?max:null};
    },field);checked.push(result);
   }
   records.push({profile,scene:id,fields:checked});console.log('Checked',profile,id,fields.length,'metrics');
  }
  }
  await page.selectOption('#profile-select','q20-n30');await wait('vocano');
  const colors=await page.evaluate(()=>{const a=window.metricViewer;return [a.colorFor(0,1).getHexString(),a.colorFor(.5,1).getHexString(),a.colorFor(1,1).getHexString(),a.colorFor(NaN,1).getHexString()];});
  assert.deepEqual(colors,['ffffff','ff8080','ff0000','8d969f']);
  await page.selectOption('#scene-select','steps');await wait('steps');await page.selectOption('#metric-select','jump_b_m');
  await page.screenshot({path:path.join(out,'steps-white-red.png'),fullPage:true});
  await page.click('[data-angle=top]');const box=await page.locator('#viewport').boundingBox();await page.locator('#viewport').click({position:{x:box.width/2,y:box.height/2}});await page.waitForFunction(()=>window.metricViewer.state.selected!==null);assert.match(await page.locator('#selection-info').innerText(),/局部函数 #\d+/);await page.click('#clear-selection');
  const camera=await page.evaluate(()=>window.metricViewer.camera.position.toArray());await page.selectOption('#metric-select','slope_deg');assert.deepEqual(await page.evaluate(()=>window.metricViewer.camera.position.toArray()),camera);
  await page.check('#mesh-mode');await page.waitForFunction(()=>window.metricViewer.content.children.find(o=>o.userData.surface)?.userData.mode==='mesh');
  assert(await page.evaluate(()=>{const a=window.metricViewer;return a.content.children.find(o=>o.userData.surface).geometry.index.count===a.state.data.mesh.length;}));
  await page.uncheck('#mesh-mode');await page.waitForFunction(()=>window.metricViewer.content.children.find(o=>o.userData.surface)?.userData.mode==='patch');
  await page.check('#cloud-overlay');await page.waitForFunction(()=>window.metricViewer.content.children.some(o=>o.isPoints&&o.geometry.attributes.position.count===window.metricViewer.state.data.meta.raw_points));await page.uncheck('#cloud-overlay');
  await page.evaluate(()=>{const a=window.metricViewer;let max=-Infinity,id=0;for(let i=0;i<a.state.data.meta.centers;i++)if(a.value(i,'valid')&&a.value(i,a.state.metric)>max){max=a.value(i,a.state.metric);id=i;}a.select(id);});
  assert.match(await page.locator('#selection-info').innerText(),/#\d+.*坡度/);const shared=page.url();await page.goto(shared,{waitUntil:'networkidle'});await wait('steps');assert.match(await page.locator('#selection-info').innerText(),/#\d+/);
  await page.goto(base+'?scene=corridor-060&metric=clearance_cost',{waitUntil:'networkidle'});await wait('corridor-060');assert(!await page.isChecked('#hide-overhead'));await page.check('#hide-overhead');await page.screenshot({path:path.join(out,'corridor-white-red.png'),fullPage:true});
  await page.selectOption('#metric-select','body_cost');await page.screenshot({path:path.join(out,'body-white-red.png'),fullPage:true});
  const csv=await page.locator('#csv-link').getAttribute('href');assert.equal((await page.request.get(csv)).status(),200);
  await page.goto(base+'?scene=forest&metric=jump_b_m',{waitUntil:'networkidle'});await wait('forest');
  assert(await page.evaluate(()=>!window.metricViewer.content.children.some(o=>o.userData.coverageEdges)));
  const forest=await page.evaluate(()=>{const a=window.metricViewer,d=a.state.data;return {coverage:d.coverage,rendered:d.patchCounts.filter(n=>n>0).length,markers:a.content.children.filter(o=>o.userData.kind).map(o=>[o.userData.kind,o.geometry.attributes.position.count])};});
  assert.deepEqual(forest,{coverage:{insufficient:19995,rejected:547,validNoPatch:28,validUnassembled:583},rendered:3930,markers:[['unfitted',20542],['unshown',28]]});
  assert.match(await page.locator('#fit-summary').innerText(),/3,958 \/ 24,500（16.2%）/);
  await page.screenshot({path:path.join(out,'forest-white-red.png'),fullPage:true});
  await page.check('#fit-coverage');assert(await page.locator('#metric-legend').isHidden());assert(await page.locator('#coverage-legend').isVisible());
  assert(await page.evaluate(()=>window.metricViewer.content.children.some(o=>o.userData.coverageEdges&&o.visible)));
  assert(await page.evaluate(()=>{const a=window.metricViewer,d=a.state.data,g=a.content.children.find(o=>o.userData.surface),colors=g.geometry.attributes.color.array,want=a.colorFor(NaN,1).set('#2586cb').toArray();return g.geometry.index.array===d.patchFaces&&[0,Math.floor(colors.length/6)*3,colors.length-3].every(k=>want.every((v,j)=>Math.abs(v-colors[k+j])<1e-6));}));
  await page.screenshot({path:path.join(out,'forest-fit-coverage.png'),fullPage:true});
  const beforeProfileCamera=await page.evaluate(()=>window.metricViewer.camera.position.toArray());
  await page.selectOption('#profile-select','q15-n15');await wait('forest','q15-n15');assert.deepEqual(await page.evaluate(()=>window.metricViewer.camera.position.toArray()),beforeProfileCamera);
  assert.equal(await page.evaluate(()=>window.metricViewer.state.data.meta.valid),11845);assert.match(await page.locator('#fit-summary').innerText(),/11,845 \/ 24,500（48.3%）/);
  assert.match(await page.locator('#csv-link').getAttribute('href'),/profiles\/q15-n15\/forest\/per-model.csv/);assert.equal((await page.request.get(await page.locator('#csv-link').getAttribute('href'))).status(),200);
  assert.match(await page.locator('#details-link').getAttribute('href'),/profiles\/q15-n15\/forest\/data.json/);
  assert(await page.evaluate(()=>{const d=window.metricViewer.state.data,half=d.meta.config.patch_size_m/2;for(let k=0;k<d.patchOwners.length;k++){const i=d.patchOwners[k];for(let j=0;j<2;j++)if(Math.abs(d.patches[k*3+j]-d.models[i*d.meta.fields.length+j])>half+1e-5)return false;}return true;}));
  await page.screenshot({path:path.join(out,'forest-q15-fit-coverage.png'),fullPage:true});
  await page.check('#cloud-overlay');await page.waitForFunction(()=>window.metricViewer.content.children.some(o=>o.isPoints&&o.geometry.attributes.position.count===window.metricViewer.state.data.meta.raw_points));await page.uncheck('#cloud-overlay');
  const profileShared=page.url();await page.goto(profileShared,{waitUntil:'networkidle'});await wait('forest','q15-n15');assert(await page.isChecked('#fit-coverage'));assert.equal(await page.locator('#profile-select').inputValue(),'q15-n15');
  await page.check('#mesh-mode');await page.waitForFunction(()=>window.metricViewer.content.children.find(o=>o.userData.surface)?.userData.mode==='mesh');
  assert(await page.evaluate(()=>{const a=window.metricViewer,d=a.state.data;return a.content.children.find(o=>o.userData.surface).geometry.index.count===d.meta.triangles*3;}));
  await page.screenshot({path:path.join(out,'forest-q15-assembled.png'),fullPage:true});
  await page.uncheck('#mesh-mode');await page.selectOption('#profile-select','q20-n30');await wait('forest');
  await page.evaluate(()=>{const a=window.metricViewer;let id=0;while(a.value(id,'valid'))id++;a.select(id);});assert.match(await page.locator('#selection-info').innerText(),/未拟合|未通过/);
  await page.evaluate(()=>{const a=window.metricViewer;let id=0;while(!a.state.data.patchCounts[id])id++;a.select(id);});assert(await page.evaluate(()=>window.metricViewer.highlight.children.some(o=>o.userData.selectedPatch===window.metricViewer.state.selected)));
  await page.click('#clear-selection');await page.check('#mesh-mode');await page.waitForFunction(()=>window.metricViewer.content.children.find(o=>o.userData.surface)?.userData.mode==='mesh');
  assert.equal(await page.evaluate(()=>window.metricViewer.content.children.find(o=>o.userData.kind==='unshown').geometry.attributes.position.count),583);
  const coverageShared=page.url();await page.goto(coverageShared,{waitUntil:'networkidle'});await wait('forest');assert(await page.isChecked('#fit-coverage'));assert(await page.isChecked('#mesh-mode'));
  await page.goto(base+'?scene=table&metric=max_error_m&coverage=1',{waitUntil:'networkidle'});await wait('table');assert(!await page.isChecked('#hide-overhead'));
  assert(await page.evaluate(()=>{const a=window.metricViewer,g=a.content.children.find(o=>o.userData.surface);return g.material.clippingPlanes.length===0&&g.geometry.attributes.position.array.some((z,k)=>k%3===2&&z>1.15);}));
  await page.screenshot({path:path.join(out,'table-fit-coverage.png'),fullPage:true});
  await page.goto(base+'assessment/?scene=hill',{waitUntil:'networkidle'});await wait('hill');assert.equal(new URL(page.url()).pathname,'/');
  await page.goto(base+'?scene=hill&color=max-error&view=overall&overlay=0&neighbors=0',{waitUntil:'networkidle'});await page.waitForFunction(()=>window.sceneFitCatalog?.state.ready);assert.equal(new URL(page.url()).pathname,'/scene-fits/');
  await page.goto(base+'assessment/details.html?scene=flat',{waitUntil:'networkidle'});await page.waitForFunction(()=>window.terrainAssessment?.state.ready&&document.getElementById('loading').hidden);assert.equal(await page.locator('[data-panel]').count(),8);
  await page.goto(base+'?scene=hill&metric=max_error_m',{waitUntil:'networkidle'});await wait('hill');
  await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.waitForFunction(()=>window.metricViewer.renderer.domElement.width>0);await page.screenshot({path:path.join(out,'mobile.png'),fullPage:true});
  await page.selectOption('#profile-select','q15-n15');await wait('hill','q15-n15');assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.screenshot({path:path.join(out,'mobile-q15.png'),fullPage:true});
  const report=await page.locator('#profile-report-link').getAttribute('href');assert.equal((await page.request.get(new URL(report,page.url()).href)).status(),200);
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  fs.writeFileSync(path.join(out,'browser-audit.json'),JSON.stringify({passed:true,scenes:17,profiles,metrics:11,records,colors,singleCanvas:true,noDetailsOrIframes:true,selectionAndShare:true,meshAndRawToggle:true,coverageToggle:true,forestCoverage:forest,profileShareAndCSV:true,profileKeepsCamera:true,patchSizeMatchesProfile:true,selectedPatchOutline:true,tabletopVisibleByDefault:true,legacyLinks:true,assessmentRedirect:true,advancedWorkbench:true,mobileOverflow:false,errors,failed},null,2)+'\n');
  console.log('PASS 2 profiles × 17 scenes × 11 metrics; correct numerical data, geometry, CSV, share links and mobile');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
