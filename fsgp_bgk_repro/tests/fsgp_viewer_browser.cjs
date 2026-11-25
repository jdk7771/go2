const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const base='http://127.0.0.1:42482/',out=path.resolve(__dirname,'../outputs/visualization/fsgp');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 const page=await browser.newPage({viewport:{width:1440,height:1100}}),errors=[],failed=[],records=[];
 page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push({url:r.url(),status:r.status()});});
 const wait=(id,method)=>page.waitForFunction(({id,method})=>{const s=window.metricViewer?.state;return s?.ready&&s.renderedScene===id&&s.renderedMethod===method&&document.getElementById('loading').hidden;},{id,method},{timeout:60000});
 try{
  // Preserve the user's exact legacy link, including its blue coverage mode.
  await page.goto(base+'?scene=steps-single&metric=body_cost&angle=3d&mesh=0&points=0&hide=0&coverage=1&profile=q15-n15',{waitUntil:'networkidle'});await wait('steps-single','quadratic');
  assert.equal(await page.locator('#metric-select').inputValue(),'body_cost');
  assert(await page.isChecked('#fit-coverage'));
  assert.equal(await page.locator('#profile-select').inputValue(),'q15-n15');
  const before=await page.evaluate(()=>window.metricViewer.camera.position.toArray());
  await page.selectOption('#method-select','fsgp-500');await wait('steps-single','fsgp-500');
  assert(!await page.isChecked('#fit-coverage'));assert.equal(await page.locator('#metric-select').inputValue(),'gp_cost');
  assert.deepEqual(await page.evaluate(()=>window.metricViewer.camera.position.toArray()),before);
  assert(await page.locator('.profile-control').isHidden());assert(await page.locator('#mesh-option').isHidden());
  assert.match(await page.locator('#known-value').innerText(),/100.0%/);assert.match(await page.locator('#method-note').innerText(),/未做 BGK/);
  assert.equal(await page.locator('canvas').count(),1);
  await page.selectOption('#method-select','quadratic');await wait('steps-single','quadratic');assert.equal(await page.locator('#metric-select').inputValue(),'body_cost');
  await page.selectOption('#method-select','fsgp-500');await wait('steps-single','fsgp-500');
  const scenes=await page.locator('#scene-select option').evaluateAll(options=>options.map(o=>o.value));assert.equal(scenes.length,17);
  for(const method of ['fsgp-125','fsgp-500']){
   await page.selectOption('#method-select',method);await wait(await page.locator('#scene-select').inputValue(),method);
   for(const id of scenes){
    await page.selectOption('#scene-select',id);await wait(id,method);
    const fields=await page.locator('#metric-select option').evaluateAll(options=>options.map(o=>o.value));assert.equal(fields.length,6);assert(!fields.includes('body_cost'));
    for(const field of fields){
     await page.selectOption('#metric-select',field);
     const record=await page.evaluate(field=>{
      const a=window.metricViewer,s=a.state,d=s.data,g=a.content.children.find(o=>o.userData.surface),f=a.fields[field];
      let sum=0,count=0,max=-Infinity;for(let i=0;i<d.meta.centers;i++)if(a.value(i,'valid')&&Number.isFinite(a.value(i,field))){sum+=a.value(i,field);max=Math.max(max,a.value(i,field));count++;}
      const formatted=v=>Number.isFinite(v)?(v*f.factor).toFixed(f.digits)+(f.unit?' '+f.unit:''):'未知';
      if(document.getElementById('mean-value').textContent!==formatted(count?sum/count:NaN))throw Error('incorrect mean');
      if(document.getElementById('max-value').textContent!==formatted(count?max:NaN))throw Error('incorrect maximum');
      if(document.getElementById('known-value').textContent!==(count/d.meta.centers*100).toFixed(1)+'%')throw Error('incorrect grid coverage denominator');
      if(g.geometry.index.count!==d.meta.triangles*3||g.userData.mode!=='mesh')throw Error('incorrect GP geometry');
      const colors=g.geometry.attributes.color.array,vertices=g.geometry.attributes.position.array;
      for(const i of [0,Math.floor(d.meta.centers/2),d.meta.centers-1]){
       const expected=a.colorFor(a.value(i,field),f.scale).toArray();for(let j=0;j<3;j++)if(Math.abs(colors[i*3+j]-expected[j])>1e-6)throw Error('incorrect GP vertex color');
       const z=a.value(i,a.value(i,'valid')?'height':'z');if(Math.abs(vertices[i*3+2]-z)>2e-5)throw Error('not the GP height');
      }
      for(const i of d.mesh)if(!a.value(i,'valid'))throw Error('mesh bridges unknown query');
      const unknown=a.content.children.find(o=>o.userData.kind==='unfitted');if((unknown?.geometry.attributes.position.count||0)!==d.meta.centers-d.meta.valid)throw Error('incorrect unknown marker count');
      const unshown=a.content.children.find(o=>o.userData.kind==='unshown');if((unshown?.geometry.attributes.position.count||0)!==d.coverage.validNoPatch)throw Error('incorrect unshown markers');
      return {scene:d.meta.id,method:s.method,field,mean:count?sum/count:null,max:count?max:null,known:count,queries:d.meta.centers};
     },field);records.push(record);
    }
    const csv=await page.locator('#csv-link').getAttribute('href');assert.equal((await page.request.get(csv)).status(),200);
    console.log('Checked',method,id);
   }
  }
  await page.selectOption('#scene-select','steps-single');await wait('steps-single','fsgp-500');await page.selectOption('#metric-select','gp_cost');
  await page.screenshot({path:path.join(out,'steps-single-fsgp-500.png'),fullPage:true});
  await page.click('[data-angle=top]');const box=await page.locator('#viewport').boundingBox();await page.locator('#viewport').click({position:{x:box.width/2,y:box.height/2}});await page.waitForFunction(()=>window.metricViewer.state.selected!==null);
  assert.match(await page.locator('#selection-info').innerText(),/GP 查询 #\d+.*预测有效.*FSGP 可通行性代价/);
  const share=page.url();await page.goto(share,{waitUntil:'networkidle'});await wait('steps-single','fsgp-500');assert.match(await page.locator('#selection-info').innerText(),/GP 查询 #\d+/);
  await page.check('#cloud-overlay');await page.waitForFunction(()=>window.metricViewer.content.children.some(o=>o.isPoints&&o.geometry.attributes.position.count===window.metricViewer.state.data.meta.raw_points));
  await page.uncheck('#cloud-overlay');await page.check('#fit-coverage');assert(await page.locator('#metric-legend').isHidden());assert(await page.locator('#coverage-legend').isVisible());
  assert(await page.evaluate(()=>window.metricViewer.content.children.some(o=>o.userData.coverageEdges&&o.visible)));
  await page.selectOption('#method-select','quadratic');await wait('steps-single','quadratic');assert.equal(await page.locator('#profile-select').inputValue(),'q15-n15');assert.equal(await page.locator('#metric-select').inputValue(),'jump_b_m');
  assert(await page.locator('.profile-control').isVisible());assert(await page.locator('#mesh-option').isVisible());
  await page.goto(base+'?scene=steps&method=fsgp-125&metric=gp_cost&coverage=0',{waitUntil:'networkidle'});await wait('steps','fsgp-125');await page.screenshot({path:path.join(out,'steps-fsgp-125.png'),fullPage:true});
  await page.goto(base+'?scene=forest&method=fsgp-500&metric=gp_cost&coverage=0',{waitUntil:'networkidle'});await wait('forest','fsgp-500');await page.screenshot({path:path.join(out,'forest-fsgp-500.png'),fullPage:true});
  await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.screenshot({path:path.join(out,'mobile-fsgp.png'),fullPage:true});
  const doc=await page.locator('#details-link').getAttribute('href');assert.equal((await page.request.get(doc)).status(),200);
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  fs.writeFileSync(path.join(out,'browser-audit.json'),JSON.stringify({passed:true,scenes:17,methods:['fsgp-125','fsgp-500'],metrics:6,legacyUserLink:true,methodKeepsCamera:true,switchClearsBlueCoverage:true,rawCloud:true,selectionAndShare:true,csv:true,mobileOverflow:false,records,errors,failed},null,2)+'\n');
  console.log('PASS FSGP: 2 methods × 17 scenes × 6 metrics, GP heights/colors/statistics, original user link and mobile');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
