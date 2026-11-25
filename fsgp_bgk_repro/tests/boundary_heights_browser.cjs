const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const base='http://127.0.0.1:42482/',root=path.resolve(__dirname,'../outputs/visualization/convex-assessment/boundary-heights');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 const errors=[],failed=[],records=[],profiles=['qcircle-adaptive','qcircle-r07','q15-n15','q20-n30'],scenes=['flat-corridor-platforms','steps'];
 const ready=(page,scene,profile)=>page.waitForFunction(({scene,profile})=>{const s=window.metricViewer?.state;return s?.ready&&s.renderedScene===scene&&s.renderedProfile===profile&&document.getElementById('loading').hidden;},{scene,profile},{timeout:90000});
 try{
  for(const profile of profiles)for(const scene of scenes){
   const page=await browser.newPage({viewport:{width:1440,height:1100}});page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push([r.status(),r.url()]);});
   await page.goto(base+`?scene=${scene}&profile=${profile}&coverage=1&boundary=1`,{waitUntil:'networkidle'});await ready(page,scene,profile);
   assert(await page.isVisible('#boundary-option'));assert(await page.isChecked('#boundary-heights'));assert.equal(await page.locator('canvas').count(),1);
   const record=await page.evaluate(async()=>{
    const a=window.metricViewer,d=a.state.data,b=d.boundary,line=a.content.children.find(o=>o.userData.boundary);
    if(line.geometry.attributes.position.count!==b.meta.line_segments*2)throw Error('Boundary lines absent');
    if(b.meta.new_fits!==0||d.meta.valid!==b.meta.valid_functions)throw Error('Fitting changed');
    const hash=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',d.models.buffer)),b=>b.toString(16).padStart(2,'0')).join('');if(hash!==b.meta.original_files_sha256['models.bin'])throw Error('Wrong original models');
    const colors=line.geometry.attributes.color.array;for(const k of [0,Math.floor(b.values.length/2),b.values.length-1]){const expected=a.colorFor(b.values[k],.8).toArray();for(let j=0;j<3;j++)if(Math.abs(colors[k*3+j]-expected[j])>1e-6)throw Error('Wrong height color');}
    return {scene:d.meta.id,profile:a.state.profile,samples:b.meta.boundary_samples,pairs:b.meta.region_pairs,max:b.meta.maximum_delta_m};
   });
   const fields=await page.locator('#metric-select option').evaluateAll(o=>o.map(x=>x.value));assert.equal(fields.length,6);
   for(const field of fields){
    await page.selectOption('#metric-select',field);
    await page.evaluate(field=>{const a=window.metricViewer,d=a.state.data,spec=a.fields[field];let n=0,sum=0,max=-Infinity;for(let i=0;i<d.meta.centers;i++)if(a.value(i,'valid')&&Number.isFinite(a.value(i,field))){const v=a.value(i,field);n++;sum+=v;max=Math.max(max,v);}const format=v=>(v*spec.factor).toFixed(spec.digits)+' '+spec.unit;if(document.getElementById('mean-value').textContent!==format(sum/n)||document.getElementById('max-value').textContent!==format(max))throw Error('Original metric changed');if(!a.content.children.some(o=>o.userData.boundary))throw Error('Metric switch hid boundaries');},field);
   }
   await page.check('#fit-coverage');const camera=await page.evaluate(()=>window.metricViewer.camera.position.toArray());
   if(profile==='qcircle-adaptive'){
    if(scene==='flat-corridor-platforms'){
     const pixel=await page.evaluate(()=>{const a=window.metricViewer,b=a.state.data.boundary;let i=b.meta.records.findIndex((r,k)=>k%8===0&&r.delta_m>.79&&r.xy[0]>0&&r.xy[1]<-1.5);if(i<0)throw Error('Missing high platform ruler');const r=b.meta.records[i],p=a.camera.position.clone().set(r.xy[0],r.xy[1],(r.left_z_m+r.right_z_m)/2).project(a.camera),rect=a.renderer.domElement.getBoundingClientRect();return {x:rect.left+(p.x+1)*rect.width/2,y:rect.top+(1-p.y)*rect.height/2};});
     await page.mouse.click(pixel.x,pixel.y);assert.match(await page.locator('#boundary-info').innerText(),/已选边界：函数 #\d+ ↔ #\d+，Δz 80.00 cm/);
    }
    await page.screenshot({path:path.join(root,`${scene}-3d.png`),fullPage:true});await page.click('[data-angle="top"]');await page.screenshot({path:path.join(root,`${scene}-top.png`),fullPage:true});
   }
   await page.uncheck('#boundary-heights');await page.waitForFunction(()=>!window.metricViewer.content.children.some(o=>o.userData.boundary));assert(!await page.isVisible('#boundary-info'));assert.equal(new URL(page.url()).searchParams.get('boundary'),'0');
   await page.check('#boundary-heights');await page.waitForFunction(()=>window.metricViewer.content.children.some(o=>o.userData.boundary));
   if(profile!=='qcircle-adaptive')assert.deepEqual(await page.evaluate(()=>window.metricViewer.camera.position.toArray()),camera);
   assert.equal((await page.request.get(await page.locator('#boundary-info a').evaluate(a=>a.href))).status(),200);
   records.push(record);console.log('Checked boundary heights',scene,profile,6,'original metrics');await page.close();
  }
  const page=await browser.newPage({viewport:{width:390,height:844}});page.on('pageerror',e=>errors.push(e.message));await page.goto(base+'?scene=flat-corridor-platforms&profile=qcircle-adaptive&coverage=1&boundary=1',{waitUntil:'networkidle'});await ready(page,'flat-corridor-platforms','qcircle-adaptive');await page.reload({waitUntil:'networkidle'});await ready(page,'flat-corridor-platforms','qcircle-adaptive');assert(await page.isChecked('#boundary-heights'));assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.screenshot({path:path.join(root,'mobile.png'),fullPage:true});
  await page.selectOption('#scene-select','forest');await ready(page,'forest','qcircle-adaptive');assert(!await page.isVisible('#boundary-option'));assert(!await page.isVisible('#boundary-info'));assert(!await page.evaluate(()=>window.metricViewer.content.children.some(o=>o.userData.boundary)));
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  fs.writeFileSync(path.join(root,'browser-audit.json'),JSON.stringify({passed:true,records,scene_settings:records.length,metric_views:records.length*6,base_metrics_unchanged:true,correct_line_colors:true,selection:true,toggle:true,share:true,mobile:true,scene_scope:true,errors,failed},null,2)+'\n');console.log('PASS: 8 boundary maps, 48 original metric views, line selection, toggles, sharing and mobile');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
