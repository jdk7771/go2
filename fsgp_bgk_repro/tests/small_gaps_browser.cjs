const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const base='http://127.0.0.1:42482/',out=path.resolve(__dirname,'../outputs/visualization/convex-assessment/small-gaps');
(async()=>{
 const launch=()=>chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});let browser;
 const summary=JSON.parse(fs.readFileSync(path.join(out,'summary.json'))),selected=process.argv.slice(2),scope=summary.records.filter(r=>!selected.length||selected.includes(r.scene)),records=[],errors=[],failed=[];
 const ready=(page,scene,profile)=>page.waitForFunction(({scene,profile})=>{const s=window.metricViewer?.state;return s?.ready&&s.renderedScene===scene&&s.renderedProfile===profile&&document.getElementById('loading').hidden;},{scene,profile},{timeout:90000});
 const rendered=(page,enabled)=>page.waitForFunction(enabled=>{const a=window.metricViewer,d=a.state.data,mesh=a.content.children.find(o=>o.userData.gapFill);if(enabled&&!d.gap.faces)return false;return enabled?(d.gap.faces.length?!!mesh:!mesh):!mesh;},enabled);
 try{
  for(const record of scope){
   // Isolate software WebGL resources across heavy snow-scene settings.
   browser=await launch();
   const page=await browser.newPage({viewport:{width:1440,height:1100}});
   page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push([r.status(),r.url()]);});
   const {scene,profile}=record;await page.goto(base+`?scene=${scene}&profile=${profile}&coverage=1&gaps=1`,{waitUntil:'networkidle'});await ready(page,scene,profile);
   assert(await page.isVisible('#small-gap-option'));assert(await page.isChecked('#small-gap-fill'));assert.equal(await page.locator('canvas').count(),1);
   const fields=await page.locator('#metric-select option').evaluateAll(o=>o.map(x=>x.value));assert.equal(fields.length,6);
   const checked=await page.evaluate(async()=>{
    const a=window.metricViewer,d=a.state.data,g=d.gap,original=a.content.children.find(o=>o.userData.surface&&!o.userData.gapFill),fill=a.content.children.find(o=>o.userData.gapFill);
    if(!g||g.meta.scene!==d.meta.id||g.meta.profile!==a.state.profile)throw Error('Wrong scene overlay');
    if(g.meta.threshold_cm2!==9||!g.meta.strictly_less||g.meta.new_fits!==0||!g.meta.geometry.boundary_height_check)throw Error('Inconsistent completion policy');
    if(d.meta.valid!==g.meta.valid_functions||d.meta.raw_points!==g.meta.raw_points)throw Error('Fit counts changed');
    const hash=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',d.models.buffer)),b=>b.toString(16).padStart(2,'0')).join('');
    if(hash!==g.meta.original_files_sha256['models.bin'])throw Error('Base models changed');
    if(original.geometry.index.count!==3*d.meta.patch_triangles)throw Error('Original hull display changed');
    if(g.meta.overlay_triangles?(fill?.geometry.index.count!==3*g.meta.overlay_triangles):!!fill)throw Error('Incorrect completion geometry');
    for(const owner of g.owners)if(!a.value(owner,'valid'))throw Error('Completion uses invalid function');
    return {scene:d.meta.id,profile:a.state.profile,filled:g.meta.filled_gaps,area:g.meta.filled_area_m2,unfilled:g.meta.unfilled_small_gaps,large:g.meta.large_gaps_retained,empty:!g.meta.overlay_triangles};
   });
   const metrics=[];
   for(const field of fields){
    await page.selectOption('#metric-select',field);
    metrics.push(await page.evaluate(field=>{
     const a=window.metricViewer,d=a.state.data,spec=a.fields[field];let n=0,sum=0,max=-Infinity;
     for(let i=0;i<d.meta.centers;i++)if(a.value(i,'valid')&&Number.isFinite(a.value(i,field))){const v=a.value(i,field);n++;sum+=v;max=Math.max(max,v);}
     const format=v=>(v*spec.factor).toFixed(spec.digits)+' '+spec.unit;
     if(n!==d.meta.valid||document.getElementById('mean-value').textContent!==format(sum/n)||document.getElementById('max-value').textContent!==format(max))throw Error('Metric statistics changed');
     for(const mesh of a.content.children.filter(o=>o.userData.surface)){
      const owners=mesh.userData.owners,colors=mesh.geometry.attributes.color.array;
      for(const k of [0,Math.floor(owners.length/2),owners.length-1]){const expected=a.colorFor(a.value(owners[k],field),spec.scale).toArray();for(let j=0;j<3;j++)if(Math.abs(colors[3*k+j]-expected[j])>1e-6)throw Error('Wrong original/parent metric color');}
     }
     a.renderer.getContext().finish();if(a.renderer.getContext().isContextLost())throw Error('WebGL context lost');
     return {field,known:n,mean:sum/n,max};
    },field));
   }
   await page.check('#fit-coverage');const camera=await page.evaluate(()=>window.metricViewer.camera.position.toArray());
   if(profile==='qcircle-adaptive')await page.screenshot({path:path.join(out,`${scene}-coverage.png`),fullPage:true});
   await page.uncheck('#small-gap-fill');await rendered(page,false);assert.equal(new URL(page.url()).searchParams.get('gaps'),'0');
   if(profile==='qcircle-adaptive'&&['forest','flat-corridor-platforms'].includes(scene))await page.screenshot({path:path.join(out,`${scene}-before.png`),fullPage:true});
   await page.check('#small-gap-fill');await rendered(page,true);assert.deepEqual(await page.evaluate(()=>window.metricViewer.camera.position.toArray()),camera);
   records.push({...checked,metrics});console.log('Checked',scene,profile,fields.length,'metrics',checked.empty?'(zero small gaps)':'');await browser.close();browser=null;
  }
  browser=await launch();
  const page=await browser.newPage({viewport:{width:1440,height:1100}});page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push([r.status(),r.url()]);});await page.goto(base+'?scene=forest&profile=qcircle-adaptive&coverage=1&gaps=0',{waitUntil:'networkidle'});await ready(page,'forest','qcircle-adaptive');assert(!await page.isChecked('#small-gap-fill'));
  await page.selectOption('#scene-select','steps');await ready(page,'steps','qcircle-adaptive');assert(!await page.isChecked('#small-gap-fill'));
  await page.check('#small-gap-fill');await rendered(page,true);const share=page.url();await page.reload({waitUntil:'networkidle'});await ready(page,'steps','qcircle-adaptive');assert(await page.isChecked('#small-gap-fill'));
  await page.setViewportSize({width:390,height:844});assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1));await page.screenshot({path:path.join(out,'mobile.png'),fullPage:true});
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);assert.equal(records.length,scope.length);
  fs.writeFileSync(path.join(out,selected.length?(selected.includes('four-platforms')?'four-platforms-browser-audit.json':selected.some(s=>s.startsWith('platform-underpass-'))?'platform-underpass-browser-audit.json':'low-table-browser-audit.json'):'browser-audit.json'),JSON.stringify({passed:true,scene_settings:records.length,metric_views:records.length*6,records,all_current_scenes:!selected.length,base_models_unchanged:true,zero_gap_settings:true,toggle:true,share,mobile:true,errors,failed},null,2)+'\n');
  console.log(`PASS: ${records.length} scene/settings, ${records.length*6} metric views; completion, empty overlays, toggle, sharing and mobile`);
 }finally{await browser?.close();}
})().catch(e=>{console.error(e);process.exit(1)});
