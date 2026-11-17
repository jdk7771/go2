const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const out=path.join(__dirname,'../outputs/visualization/fsgp-q20'),base='http://127.0.0.1:42482/';
const protocol=JSON.parse(fs.readFileSync(path.join(out,'protocol.json')));
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1540,height:1100}}),page=await context.newPage(),errors=[],failed=[],checked=[];
 page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push({url:r.url(),status:r.status()});});
 try{
  await page.goto(base+'fsgp-q20/',{waitUntil:'networkidle'});
  await page.waitForFunction(()=>window.fsgpQ20?.state.ready&&document.getElementById('loading').hidden);
  assert.equal(await page.locator('#scene option').count(),9);
  assert.equal(await page.locator('#findings li').count(),4);
  for(const scene of protocol.scenes)for(const condition of Object.keys(protocol.conditions))for(const seed of protocol.sensor_seeds){
   const info=await page.evaluate(async ({scene,condition,seed})=>{
    for(const [key,value]of Object.entries({scene,condition,seed}))document.getElementById(key).value=String(value);
    await window.fsgpQ20.load();const d=window.fsgpQ20.state.data;
    if(!window.fsgpQ20.state.ready||d.id!==`${scene}_${condition}_${seed}`)throw Error('case did not load');
    document.getElementById('field').value='error';window.fsgpQ20.draw();
    let checkedUnknown=0,checkedExcluded=0;
    for(const method of ['Q20','FSGP-125','FSGP-500']){
     const canvas=document.getElementById('map-'+method),ctx=canvas.getContext('2d'),m=d.methods[method];
     if(!canvas.width||!document.querySelector('#profile svg path'))throw Error('empty map/profile');
     for(const [index,kind]of [[d.truth.height_mask.findIndex((v,i)=>v&&!m.known[i]),'unknown'],[d.truth.height_mask.findIndex(v=>!v),'excluded']]){
      if(index<0)continue;
      const actual=Array.from(ctx.getImageData(index%25*20+10,(24-Math.floor(index/25))*20+10,1,1).data).slice(0,3),expected=kind==='unknown'?[124,139,130]:[222,228,223];
      if(String(actual)!==String(expected))throw Error(`${kind} painted incorrectly ${method} ${d.id}`);
      if(kind==='unknown')checkedUnknown++;else checkedExcluded++;
     }
    }
    return {id:d.id,unknownChecks:checkedUnknown,excludedChecks:checkedExcluded,common:d.rows.find(r=>r.method==='Q20').common.n};
   },{scene,condition,seed});
   checked.push(info);
   for(const field of ['height','normal','sigma','risk'])await page.selectOption('#field',field);
  }
  await page.selectOption('#scene','corridor-040');await page.selectOption('#condition','sparse_noisy');await page.selectOption('#seed','1101');
  await page.waitForFunction(()=>window.fsgpQ20.state.ready&&window.fsgpQ20.state.data.id==='corridor-040_sparse_noisy_1101');
  await page.check('#controls');assert.equal(await page.locator('#metrics tr').count(),6);
  await page.selectOption('#field','error');await page.screenshot({path:path.join(out,'corridor-sparse.png'),fullPage:true});
  await page.goto(base+'fsgp-q20/?scene=hill&condition=dense_low&seed=1101',{waitUntil:'networkidle'});
  await page.screenshot({path:path.join(out,'hill-dense.png'),fullPage:true});
  await page.goto(base+'fsgp-q20/?scene=steps-single&condition=sparse_noisy&seed=1101&field=risk',{waitUntil:'networkidle'});
  assert.match(await page.locator('#scale').innerText(),/各方法原生代价分别使用自己的色标/);
  await page.screenshot({path:path.join(out,'step-sparse.png'),fullPage:true});
  await page.goto(base+'fsgp-q20/?scene=invalid&condition=invalid&seed=oops',{waitUntil:'networkidle'});
  assert.equal(await page.inputValue('#scene'),'corridor-040');
  assert.equal(await page.inputValue('#condition'),'dense_low');
  assert.equal(await page.inputValue('#seed'),'1101');
  await page.setViewportSize({width:390,height:844});
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1),'mobile horizontal overflow');
  await page.screenshot({path:path.join(out,'mobile.png'),fullPage:true});
  for(const file of ['protocol.json','summary.json','summary.csv','per-run.csv','audit.json','numerical-audit.json','report.md','cases/corridor-040_dense_low_1101.npz']){
   const response=await context.request.get(base+'fsgp-q20/'+file);assert.equal(response.status(),200,file);
   assert((await response.body()).length>0,file+' empty');
  }
  await page.goto(base,{waitUntil:'domcontentloaded'});
  const homeLink=page.locator('a[href="../fsgp-q20/"]');assert.equal(await homeLink.count(),1);
  assert.equal(await homeLink.evaluate(a=>a.href),base+'fsgp-q20/');
  await page.goto(base+'assessment/',{waitUntil:'domcontentloaded'});
  assert.equal(await page.locator('a[href="../fsgp-q20/"]').count(),1);
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  fs.writeFileSync(path.join(out,'browser-audit.json'),JSON.stringify({passed:true,cases:checked.length,fields_per_case:5,checked,
   errors,failed,unknown_and_excluded_colors_verified:true,controls:true,invalid_url_fallback:true,mobile_overflow:false,
   home_and_assessment_links:true,downloads:true,renderer:'Chrome headless canvas 2D; no GPU speed claim',chrome_version:browser.version()},null,2)+'\n');
  console.log('Passed',checked.length,'cases × 5 fields; controls, unknown colors, mobile, links and downloads');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
