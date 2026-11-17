// Measure saved-data loading and isolated windows on the agreed port.
// No Python algorithms are run by these browser operations.
const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const root=path.join(__dirname,'outputs/visualization/assessment'),output=path.join(root,'timing');
const args=process.argv.slice(2),option=(name,fallback)=>{const i=args.indexOf(name);return i<0?fallback:args[i+1];};
const repeats=Number(option('--repeats','3')),requested=option('--scenes','').split(',').filter(Boolean);
const views=['cloud','fit','mesh','error','clearance','jump','foot','body'];
const viewNames={cloud:'原始点云',fit:'独立二次曲面',mesh:'组合曲面',error:'误差与不确定性',clearance:'10–60 cm 净空',jump:'A/B/C 跳变',foot:'落足综合',body:'50×80 cm 机身'};
const median=a=>{a=[...a].sort((x,y)=>x-y);return a.length%2?a[(a.length-1)/2]:(a[a.length/2-1]+a[a.length/2])/2;};
const describe=a=>({median_ms:median(a),min_ms:Math.min(...a),max_ms:Math.max(...a),samples:a.length});
const csvCell=v=>`"${String(v).replaceAll('"','""')}"`;
function csv(file,fields,rows){fs.writeFileSync(path.join(output,file),[fields.map(csvCell).join(','),...rows.map(r=>fields.map(k=>csvCell(r[k])).join(','))].join('\n')+'\n');}
function save(records,environment,errors,failed){
 fs.mkdirSync(output,{recursive:true});
 const rows=[];
 for(const scene of records)for(let i=0;i<scene.runs.length;i++)for(const [view,values]of Object.entries(scene.runs[i].views))rows.push({scene:scene.id,run:i+1,view,view_name:viewNames[view],load_without_3d_ms:scene.runs[i].load_without_3d_ms,first_open_gpu_complete_ms:values.first_open_gpu_complete_ms,rebuild_gpu_complete_median_ms:median(values.rebuild_gpu_complete_samples_ms),canvas_width:values.canvas_width,canvas_height:values.canvas_height,webgl_renderer:values.webgl_renderer});
 csv('browser-runs.csv',['scene','run','view','view_name','load_without_3d_ms','first_open_gpu_complete_ms','rebuild_gpu_complete_median_ms','canvas_width','canvas_height','webgl_renderer'],rows);
 const summary=[];
 for(const scene of records)for(const view of views){const s=scene.views[view];summary.push({scene:scene.id,view,view_name:viewNames[view],load_without_3d_median_ms:scene.load_without_3d.median_ms,first_open_median_ms:s.first_open_gpu_complete.median_ms,first_open_min_ms:s.first_open_gpu_complete.min_ms,first_open_max_ms:s.first_open_gpu_complete.max_ms,rebuild_median_ms:s.rebuild_gpu_complete.median_ms,rebuild_min_ms:s.rebuild_gpu_complete.min_ms,rebuild_max_ms:s.rebuild_gpu_complete.max_ms});}
 csv('browser-summary.csv',['scene','view','view_name','load_without_3d_median_ms','first_open_median_ms','first_open_min_ms','first_open_max_ms','rebuild_median_ms','rebuild_min_ms','rebuild_max_ms'],summary);
 const report={experiment_date:'2026-11-17',timezone:'Asia/Shanghai',passed:errors.length===0&&failed.length===0,environment,scenes:records,errors,failed,notes:[
  'All windows are initially collapsed in this benchmark only. Actual published viewer source is unchanged. Each window is opened alone, in the listed order, with its default field and no raw overlay, slicing or selected-only filter.',
  'load_without_3d_ms uses browser performance.now from navigation to saved scene data ready. All scene binary assets are fetched and decoded; no WebGL window is created yet. Browser HTTP cache is disabled; OS/file caches are not flushed. This is not default-page latency with its three open windows.',
  'first_open_gpu_complete_ms starts when details.open is set, waits two animation frames for toggle/layout/resize, then calls gl.finish. It includes context creation, geometry/color preparation, driver submission, frame scheduling and completion of queued GPU work, not monitor scanout. It is deliberately broader than a pure rendering kernel.',
  'rebuild_gpu_complete_ms times the existing installOpen() call with only one visible panel, followed by gl.finish. Three samples per navigation, pooled across three fresh navigations by default. Includes geometry/color rebuilding, disposal, highlight and renderer work; excludes network, Python compute and PNG encoding.',
  'Desktop interactive rendering may differ from headless Chrome. The actual WebGL renderer is recorded; SwiftShader values are software rendering and must not be presented as hardware GPU timings.',
  'All browser images use precomputed metrics. A/B/C selector changes do not refit a point cloud.',
 ]};
 fs.writeFileSync(path.join(output,'browser.tmp'),JSON.stringify(report,null,2)+'\n');fs.renameSync(path.join(output,'browser.tmp'),path.join(output,'browser.json'));
}
(async()=>{
 assert(Number.isInteger(repeats)&&repeats>0);
 const catalog=JSON.parse(fs.readFileSync(path.join(root,'catalog.json'))),scenes=catalog.scenes.filter(c=>!requested.length||requested.includes(c.id));
 assert(scenes.length>0);for(const id of requested)assert(scenes.some(c=>c.id===id),'unknown scene '+id);
 const html=fs.readFileSync(path.join(root,'index.html'),'utf8').replace(/(<details\s+data-panel="[^"]+")\s+open/g,'$1');
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 const context=await browser.newContext({viewport:{width:1440,height:1000},deviceScaleFactor:1}),records=[],errors=[],failed=[];
 const environment={chrome_version:browser.version(),headless:true,viewport:{width:1440,height:1000},device_scale_factor:1,http_cache_disabled:true,view_order:views,webgl_renderers:[]};
 await context.route('http://127.0.0.1:42482/assessment/**',route=>{const url=new URL(route.request().url());if(url.pathname==='/assessment/'||url.pathname==='/assessment/index.html')return route.fulfill({status:200,contentType:'text/html',body:html});return route.continue();});
 try{
  for(const scene of scenes){
   const runs=[];
   for(let rep=0;rep<repeats;rep++){
    const page=await context.newPage();page.setDefaultTimeout(120000);
    page.on('pageerror',e=>errors.push({scene:scene.id,run:rep+1,message:e.message}));page.on('response',r=>{if(r.status()>=400)failed.push({url:r.url(),status:r.status()});});
    const cdp=await context.newCDPSession(page);await cdp.send('Network.enable');await cdp.send('Network.setCacheDisabled',{cacheDisabled:true});
    await page.goto('http://127.0.0.1:42482/assessment/?scene='+scene.id,{waitUntil:'domcontentloaded'});
    await page.waitForFunction(id=>window.terrainAssessment?.state.ready&&window.terrainAssessment.state.data.meta.id===id&&document.getElementById('loading').hidden,scene.id);
    const load=await page.evaluate(()=>{if(window.terrainAssessment.state.panels.size)throw Error('benchmark initial panels must be collapsed');return performance.now();});
    const run={load_without_3d_ms:load,views:{}};
    for(const type of views){
     const result=await page.evaluate(async type=>{
      const a=window.terrainAssessment,details=document.querySelector(`[data-panel="${type}"]`);
      const start=performance.now();details.open=true;
      await new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)));
      const p=a.state.panels.get(type);if(!p||!details.open)throw Error('missing panel '+type);
      const gl=p.renderer.getContext();gl.finish();const first=performance.now()-start;
      if(gl.isContextLost())throw Error('context lost '+type);
      const ext=gl.getExtension('WEBGL_debug_renderer_info'),renderer=ext?gl.getParameter(ext.UNMASKED_RENDERER_WEBGL):gl.getParameter(gl.RENDERER);
      const surface=p.content.children.find(o=>o.userData.surface),meta=a.state.data.meta;
      if(type!=='cloud'&&surface.geometry.index.count/3!==(type==='mesh'?meta.triangles:meta.patch_triangles))throw Error('incorrect graph geometry '+type);
      const samples=[];for(let j=0;j<3;j++){const begin=performance.now();a.installOpen();gl.finish();samples.push(performance.now()-begin);}
      const result={first_open_gpu_complete_ms:first,rebuild_gpu_complete_samples_ms:samples,canvas_width:p.renderer.domElement.width,canvas_height:p.renderer.domElement.height,webgl_renderer:renderer};
      details.open=false;await new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)));return result;
     },type);
     run.views[type]=result;if(!environment.webgl_renderers.includes(result.webgl_renderer))environment.webgl_renderers.push(result.webgl_renderer);
    }
    runs.push(run);await page.close();console.log(JSON.stringify({scene:scene.id,run:rep+1,load_ms:Math.round(load),first_open_ms:Object.fromEntries(views.map(v=>[v,Math.round(run.views[v].first_open_gpu_complete_ms)]))}));
   }
   records.push({id:scene.id,name:scene.name,repeats,raw_points:scene.raw_points,centers:scene.centers,valid:scene.valid,
     load_without_3d:describe(runs.map(r=>r.load_without_3d_ms)),runs,
     views:Object.fromEntries(views.map(view=>[view,{first_open_gpu_complete:describe(runs.map(r=>r.views[view].first_open_gpu_complete_ms)),rebuild_gpu_complete:describe(runs.flatMap(r=>r.views[view].rebuild_gpu_complete_samples_ms))}]))});
   save(records,environment,errors,failed);
  }
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);console.log('Saved browser timing:',records.length,'scenes');
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
