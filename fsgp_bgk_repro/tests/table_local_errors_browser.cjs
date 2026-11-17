// Browser integration checks for the local error viewer; existing 42482 service
// is reused, or restored if it is no longer running.
const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const assert=require('assert/strict'),fs=require('fs'),path=require('path'),{spawn}=require('child_process');
const root=path.resolve(__dirname,'..'),output=path.join(root,'outputs/table_local_errors');
const base='http://127.0.0.1:42482/table-local/';
async function server(){
 try{await fetch(base,{signal:AbortSignal.timeout(5000)});return;}catch(e){
  const log=fs.openSync(path.join(root,'outputs/42482-http.log'),'a');
  const child=spawn('/usr/bin/python3',['-m','http.server','42482','--bind','127.0.0.1','--directory',path.join(root,'outputs/visualization')],{detached:true,stdio:['ignore',log,log]});child.unref();fs.closeSync(log);
  for(let i=0;i<20;i++){await new Promise(r=>setTimeout(r,100));try{await fetch(base);return;}catch(e){}}
  throw Error('Local viewer could not be restored');
 }
}
(async()=>{
 fs.mkdirSync(output,{recursive:true});await server();
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1450,height:1100}}),errors=[],failed=[];
  page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push({url:r.url(),status:r.status()});});
  assert.equal((await page.goto(base+'?v=3',{waitUntil:'networkidle'})).status(),200);
  await page.waitForFunction(()=>window.tableLocal?.state.ready&&window.tableLocal.state.errorsReady);
  assert.equal(await page.locator('#error-plot .curve-error-point').count(),1964);
  assert.equal(await page.locator('#error-plot [data-valid="false"]').count(),9);
  assert.equal(await page.locator('#neighbor-error-plot rect').count(),30);
  assert.match(await page.locator('#selected-range-info').innerText(),/含种子自身/);
  await page.locator('#curve-errors').screenshot({path:path.join(output,'browser-overview.png')});
  for(const [filter,count]of [['valid',1955],['rejected',9],['component:0',1373],['solved',1964]]){
   await page.selectOption('#error-filter',filter);assert.equal(await page.locator('#error-plot .curve-error-point').count(),count);
  }
  await page.selectOption('#error-metric','truth_neighbor_rmse_m');assert.match(await page.locator('#error-definition').innerText(),/未参与拟合/);
  const expectedWorst=await page.evaluate(()=>window.tableLocal.state.errors.records.filter(r=>r.solved).reduce((a,b)=>a.truth_neighbor_rmse_m>b.truth_neighbor_rmse_m?a:b).seed);
  await page.click('#worst-error');assert.equal(await page.inputValue('#seed-id'),String(expectedWorst));assert.match(await page.locator('#selected-error-info').innerText(),/未通过检查/);
  await page.selectOption('#error-metric','fit_rmse_m');await page.locator('#error-plot [data-seed="1057"]').dispatchEvent('click');
  assert.equal(await page.inputValue('#seed-id'),'1057');assert.equal(await page.locator('#neighbor-error-plot rect').count(),30);
  const noFit=await page.evaluate(()=>window.tableLocal.state.errors.records.find(r=>!r.solved).seed);
  await page.fill('#seed-id',String(noFit));await page.locator('#seed-id').dispatchEvent('change');
  assert.equal(await page.locator('#neighbor-error-plot rect').count(),0);assert.match(await page.locator('#selected-error-info').innerText(),/未求解/);
  await page.fill('#seed-id','263');await page.locator('#seed-id').dispatchEvent('change');
  const renderChecks=[];
  for(const metric of ['fit_rmse_m','truth_neighbor_rmse_m']){
   await page.selectOption('#patch-error-color',metric);assert(await page.locator('#error-color-scale').isVisible());
   const rendered=await page.evaluate(()=>{
    const p=window.tableLocal.state.panels.find(p=>p.type==='patch'),mesh=p.content.children.find(o=>o.userData.surface),rgb=mesh.geometry.attributes.color.array,unique=new Set();
    for(let i=0;i<rgb.length;i+=3)unique.add([rgb[i],rgb[i+1],rgb[i+2]].map(v=>v.toFixed(4)).join(','));
    return {vertexColors:mesh.material.vertexColors,count:mesh.geometry.attributes.color.count,unique:unique.size,triangles:p.renderer.info.render.triangles};
   });assert(rendered.vertexColors&&rendered.count>40000&&rendered.unique>100&&rendered.triangles>50000,JSON.stringify(rendered));renderChecks.push({metric,...rendered});
   await page.locator('.panels').screenshot({path:path.join(output,`browser-${metric}.png`)});
  }
  await page.check('#back-color');assert.equal(await page.inputValue('#patch-error-color'),'none');await page.uncheck('#back-color');
  await page.selectOption('#patch-error-color','fit_rmse_m');
  await page.locator('.triangle-explanation').scrollIntoViewIfNeeded();await page.waitForFunction(()=>document.querySelector('.triangle-explanation img').complete&&document.querySelector('.triangle-explanation img').naturalWidth>0);
  assert.match(await page.locator('#triangle-example-values').innerText(),/2.76 cm/);await page.locator('.triangle-explanation').screenshot({path:path.join(output,'browser-triangle.png')});
  await page.locator('#curve-errors details summary').click();
  for(const img of await page.locator('#curve-errors details img').all()){await img.scrollIntoViewIfNeeded();await img.evaluate(im=>im.loading='eager');}
  await page.waitForFunction(()=>Array.from(document.querySelectorAll('#curve-errors details img')).every(im=>im.complete&&im.naturalWidth>0));
  await page.locator('#curve-errors details summary').click();
  for(const asset of ['data.json','per-curve.csv','REPORT.md','audit.json','per-curve-errors.png','error-map.png','triangle-example.png'])assert.equal((await page.request.get(base+'errors/'+asset)).status(),200);
  const csv=await (await page.request.get(base+'errors/per-curve.csv')).text();assert.equal(csv.trim().split('\n').length,2126);
  await page.setViewportSize({width:390,height:844});await page.waitForFunction(()=>window.tableLocal.state.panels.every(p=>p.renderer.domElement.width===p.viewport.clientWidth));
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
  await page.locator('#curve-errors').screenshot({path:path.join(output,'browser-mobile.png')});assert.equal(await page.locator('#error-plot .curve-error-point').count(),1964);
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  const result={passed:true,actualFits:1964,validFits:1955,rejectedFitsIncluded:9,allCurvesPlotted:true,filters:true,pointClickLinksSeed:true,neighborResiduals:30,unknownIsNotZero:true,threeDimensionalErrorColors:renderChecks,triangleExample:true,csvRows:2125,mobileOverflow:false,errors,failed};
  fs.writeFileSync(path.join(output,'browser-audit.json'),JSON.stringify(result,null,2));console.log(JSON.stringify(result));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
