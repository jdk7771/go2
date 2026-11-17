const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const assert=require('assert/strict'),fs=require('fs'),path=require('path');
const output=path.resolve(__dirname,'../outputs/visualization/scene-fits');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1480,height:1100}}),errors=[],failed=[],records=[];
  page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push({url:r.url(),status:r.status()});});
  const load=async id=>{await page.goto(`http://127.0.0.1:42482/?scene=${id}&color=max-error&view=overall&overlay=0&neighbors=0`,{waitUntil:'networkidle',timeout:60000});await page.waitForFunction(id=>window.sceneFitCatalog?.state.ready&&window.sceneFitCatalog.state.data.meta.id===id,id,{timeout:60000});};
  for(const id of ['combined','hill','flat_ramp']){
   await load(id);assert.equal(await page.inputValue('#color-mode'),'max-error');assert(await page.isVisible('#max-error-legend'));
   const result=await page.evaluate(()=>{
    const app=window.sceneFitCatalog,d=app.state.data,n=d.meta.seed_count,mesh=app.state.panels[1].content.children.find(o=>o.userData.surface),colors=mesh.geometry.attributes.color.array;
    const ids=new Set([0,n-1,d.meta.max_abs_error.worst_seed,d.meta.max_abs_error.worst_valid_seed]);for(let i=0;i<n;i+=Math.max(1,Math.floor(n/53)))ids.add(i);
    let residualDifference=0,colorDifference=0;
    const linear=x=>x<=.04045?x/12.92:Math.pow((x+.055)/1.055,2.4);
    for(const i of ids){if(!Number.isFinite(d.maxErrors[i]))continue;const o=i*14,c=d.seeds,u0=c[o],v0=c[o+1];let maximum=0;
     for(const j of d.neighbors.subarray(i*30,i*30+30)){const u=d.points[j*3]-u0,v=d.points[j*3+1]-v0,pred=c[o+2]+c[o+3]+c[o+4]*u+c[o+5]*v+c[o+6]*u*u+c[o+7]*u*v+c[o+8]*v*v;maximum=Math.max(maximum,Math.abs(pred-d.points[j*3+2]));}
     residualDifference=Math.max(residualDifference,Math.abs(maximum-d.maxErrors[i]));const g=linear(1-Math.min(1,Math.max(0,d.maxErrors[i]/d.meta.max_abs_error.color_scale_max_m)));
     colorDifference=Math.max(colorDifference,Math.abs(colors[i*3]-1),Math.abs(colors[i*3+1]-g),Math.abs(colors[i*3+2]-g));
    }
    return {scene:d.meta.id,centers:n,solved:d.meta.solved,missing:d.meta.insufficient,residualDifference,colorDifference,basicMaterial:mesh.material.isMeshBasicMaterial,white:app.maxErrorColor(0).getHexString(),red:app.maxErrorColor(d.meta.max_abs_error.color_scale_max_m).getHexString(),worst:d.meta.max_abs_error.worst_seed,max:d.meta.max_abs_error.all.max_m};
   });
   assert(result.residualDifference<1e-10);assert(result.colorDifference<1e-6);assert(result.basicMaterial);assert.equal(result.white,'ffffff');assert.equal(result.red,'ff0000');
   const response=await page.request.get(await page.getAttribute('#max-error-csv-link','href'));assert.equal(response.status(),200);const rows=(await response.text()).trimEnd().split(/\r?\n/).map(row=>row.split(','));assert.equal(rows.length-1,result.centers);assert.equal(rows[0][6],'training_max_abs_error_m');assert.equal(rows.slice(1).filter(r=>r[6]==='').length,result.missing);assert.equal(Number(rows[result.worst+1][6]),result.max);
   await page.locator('.workspace').screenshot({path:path.join(output,id,'max-error-overall.png')});
   await page.click('#largest-error');assert.equal(await page.inputValue('#seed-id'),String(result.worst));assert.match(await page.innerText('#seed-info'),/最大绝对训练残差/);records.push(result);
  }
  for(const [x,label]of [['-1.04','start'],['1.04','end'],['-2','flat']]){
   await page.click(`[data-ramp-x="${x}"]`);assert(await page.isChecked('#only-current'));assert(await page.isChecked('#neighbors'));assert.equal(await page.inputValue('#surface-mode'),'patch');
   const patch=await page.evaluate(()=>{const a=window.sceneFitCatalog,d=a.state.data,i=a.state.seed,c=d.seeds,o=i*14,m=a.state.panels[1].content.children.find(o=>o.userData.surface),pos=m.geometry.attributes.position.array;let difference=0;for(let j=0;j<pos.length;j+=3){const u=pos[j]-c[o],v=pos[j+1]-c[o+1],z=c[o+2]+c[o+3]+c[o+4]*u+c[o+5]*v+c[o+6]*u*u+c[o+7]*u*v+c[o+8]*v*v;difference=Math.max(difference,Math.abs(z-pos[j+2]));}return {seed:i,coefficientXX:c[o+6],vertices:pos.length/3,triangles:m.geometry.index.count/3,difference,url:location.href};});
   assert.equal(patch.vertices,81);assert(patch.triangles>0);assert(patch.difference<1e-6);assert(label==='flat'?Math.abs(patch.coefficientXX)<1e-9:Math.abs(patch.coefficientXX)>.4);
   assert((await page.locator('[data-role="quadratic-profile"]').count())>0);assert.equal(await page.locator('[data-role="analytic-reference"]').count(),1);assert.match(await page.innerText('#profile-title'),/局部剖面/);
   await page.locator('.workspace').screenshot({path:path.join(output,'flat_ramp',`quadratic-junction-${label}.png`)});await page.locator('.profile-card').screenshot({path:path.join(output,'flat_ramp',`quadratic-profile-${label}.png`)});
   await page.goto(patch.url,{waitUntil:'networkidle'});await page.waitForFunction(()=>window.sceneFitCatalog?.state.ready);assert.equal(await page.inputValue('#seed-id'),String(patch.seed));assert(await page.isChecked('#only-current'));
   records.push({probe:label,...patch});
  }
  await load('quick');const missing=await page.evaluate(()=>{const d=window.sceneFitCatalog.state.data;return Array.from(d.maxErrors).findIndex(e=>!Number.isFinite(e));});assert(missing>=0);await page.fill('#seed-id',String(missing));await page.locator('#seed-id').dispatchEvent('change');assert.match(await page.innerText('#seed-info'),/未求解/);assert.match(await page.innerText('#polynomial'),/误差留空/);
  await page.selectOption('#color-mode','error');assert(!(await page.isVisible('#max-error-legend')));assert.match(await page.innerText('#error-chart-title'),/RMSE/);await page.selectOption('#color-mode','max-error');
  await page.setViewportSize({width:390,height:844});await page.waitForFunction(()=>window.sceneFitCatalog.state.panels.every(p=>p.renderer.domElement.width===p.viewport.clientWidth));assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);await page.screenshot({path:path.join(output,'max-error-mobile.png'),fullPage:true});
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);
  const report={passed:true,port:42482,whiteToRed:true,allFunctionCSV:true,realQuadraticPatches:true,localProfileZoom:true,analyticReferenceOnly:true,missingNotZero:true,mobileOverflow:false,records,errors,failed};fs.writeFileSync(path.join(output,'max-error-browser-audit.json'),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
