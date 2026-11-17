const {chromium}=require('/home/jiang/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright-core');
const assert=require('assert/strict'),fs=require('fs'),path=require('path');
const output=path.resolve(__dirname,'../outputs/visualization/scene-fits');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1480,height:1100}}),errors=[],failed=[],records=[];
  page.on('pageerror',e=>errors.push(e.message));page.on('response',r=>{if(r.status()>=400)failed.push({url:r.url(),status:r.status()});});
  await page.goto('http://127.0.0.1:42482/?scene=hill&color=fit-status&view=overall&overlay=1&neighbors=0',{waitUntil:'networkidle'});
  const wait=async id=>page.waitForFunction(id=>{const a=window.sceneFitCatalog;return a?.state.ready&&a.state.data.meta.id===id&&a.state.data.statusGeometry&&!document.getElementById('status-legend').hidden;},id,{timeout:60000});
  await wait('hill');const ids=await page.locator('#scene-select option').evaluateAll(options=>options.map(o=>o.value));
  for(const id of ids){
   if((await page.inputValue('#scene-select'))!==id){await page.selectOption('#scene-select',id);await wait(id);}
   const camera=await page.evaluate(()=>window.sceneFitCatalog.camera.position.toArray());
   const modes={};for(const mode of ['fit-status','assembly-status']){
    await page.selectOption('#color-mode',mode);await wait(id);assert(await page.isDisabled('#surface-mode'));assert.equal(await page.inputValue('#surface-mode'),'patch');
    const result=await page.evaluate(()=>{
     const a=window.sceneFitCatalog,d=a.state.data,p=a.state.panels[1],mesh=p.content.children.find(o=>o.userData.surface),markers=p.content.children.find(o=>o.userData.statusCenters),color=mesh.geometry.attributes.color.array,mc=markers.geometry.attributes.color.array,owners=d.statusGeometry.owners,mode=document.getElementById('color-mode').value;
     const linear=x=>x<=.04045?x/12.92:Math.pow((x+.055)/1.055,2.4),blue=[86,142,182].map(x=>linear(x/255)),expected=i=>mode==='fit-status'?!!d.seeds[i*14+12]:!!d.assembled[i];let difference=0,blueCenters=0;
     for(let i=0;i<owners.length;i++){const rgb=expected(owners[i])?blue:[1,1,1];for(let k=0;k<3;k++)difference=Math.max(difference,Math.abs(color[i*3+k]-rgb[k]));}
     for(let i=0;i<d.meta.seed_count;i++){if(expected(i))blueCenters++;const rgb=expected(i)?blue:[1,1,1];for(let k=0;k<3;k++)difference=Math.max(difference,Math.abs(mc[i*3+k]-rgb[k]));}
     return {mode,scene:d.meta.id,centers:d.meta.seed_count,valid:d.meta.valid,assembled:d.assembledCount,blueCenters,difference,allValidOwners:new Set(owners).size,sameGeometry:mesh.geometry.attributes.position.array===d.statusGeometry.positions,meshTriangles:mesh.geometry.index.count/3,basic:mesh.material.isMeshBasicMaterial};
    });assert(result.difference<1e-6);assert(result.sameGeometry);assert(result.basic);assert.equal(result.allValidOwners,result.valid);assert.equal(result.blueCenters,mode==='fit-status'?result.valid:result.assembled);modes[mode]=result;
    if(['hill','combined','steps'].includes(id))await page.locator('.workspace').screenshot({path:path.join(output,id,`${mode}.png`)});
   }
   assert.deepEqual(await page.evaluate(()=>window.sceneFitCatalog.camera.position.toArray()),camera);assert.equal(modes['fit-status'].meshTriangles,modes['assembly-status'].meshTriangles);records.push(modes);console.log(JSON.stringify({scene:id,valid:modes['fit-status'].valid,assembled:modes['assembly-status'].assembled}));
  }
  await page.selectOption('#scene-select','hill');await wait('hill');
  const unused=await page.evaluate(()=>{const d=window.sceneFitCatalog.state.data,owners=d.statusGeometry.owners,faces=d.statusGeometry.indices;for(const vertex of faces){const i=owners[vertex];if(!d.assembled[i]&&d.seeds[i*14+12])return i;}return -1;});assert(unused>=0);
  await page.evaluate(i=>window.sceneFitCatalog.chooseSeed(i),unused);await page.click('#focus-seed');assert(await page.isChecked('#only-current'));assert.match(await page.innerText('#seed-info'),/拟合有效，但没有参与最终三角形/);
  let first=await page.evaluate(()=>Array.from(window.sceneFitCatalog.state.panels[1].content.children.find(o=>o.userData.surface).geometry.attributes.color.array.slice(0,3)));assert.deepEqual(first,[1,1,1]);
  await page.click('[data-status-mode="fit-status"]');first=await page.evaluate(()=>Array.from(window.sceneFitCatalog.state.panels[1].content.children.find(o=>o.userData.surface).geometry.attributes.color.array.slice(0,3)));assert(first[0]<1&&first[2]<1);await page.click('[data-status-mode="assembly-status"]');
  const share=page.url();await page.goto(share,{waitUntil:'networkidle'});await wait('hill');assert.equal(await page.inputValue('#color-mode'),'assembly-status');assert(await page.isChecked('#only-current'));assert.equal(await page.inputValue('#seed-id'),String(unused));
  await page.uncheck('#only-current');await page.click('#show-final-mesh');assert.equal(await page.inputValue('#color-mode'),'uniform');assert(!(await page.isDisabled('#surface-mode')));assert.equal(await page.inputValue('#surface-mode'),'mesh');assert.equal(await page.evaluate(()=>window.sceneFitCatalog.state.panels[1].content.children.find(o=>o.userData.surface).geometry.index.count/3),16499);
  await page.selectOption('#color-mode','max-error');assert(await page.isVisible('#max-error-legend'));await page.selectOption('#color-mode','assembly-status');await wait('hill');
  await page.setViewportSize({width:390,height:844});await page.waitForFunction(()=>window.sceneFitCatalog.state.panels.every(p=>p.renderer.domElement.width===p.viewport.clientWidth));assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);await page.screenshot({path:path.join(output,'status-mobile.png'),fullPage:true});
  assert.deepEqual(errors,[]);assert.deepEqual(failed,[]);const report={passed:true,sceneCount:ids.length,allValidPatchesIncluded:true,noRefitting:true,actualMeshMembership:true,sameGeometryAndCamera:true,invalidCentersWhite:true,whiteUnusedFunction:unused,shareable:true,restoreActualMesh:true,mobileOverflow:false,records,errors,failed};fs.writeFileSync(path.join(output,'status-browser-audit.json'),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({passed:true,scenes:ids.length,whiteUnusedFunction:unused}));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
