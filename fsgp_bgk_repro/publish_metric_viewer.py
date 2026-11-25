"""Publish the convex-domain quadratic viewer on port 42482."""
import json
import hashlib
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parent
WEB=ROOT/'outputs/visualization'


def publish():
    data=WEB/'convex-assessment'
    if not (data/'catalog.json').exists() or not (data/'profiles.json').exists():
        raise RuntimeError('Run python -m fsgp_bgk_repro.convex_surface_assessment before publishing')
    profiles=json.loads((data/'profiles.json').read_text())['profiles']
    if not profiles:raise RuntimeError('No fit setting is complete for all current scenes')
    gap_options={}
    boundary_options={}
    scenes=[s['id'] for s in json.loads((data/'catalog.json').read_text())['scenes']]
    for profile in profiles:
        name=profile['id']
        for scene in profile.get('scene_ids',scenes):
            boundary_path=f'boundary-heights/{name}/{scene}/'
            boundary_meta=data/boundary_path/'data.json'
            if boundary_meta.exists():
                meta=json.loads(boundary_meta.read_text());original=data/name/scene
                if meta['scene']==scene and meta['profile']==name and all(
                        (original/file).exists() and hashlib.sha256((original/file).read_bytes()).hexdigest()==digest
                        for file,digest in meta['original_files_sha256'].items()):
                    boundary_options.setdefault(scene,{})[name]=boundary_path
            paths=[f'small-gaps/{name}/{scene}/']
            if scene=='hill':paths.append(f'hill-small-gaps/{name}/')
            for path in paths:
                meta_path=data/path/'data.json'
                if not meta_path.exists():continue
                meta=json.loads(meta_path.read_text());original=data/name/scene
                if meta['scene']!=scene or meta['profile']!=name:continue
                if all((original/file).exists() and hashlib.sha256((original/file).read_bytes()).hexdigest()==digest
                       for file,digest in meta['original_files_sha256'].items()):
                    gap_options.setdefault(scene,{})[name]=path;break
    (data/'gap-options.json').write_text(json.dumps(gap_options,ensure_ascii=False,indent=2)+'\n')
    (data/'boundary-options.json').write_text(json.dumps(boundary_options,ensure_ascii=False,indent=2)+'\n')
    planner_source=ROOT/'visualization/surface-planner';planner_output=WEB/'surface-planner';planner_output.mkdir(exist_ok=True)
    for name in ('index.html','viewer.js','viewer.css'):
        if (planner_source/name).exists():shutil.copyfile(planner_source/name,planner_output/name)
    workflow=ROOT.parent/'planning-workflow.md'
    if workflow.exists():shutil.copyfile(workflow,planner_output/'workflow.md')
    source=ROOT/'visualization/metrics';output=WEB/'metrics';output.mkdir(exist_ok=True)
    for name in ('index.html','viewer.js','viewer.css'):shutil.copyfile(source/name,output/name)
    (output/'methods.json').write_text(json.dumps(dict(methods=[dict(id='quadratic',label='局部二次曲面')]),ensure_ascii=False,indent=2)+'\n')
    template=(source/'index.html').read_text()
    (WEB/'index.html').write_text(template.replace('<head>','<head><base href="./metrics/"><meta name="go2-metric-home" content="42482">',1))
    return output


if __name__=='__main__':print('Published',publish(),'at http://127.0.0.1:42482/')
