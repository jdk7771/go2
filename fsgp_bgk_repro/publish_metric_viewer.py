"""Publish the compact white-to-red viewer without recalculating experiments."""
from pathlib import Path
import json
import shutil

ROOT=Path(__file__).resolve().parent
WEB=ROOT/'outputs/visualization'


def publish():
    source=ROOT/'visualization/metrics';output=WEB/'metrics'
    output.mkdir(parents=True,exist_ok=True)
    for name in ('index.html','viewer.js','viewer.css'):
        shutil.copyfile(source/name,output/name)
    template=(source/'index.html').read_text()
    (WEB/'index.html').write_text(template.replace('<head>','<head><base href="./metrics/"><meta name="go2-metric-home" content="42482">',1))
    # Keep the original full workbench at its own URL. The old assessment entry
    # now leads straight to the single-view page, including bookmarked scenes.
    assessment=WEB/'assessment';assessment.mkdir(exist_ok=True)
    for name in ('index.html','details.html'):
        shutil.copyfile(ROOT/'visualization/assessment'/name,assessment/name)
    # Publish only complete, saved profiles. A baseline-only installation still
    # works; profile folders contain independent numerical results, not colors.
    catalog=assessment/'catalog.json'
    if catalog.exists():
        base=json.loads(catalog.read_text());ids=[s['id'] for s in base['scenes']]
        profiles=[dict(id='q20-n30',label='20×20 cm · 至少 30 点',path='',scene_ids=ids)]
        for folder in sorted((assessment/'profiles').glob('*')):
            path=folder/'catalog.json'
            if not path.exists():continue
            cases=json.loads(path.read_text())['scenes']
            if set(s['id'] for s in cases)!=set(ids):continue
            profiles.append(dict(id=folder.name,label=cases[0]['profile_label'],path=f'profiles/{folder.name}/',scene_ids=[s['id'] for s in cases]))
        (assessment/'profiles.json').write_text(json.dumps(dict(default_profile='q20-n30',profiles=profiles),ensure_ascii=False,indent=2)+'\n')
    return output


if __name__=='__main__':
    print('Published',publish(),'at http://127.0.0.1:42482/')
