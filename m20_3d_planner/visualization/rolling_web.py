"""Offline Three.js viewer for actual growing execution prefixes."""
import base64
import json
import os
from pathlib import Path
import shutil

import numpy as np


def export(run, destination=None):
    run = Path(run).resolve()
    destination = Path(destination).resolve() if destination else run
    destination.mkdir(parents=True, exist_ok=True)
    assets = Path(__file__).with_name('assets')
    results = json.loads((run/'results.json').read_text())
    manifest = []
    data_dir = destination/'rolling_data'
    data_dir.mkdir(exist_ok=True)
    for result in results:
        name = result['name']
        history = json.loads((run/f'{name}.json').read_text())
        with np.load(run/f'{name}.npz') as data:
            payload = {key:np.round(data[key], 7).tolist() for key in ('executed', 'start', 'goal', 'half')}
        with np.load(run/f'{name}.cloud.npz') as data:
            payload['cloud'] = np.round(data['points'], 5).tolist()
        for record in history['records']:
            for attempt in record.get('attempts', []):
                # Per-solve states remain in the inspectable archive.
                attempt.pop('states', None)
        payload.update(history, label=result['label'], source=result['params'].get('source_repository'))
        target = data_dir/f'{name}.js'
        target.write_text('window.ROLLING_DATA='+json.dumps(payload, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+';\n')
        manifest.append(dict(name=name, label=result['label'], file=f'rolling_data/{name}.js'))
    imports = {}
    for module, filename in [('three', 'three.module.min.js'), ('three/addons/controls/OrbitControls.js', 'OrbitControls.js')]:
        imports[module] = 'data:text/javascript;base64,'+base64.b64encode((assets/'vendor'/filename).read_bytes()).decode()
    html = (assets/'rolling.html').read_text()
    html = html.replace('__IMPORT_MAP__', json.dumps(dict(imports=imports)))
    html = html.replace('__MANIFEST__', json.dumps(manifest, ensure_ascii=False))
    html = html.replace('__SCRIPT__', (assets/'rolling.js').read_text())
    (destination/'index.html').write_text(html)
    if (run/'REPORT.md').exists():
        shutil.copy2(run/'REPORT.md', destination/'rolling_REPORT.md')
    (destination/'rolling_viewer_source.json').write_text(json.dumps(dict(run=os.path.relpath(run, destination))))
    # A subsequent generic `view` export should preserve this selected format.
    print(f'Rolling viewer: {destination / "index.html"} ({len(results)} scenes)', flush=True)
    return destination/'index.html'
