"""Publish the compact white-to-red viewer without recalculating experiments."""
from pathlib import Path
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
    return output


if __name__=='__main__':
    print('Published',publish(),'at http://127.0.0.1:42482/')
