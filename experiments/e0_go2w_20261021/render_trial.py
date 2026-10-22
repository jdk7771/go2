"""Render saved physically integrated states; replay is visualization only."""
import argparse
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image,ImageDraw
from model import load_recorded_model


def render(p,method):
    p=Path(p);m=load_recorded_model(p/'model.xml');d=mujoco.MjData(m)
    m.vis.global_.offwidth=960;m.vis.global_.offheight=540
    m.vis.headlight.ambient[:]=[.55,.55,.55]
    m.vis.headlight.diffuse[:]=[.8,.8,.8]
    m.geom_rgba[m.geom('floor').id]=[.55,.57,.6,1.]
    track=np.load(p/f'{method}_states.npz')
    renderer=mujoco.Renderer(m,height=540,width=960)
    camera=mujoco.MjvCamera();camera.lookat[:]=[.28,0.,.22]
    camera.distance=1.65;camera.azimuth=110;camera.elevation=-25
    process=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
        '-s','960x540','-r','20','-i','-','-c:v','libx264','-crf','23','-pix_fmt','yuv420p',
        str(p/f'{method}.mp4')],stdin=subprocess.PIPE)
    for t in np.arange(track['time'][0],track['time'][-1],.05):
        i=int(np.argmin(abs(track['time']-t)))
        d.qpos[:]=track['qpos'][i];mujoco.mj_forward(m,d)
        renderer.update_scene(d,camera=camera)
        img=Image.fromarray(renderer.render());draw=ImageDraw.Draw(img)
        draw.rectangle([0,0,960,35],fill='#ffffff')
        draw.text((12,10),f'Go2W E1 | integrated MuJoCo trajectory | t = {t:.2f} s',fill='#000000')
        process.stdin.write(np.asarray(img).tobytes())
    process.stdin.close();assert process.wait()==0
    frames=[]
    for t in [2.,7.9,9.,11.8,14.5,17.5]:
        i=int(np.argmin(abs(track['time']-t)));d.qpos[:]=track['qpos'][i]
        mujoco.mj_forward(m,d);renderer.update_scene(d,camera=camera)
        img=Image.fromarray(renderer.render()).resize((640,360))
        ImageDraw.Draw(img).text((12,12),f't = {track["time"][i]:.2f} s',fill='white')
        frames.append(img)
    montage=Image.new('RGB',(1920,720))
    for i,img in enumerate(frames):montage.paste(img,((i%3)*640,(i//3)*360))
    montage.save(p/f'{method}_montage.png');renderer.close()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--method',default='searched_shift')
    a=p.parse_args();render(a.directory,a.method)
