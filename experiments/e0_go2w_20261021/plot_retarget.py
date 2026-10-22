"""Plot the contact-point ablation and measured-base retargeting diagnosis."""
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from model import ROOT
from make_figures import data


def main():
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    colors=['#53758c','#c5663f','#267f86']
    for k,(label,color) in enumerate(zip(['Original reference','Measured contact points','Measured-base FL IK'],colors)):
        errors=[]
        for h in [.086,.129]:
            source=ROOT/'contact_point_screen'/f'h{h:.3f}_measured{k}' if k<2 else ROOT/'retarget_screen'/f'h{h:.3f}'
            row=json.loads((source/'searched_shift.json').read_text())
            errors.append(row['landing_error_xy_m']*1000)
        axes[0,0].bar(np.arange(2)+(k-1)*.24,errors,.24,label=label,color=color)
    axes[0,0].axhline(30,color='black',ls='--',lw=1,label='Original landing gate')
    axes[0,0].set(xticks=[0,1],xticklabels=['8.6 cm','12.9 cm'],ylabel='Landing XY error (mm)',title='One development trial per cell')
    axes[0,0].legend(fontsize=8)
    for source,label,color in [(ROOT/'e1_height_sweep'/'h0.129','Original reference',colors[0]),
                               (ROOT/'retarget_validation'/'nominal','Measured-base FL IK',colors[2])]:
        a=data(source/'searched_shift.csv');p=np.load(source/'searched_shift_plan.npz')
        mask=(a['t']>=4.5)&(a['t']<=16.5)
        axes[0,1].plot(a['t'][mask],1000*(a['y'][mask]-np.interp(a['t'][mask],p['time'],p['body'][:,1])),label=label,color=color)
        axes[1,0].plot(a['t'][mask],1000*(a['FL_center_y'][mask]-np.interp(a['t'][mask],p['time'],p['centers'][:,0,1])),label=label,color=color)
    axes[0,1].set(ylabel='Body lateral error (mm)',title='12.9 cm obstacle');axes[0,1].legend(fontsize=8)
    axes[1,0].set(ylabel='FL center lateral error (mm)',xlabel='Simulation time (s)')
    states=np.load(ROOT/'retarget_validation'/'nominal'/'searched_shift_states.npz')
    axes[1,1].plot(states['retarget_time'],states['retarget_error']*1000,color=colors[2])
    axes[1,1].axhline(1,color='black',ls='--',lw=1)
    axes[1,1].set(ylabel='Online IK residual (mm)',xlabel='Simulation time (s)',title='Limitation: online targets can exceed reach')
    for ax in axes.flat:ax.grid(alpha=.2)
    for ax in [axes[0,1],axes[1,0]]:ax.axvline(12.5,color='gray',ls=':',lw=1)
    fig.suptitle('Contact geometry alone did not resolve the landing failure\nRetargeting passed execution gates, but does not certify reference feasibility')
    fig.savefig(ROOT/'published'/'figures'/'retarget_comparison.png',dpi=160)


if __name__=='__main__':main()
