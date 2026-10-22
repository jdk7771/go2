"""Figures from recorded trials. No simulation or threshold changes."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap, BoundaryNorm

from model import ROOT, LEGS


def data(path):
    return np.genfromtxt(path,delimiter=',',names=True,dtype=None,encoding=None)


def main():
    out=ROOT/'published'/'figures';out.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.size':10,'axes.spines.right':False,'axes.spines.top':False})
    fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True,layout='constrained')
    for name,label,color in [('lift_naive','Fixed body reference','#c5663f'),('lift_shift','Shift body before lift','#267f86')]:
        a=data(ROOT/'formal_gravity'/f'{name}.csv')
        axes[0].plot(a['t'],a['FL_load'],label=label,color=color)
        loads=np.array([a[f'{leg}_load'] for leg in LEGS[1:]])
        axes[1].plot(a['t'],loads.min(axis=0),label=label,color=color)
        axes[2].plot(a['t'],np.rad2deg(a['tilt']),label=label,color=color)
    axes[0].set_ylabel('Swing-wheel load (N)');axes[0].axhline(1,color='gray',ls='--',lw=.8)
    axes[1].set_ylabel('Min other-wheel load (N)');axes[1].axhline(5,color='gray',ls='--',lw=.8)
    axes[2].set(ylabel='Body tilt (deg)',xlabel='Simulation time (s)')
    for ax in axes:ax.axvspan(4.7,5.3,alpha=.1,color='#267f86');ax.grid(alpha=.2)
    axes[0].legend();fig.suptitle('E0: same torque-limited controller, different body references')
    fig.savefig(out/'e0_support_comparison.png',dpi=160);plt.close(fig)

    case=ROOT/'e1_height_sweep'/'h0.086';a=data(case/'searched_shift.csv')
    plan=np.load(case/'searched_shift_plan.npz')
    fig,axes=plt.subplots(4,1,figsize=(11,10),sharex=True,layout='constrained')
    axes[0].plot(a['t'],a['FL_center_x'],label='Measured wheel center x',color='#267f86')
    axes[0].plot(plan['time'],plan['centers'][:,0,0],'--',label='Reference x',color='#c5663f')
    axes[0].plot(a['t'],a['FL_center_z'],label='Measured wheel center z',color='#50864d')
    axes[0].plot(plan['time'],plan['centers'][:,0,2],'--',label='Reference z',color='#8b8656')
    axes[0].set_ylabel('Wheel center (m)');axes[0].legend(ncol=2,fontsize=9)
    for leg in LEGS:axes[1].plot(a['t'],a[f'{leg}_load'],label=leg,lw=1)
    axes[1].set_ylabel('Normal load (N)');axes[1].legend(ncol=4,fontsize=9)
    axes[2].plot(a['t'],a['z'],label='Measured body height',color='#267f86')
    axes[2].plot(plan['time'],plan['body'][:,2],'--',label='Reference',color='#c5663f')
    axes[2].set_ylabel('Body height (m)');axes[2].legend(fontsize=9)
    axes[3].plot(a['t'],a['slip']*1000,color='#267f86');axes[3].set(ylabel='Load-weighted slip (mm/s)',xlabel='Simulation time (s)')
    for ax in axes:
        ax.axvspan(7,11.5,alpha=.07,color='#c5663f');ax.axvspan(12.5,16.5,alpha=.07,color='#267f86');ax.grid(alpha=.2)
    fig.suptitle('E1: 8.6 cm obstacle, one front-wheel crossing and resumed rolling\nDevelopment example; not a statistical success rate or RL evaluation')
    fig.savefig(out/'e1_h086_trajectory.png',dpi=160);plt.close(fig)

    rows=json.loads((ROOT/'e1_height_sweep'/'summary.json').read_text())
    heights=[.043,.086,.129,.172];methods=['fixed','searched_shift']
    grid=np.zeros((2,4));labels={0:'Planning rejected',1:'Execution gate failed',2:'Completed'}
    for r in rows:
        h=r['initial_obstacle']['size'][2]*2
        grid[methods.index(r['method']),heights.index(h)]=2 if r['diagnostic_success'] else 0 if r['terminal_reason']=='planning_rejected' else 1
    fig,ax=plt.subplots(figsize=(9,3.5),layout='constrained')
    ax.imshow(grid,cmap=ListedColormap(['#d9d9df','#d69a65','#267f86']),norm=BoundaryNorm([-.5,.5,1.5,2.5],3),aspect='auto')
    ax.set(xticks=range(4),xticklabels=['4.3','8.6','12.9','17.2'],yticks=range(2),yticklabels=['Fixed body','Body candidate search'],xlabel='Obstacle height (cm)')
    for i in range(2):
        for j in range(4):ax.text(j,i,labels[int(grid[i,j])],ha='center',va='center',fontsize=9,color='white' if grid[i,j]==2 else '#222222')
    ax.set_title('E1 height screen: one deterministic trial per cell\nSame landing target, controller and gates')
    fig.savefig(out/'e1_height_screen.png',dpi=160);plt.close(fig)

    p=ROOT/'body_reference_screen'/'summary.json'
    if p.exists():
        rows=json.loads(p.read_text());grid=np.zeros((2,3))
        for r in rows:
            i=[.129,.172].index(r['initial_obstacle']['size'][2]*2);j=[.03,.06,.09].index(r['body_raise_m'])
            grid[i,j]=2 if r['diagnostic_success'] else 0 if r['terminal_reason']=='planning_rejected' else 1
        fig,ax=plt.subplots(figsize=(8,3.5),layout='constrained')
        ax.imshow(grid,cmap=ListedColormap(['#d9d9df','#d69a65','#267f86']),norm=BoundaryNorm([-.5,.5,1.5,2.5],3),aspect='auto')
        ax.set(xticks=range(3),xticklabels=['3','6','9'],yticks=range(2),yticklabels=['12.9','17.2'],xlabel='Body raise during swing (cm)',ylabel='Obstacle height (cm)')
        for i in range(2):
            for j in range(3):ax.text(j,i,labels[int(grid[i,j])],ha='center',va='center',fontsize=9,color='white' if grid[i,j]==2 else '#222222')
        ax.set_title('Alternative body reference family: development screen\nOne deterministic trial per cell; selected after observing earlier failures')
        fig.savefig(out/'body_reference_screen.png',dpi=160);plt.close(fig)

    p=ROOT/'robustness_screen'/'summary.json'
    if p.exists():
        rows=json.loads(p.read_text());error=np.zeros((3,3));slip=np.zeros((3,3))
        for r in rows:
            i=[.3,.5,.8].index(r['actual_friction']);j=[-20.,0.,20.].index(r['push_force_y_n'])
            error[i,j]=r['landing_error_xy_m']*1000;slip[i,j]=r['slip_p95_m_s']*1000
        fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
        for ax,values,title in zip(axes,[error,slip],['Landing-center error (mm)','Slip p95 (mm/s)']):
            image=ax.imshow(values,cmap='YlGnBu',vmin=0,aspect='auto')
            for i in range(3):
                for j in range(3):ax.text(j,i,f'{values[i,j]:.1f}',ha='center',va='center',color='white' if values[i,j]>.6*values.max() else 'black')
            ax.set(xticks=range(3),xticklabels=['-20','0','+20'],yticks=range(3),yticklabels=['0.3','0.5','0.8'],xlabel='Lateral force for 0.15 s (N)',ylabel='Actual sliding friction',title=title)
            fig.colorbar(image,ax=ax,shrink=.8)
        fig.suptitle('8.6 cm obstacle: one deterministic development trial per cell\nController nominal friction remains 0.8')
        fig.savefig(out/'robustness_screen.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()
