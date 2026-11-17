"""Static scientific figures from saved assessment measurements."""
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from .assessment_export import OUT, NAMES


def main():
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':10})
    rows=json.loads((OUT/'controlled-noise.json').read_text())['cases']
    fig,axes=plt.subplots(2,2,figsize=(11,7),constrained_layout=True)
    for ax,(kind,name) in zip(axes.flat,[('flat','平面'),('quadratic','二次背景'),('kink','连续折角（真实边界已知）'),('cubic','三次背景：模型不匹配')]):
        selected=[r for r in rows if r['background']==kind and r['true_jump_m']==.12]
        for method,color in [('a','#c95e43'),('b','#188765')]:
            ax.plot([r['observation_noise_m']*1000 for r in selected],
                    [r[method+'_rmse_m']*1000 for r in selected],'-o',c=color,label=method.upper())
        ax.set(title=name,xlabel='添加的高度噪声标准差 / mm',ylabel='台阶估计 RMSE / mm',xticks=[0,3,15],ylim=(0,85))
        ax.grid(alpha=.2);ax.legend(title='方法')
    fig.suptitle('真实跳变 120 mm；每个条件 100 次独立噪声试验\nA：单侧中心外推；B：双侧同中点。中心高度也使用含噪观测。',fontsize=13)
    fig.savefig(OUT/'controlled-jump-errors.png',dpi=170);plt.close(fig)
    rows=json.loads((OUT/'repeat-timing.json').read_text())['scenes']
    fig,ax=plt.subplots(figsize=(11,max(4.8,len(rows)*.6)),constrained_layout=True)
    groups=[(['select_centers_ms','window_query_ms','fit_and_covariance_ms','assembly_ms'],'选中心／搜索／主拟合及协方差／组合','#426b8a'),
            (['display_patch_geometry_ms'],'面片显示几何','#8aa38d'),
            (['three_jump_methods_ms'],'三种跳变（含双侧重拟合）','#d39b48'),
            (['clearance_ms'],'10–60 cm 净空','#6faaa4'),
            (['body_50x80_ms','local_costs_ms'],'局部代价／机身评估','#9a759f')]
    left=[0.]*len(rows)
    for keys,name,color in groups:
        values=[sum(r['medians_ms'][key] for key in keys)/1000 for r in rows]
        ax.barh([NAMES[r['scene']] for r in rows],values,left=left,label=name,color=color)
        left=[a+b for a,b in zip(left,values)]
    ax.set(xlabel='CPU 秒；各阶段三次中位数',title='完整新增流程的时间组成（不含文件读写与浏览器渲染）')
    ax.legend(loc='upper center',bbox_to_anchor=(.5,-.16),ncol=2,fontsize=9);ax.grid(axis='x',alpha=.15)
    fig.savefig(OUT/'pipeline-timing.png',dpi=170);plt.close(fig)
    print('Wrote assessment figures')


if __name__=='__main__':main()
