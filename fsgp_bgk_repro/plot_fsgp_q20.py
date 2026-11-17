"""Scientific summary figure from saved paired-test metrics."""
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from .compare_fsgp_q20 import OUT, METHODS


def main():
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':10})
    summary=json.loads((OUT/'summary.json').read_text())
    rows=summary['scene_summary'];conditions=summary['condition_summary']
    get=lambda scene,condition,method:next(r for r in rows if r['scene']==scene and r['condition']==condition and r['method']==method)
    colors=['#16804f','#477ac2','#985db0']
    scenes=['flat','ramp','flat_ramp','steps','steps-single','hill','corridor-040','corridor-060','corridor-080']
    labels=['平地','斜坡','平斜组合','多级台阶','单级台阶','丘陵','0.4 m通道','0.6 m通道','0.8 m通道']
    fig,axes=plt.subplots(2,2,figsize=(15,10),constrained_layout=True)
    for ax in axes.ravel():ax.spines[['top','right']].set_visible(False);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    x=np.arange(len(scenes));width=.24
    for j,(method,color) in enumerate(zip(METHODS,colors)):
        axes[0,0].bar(x+(j-1)*width,[get(s,'dense_low',method)['common']['rmse_m']*1000 for s in scenes],width,color=color,label=method)
        axes[0,1].bar(x+(j-1)*width,[get(s,'sparse_noisy',method)['coverage']*100 for s in scenes],width,color=color,label=method)
    axes[0,0].set_yscale('log');axes[0,0].set_ylabel('高度 RMSE / mm（对数轴）')
    axes[0,0].set_title('密集低噪声：共同支持位置的独立高程误差');axes[0,0].legend(loc='upper left')
    axes[0,1].set_title('稀疏高噪声：高程覆盖率');axes[0,1].set_ylabel('有预测的参考位置 / %');axes[0,1].set_ylim(0,108)
    for ax in axes[0]:ax.set_xticks(x,labels,rotation=30,ha='right')
    risks=['steps','steps-single','corridor-040','corridor-060','corridor-080'];x=np.arange(len(risks))
    for j,(method,color) in enumerate(zip(METHODS,colors)):
        axes[1,0].bar(x+(j-1)*width,[get(s,'sparse_noisy',method)['risk_common']['auc'] for s in risks],width,color=color)
    axes[1,0].axhline(.5,linestyle='--',color='#7e8b84',linewidth=1)
    axes[1,0].set_xticks(x,['多级台阶','单级台阶','0.4 m通道','0.6 m通道','0.8 m通道'],rotation=20,ha='right')
    axes[1,0].set_ylim(0,1.08);axes[1,0].set_ylabel('共同可评分集合 AUROC');axes[1,0].set_title('稀疏高噪声：危险排序，单级台阶中 FSGP 更好')
    x=np.arange(3);ax=axes[1,1]
    for j,(method,color) in enumerate(zip(METHODS,colors)):
        r=next(r for r in conditions if r['condition']=='dense_low' and r['method']==method)
        ax.bar(j-.18,r['geometry_ms'],.34,color=color)
        ax.bar(j+.18,r['native_ms'],.34,color=color,hatch='///',alpha=.6)
        ax.text(j-.18,r['geometry_ms']+28,f"{r['geometry_ms']:.0f}",ha='center',fontsize=9)
        ax.text(j+.18,r['native_ms']+28,f"{r['native_ms']:.0f}",ha='center',fontsize=9)
    ax.set_xticks(x,METHODS);ax.set_ylabel('CPU 中位耗时 / ms');ax.set_title('密集低噪声：实色为共同几何，斜线含各自评分')
    ax.text(.5,.82,'各自评分工作量不同\nQ20 包含跳变和净空，未含组合／机身评分',transform=ax.transAxes,ha='center',fontsize=10,color='#586b60')
    ax.set_ylim(0,2800)
    fig.suptitle('81 组配对输入：当前 20 cm 二次方案与本地 FSGP CPU 参考',fontsize=17)
    fig.supxlabel('9 场景 × 3 条件 × 3 种子；Q20 用完整局部邻域，GP 高程保留 M 点。非作者 GPU／论文基准成绩。',fontsize=10)
    for suffix in ('png','pdf'):fig.savefig(OUT/f'comparison.{suffix}',dpi=180)
    plt.close(fig)
    print('Saved comparison.png and comparison.pdf')


if __name__=='__main__':main()
