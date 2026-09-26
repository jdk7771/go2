
---
title: "MoE-Loco: Mixture of Experts for Multitask Locomotion"
authors:
  - Runhan Huang
  - Shaoting Zhu
  - Yilun Du
  - Hang Zhao
year: 2025
status: reading
topic:
  - legged-locomotion
  - mixture-of-experts
  - reinforcement-learning
arxiv: "2503.08564"
tags:
  - paper
  - leg-dog
---

# MoE-Loco

> [!info] 论文
> [[MoE-Loco.pdf|打开 PDF]] · [arXiv](https://arxiv.org/abs/2503.08564) · 状态：阅读中

## 项目文件

- [[12周学习计划]]
- [[学习日报与周报]]

## 阅读进度

- [ ] 第一遍：理解问题、动机和核心思路
- [ ] 第二遍：看清方法、训练设置和实验
- [ ] 整理结论、疑问和可复用的想法

## 一句话总结


## 研究问题
	怎样让同一个腿足机器人策略，同时学会很多差异很大的运动技能，而且这些技能之间不要互相干扰。
	  梯度冲突，不同的任务梯度有 区别 甚至完全相反 这个可以用 CVAE那个act的想法去v做吗？会有用吗

## 核心方法

	soft/dense MoE   blind locomotion
	
  ## 第一类：Proprioception ptp_t

	这是机器人真实部署时可以获得的本体信息。
	
	主要包括：
	
	- IMU 的 projected gravity；
	- base angular velocity；
	- joint position；
	- joint velocity；
	- 上一步 action。
	
	即：pt​=[gravity,ω,q,q˙​,at−1​]

  ## 第二类：Explicit privileged state ete_t

	这是仿真训练期间容易获得，但真实机器人上不一定可靠获得的信息。
	
	包括：et=[vbase,μ]e_t=[ v_{\text{base}},\mu ]
	
	也就是：- base linear velocity；- 地面摩擦系数。
	
 ## 第三类：Implicit privileged state iti_t

	主要是：
	不同 robot link 的 contact force。
	
	也就是机器人各个部位和环境接触的信息。
	
	这些东西首先经过 encoder：
	
	zt=Enc(it)z_t=Enc(i_t)
	
	把高维 contact information 编成一个低维 latent。
	
	于是形成：
	
	lt​=[Enc(it​),et​,pt​]

Command $C_t= (V,g)$  g决定是两足还是四足

有些信息在部署的时候没有，在部署的时候估计出来就好了
 ## 有Estimator的原因 
	 训练的时候都知道，但是部署的时候不知道
	PAS的使用
## 实验与结论


## 我的笔记

1.解决

## 疑问


## 对我的启发


## 相关论文
