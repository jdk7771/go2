
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

## 一句话总结


## 研究问题
	怎样让同一个腿足机器人策略，同时学会很多差异很大的运动技能，而且这些技能之间不要互相干扰。
	  梯度冲突，不同的任务梯度有 区别 甚至完全相反 这个可以用 CVAE那个act的想法去v做吗？会有用吗

## 核心方法

	 1. concurrent teacher-student framework
	 2.the sharing–separation tension in multi-reward RL asymmetrically, a dense       MoE actor composes shared behaviors
	
![[Pasted image 20260925101902.png|398]]
## 实验与结论


## 我的笔记

1.解决

## 疑问


## 对我的启发


## 相关论文
