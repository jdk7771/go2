# 3DPlanner

本分支新增 M20 三维点云 virtual-wrench 机身轨迹规划与批量验证，原有 Go2 代码保持不变。

代码、复现方法和实验边界见 [`m20_3d_planner/README.md`](m20_3d_planner/README.md)。

本次固定参数盲测：17/17 条轨迹无穿透，14/17 达到 20 mm 安全间隙。当前结果属于
机身级离线几何验证，不能直接下发实机。
