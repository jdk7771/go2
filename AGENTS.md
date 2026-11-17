# 本地点云实验约定

- 所有点云场景、局部二次曲面拟合测试和可视化统一使用 `http://127.0.0.1:42482/`。用户要求今后沿用此端口，不另外启用其他可视化端口。
- 主页面是全场景拟合查看器；新场景加入 `fsgp_bgk_repro/scene_fit_catalog.py` 的场景目录，生成结果后在同一页面选择。
- 0.4 / 0.6 / 0.8 米通道为独立小场景，只包含通道和地面；丘陵、森林、桌子拼接大场景保留一个，不把它复制到各通道场景。
- 网页目录为 `fsgp_bgk_repro/outputs/visualization`，服务入口为 `fsgp_bgk_repro/serve_visualization.py`，用户服务名为 `go2-pointcloud-viewer.service`。
- 保留已有场景和历史实验的独立链接。统一首页之前的页面存放在 `outputs/visualization/legacy-results.html`。
- 区分原始点数、拟合中心数量、实际求解数量、通过检查的函数数量和显示面片数量。训练残差不能称作独立测试精度；局部二次拟合不是 FSGP 论文的 GP 方法。
