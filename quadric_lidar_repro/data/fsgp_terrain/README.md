# FSGP-BGK官方示例点云

来源：https://github.com/ZJU-FAST-Lab/FSGP_BGK

固定提交：`6e5976957734b90f367f8460a6eea6a0fc18c1cc`。
`upstream_inventory.json`保存本次递归核查到的公开场景文件，`manifest.json`记录每个文件的
精确源链接、大小、Git blob SHA1和SHA256。ROS1和ROS2目录中的对应文件相同，本次采用ROS2版本。

本目录补充forest、desert、mountain、snow、vocano；hill和quick复用
`fsgp_bgk_repro/data/`中的同一上游文件。文件内的法向仅用于几何参照，分割和拟合只读取XYZ。
forest没有附带法向，参照法向由完整干净点云估计；所有曲率参照都是估计值。

这些是作者公开的7份示例，不能视作论文五类地形各500组的原始批量评测集。
本次仓库发布没有提供Road/Indoor/Ruin原始批测集及通行代价真值。
