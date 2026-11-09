"""One command entry point for experiments and visualization."""

import argparse
from importlib import import_module


def main():
    parser = argparse.ArgumentParser(description="M20 点云机身规划")
    parser.add_argument(
        "command",
        choices=("run", "view", "video", "replay", "compare"),
        help="运行实验 / 打开交互查看器 / 导出视频 / 补录历史实验迭代 / 对比筛选实验",
    )
    parser.add_argument("arguments", nargs=argparse.REMAINDER, help=argparse.SUPPRESS)
    args = parser.parse_args()
    modules = {
        "run": ".experiment",
        "view": ".visualization.web",
        "video": ".visualization.video",
        "replay": ".visualization.data",
        "compare": ".comparison",
    }
    import_module(modules[args.command], __package__).main(args.arguments)


if __name__ == "__main__":
    main()
