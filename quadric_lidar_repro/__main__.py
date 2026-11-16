import argparse
from dataclasses import replace
from pathlib import Path
from .core import Config


def main():
    parser=argparse.ArgumentParser(description="Quadric LiDAR odometry/mapping CPU reference")
    sub=parser.add_subparsers(dest="command",required=True)
    bench=sub.add_parser("benchmark")
    bench.add_argument("--data",type=Path,default=Path(__file__).parent/'data/kitti')
    bench.add_argument("--sequence",default='04')
    bench.add_argument("--output",type=Path,default=Path(__file__).parent/'outputs/kitti04')
    bench.add_argument("--limit",type=int)
    bench.add_argument("--literal",action="store_true",help="disable the explicit robust-loss/normal-consistency stability extensions")
    bench.add_argument("--fitting",choices=['schur','taubin'],default='schur')
    args=parser.parse_args()
    if args.command=='benchmark':
        from .experiment import benchmark
        config=Config(fitting=args.fitting)
        if args.literal: config=replace(config,robust_scale=0,association_normal_deg=180)
        benchmark(args.data,args.sequence,args.output,config,args.limit)


if __name__=='__main__':
    main()
