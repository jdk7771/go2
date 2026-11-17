"""Add complete local display patches to existing fits without refitting."""
import argparse
import json

from .scene_fit_catalog import ORDER, OUT, export_status_patches, publish_index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", nargs="+", choices=ORDER, default=ORDER)
    args = parser.parse_args()
    for name in args.scenes:
        meta = export_status_patches(OUT / name)
        print(json.dumps(dict(scene=name, **meta["status_patches"]), ensure_ascii=False), flush=True)
    publish_index()


if __name__ == "__main__":
    main()
