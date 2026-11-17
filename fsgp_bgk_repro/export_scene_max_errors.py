"""Export max absolute residuals for existing scene fits, without refitting."""
import argparse
import json

from .scene_fit_catalog import ORDER, OUT, export_max_errors, publish_index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", nargs="+", choices=ORDER, default=ORDER)
    args = parser.parse_args()
    for name in args.scenes:
        meta = export_max_errors(OUT / name)
        print(json.dumps(dict(scene=name, functions=meta["seed_count"],
                              maximum=meta["max_abs_error"]), ensure_ascii=False), flush=True)
    publish_index()


if __name__ == "__main__":
    main()
