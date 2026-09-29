#!/usr/bin/env python3
"""Upload a checkpoint artifact folder to a private Hugging Face model repo.

Requires HF_TOKEN in the environment. The token is never read from a file or
printed by this script.
"""

import argparse
import os
from pathlib import Path

from huggingface_hub import HfApi


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path, help="Local folder to upload")
    parser.add_argument("--repo-suffix", default="m20-isaaclab-rslrl-checkpoints")
    parser.add_argument("--path-in-repo", default=".")
    args = parser.parse_args()

    if not os.environ.get("HF_TOKEN"):
        raise SystemExit("HF_TOKEN is required in the environment.")
    if not args.folder.is_dir():
        raise SystemExit(f"Folder not found: {args.folder}")

    api = HfApi()
    username = api.whoami()["name"]
    repo_id = f"{username}/{args.repo_suffix}"
    api.create_repo(repo_id=repo_id, repo_type="model", private=True, exist_ok=True)
    api.upload_folder(
        repo_id=repo_id,
        repo_type="model",
        folder_path=str(args.folder),
        path_in_repo=args.path_in_repo,
        commit_message="Upload M20 Isaac Lab training checkpoints and reproduction notes",
    )
    print(f"Uploaded to private repo: https://huggingface.co/{repo_id}")


if __name__ == "__main__":
    main()
