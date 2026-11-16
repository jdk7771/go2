"""Restore pinned public upstream files, verifying SHA-256 before replacement.

All inputs are already included. Run only to restore a missing/corrupted snapshot:
    python3 fsgp_bgk_repro/fetch_sources.py
Existing valid files are skipped. No upstream code is executed by this script.
"""

from pathlib import Path
import hashlib
import json
import urllib.request


def main():
    root = Path(__file__).resolve().parent
    lock = json.loads((root / "sources.lock.json").read_text())
    for entry in lock["files"]:
        target = root / entry["local_path"]
        if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest() == entry["sha256"]:
            print(f"OK {entry['local_path']}")
            continue
        url = f"https://raw.githubusercontent.com/ZJU-FAST-Lab/FSGP_BGK/{lock['commit']}/{entry['upstream_path']}"
        with urllib.request.urlopen(url, timeout=60) as response:
            payload = response.read()
        if hashlib.sha256(payload).hexdigest() != entry["sha256"]:
            raise RuntimeError(f"checksum mismatch: {url}")
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_suffix(target.suffix + ".download")
        temporary.write_bytes(payload)
        temporary.replace(target)
        print(f"RESTORED {entry['local_path']}")


if __name__ == "__main__":
    main()
