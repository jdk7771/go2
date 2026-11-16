"""Fetch the exact public collision assets recorded in sources.lock.json."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import urllib.request


def main():
    base = Path(__file__).resolve().parent
    sources = json.loads((base / "sources.lock.json").read_text())["sources"]
    tasks = [(source, file) for source in sources for file in source["files"]]

    def fetch(task):
        source, file = task
        root = base / source["kind"]
        destination = root / file["path"]
        if not destination.resolve().is_relative_to(root.resolve()):
            raise ValueError("Invalid source path")
        if destination.exists():
            if hashlib.sha256(destination.read_bytes()).hexdigest() != file["sha256"]:
                raise ValueError(f"Existing source was modified: {destination}")
            return
        repository = source["repository"].removeprefix("https://github.com/")
        url = f"https://raw.githubusercontent.com/{repository}/{source['commit']}/{file['path']}"
        with urllib.request.urlopen(url, timeout=60) as response:
            content = response.read()
        if hashlib.sha256(content).hexdigest() != file["sha256"]:
            raise ValueError(f"Source hash mismatch: {url}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)

    with ThreadPoolExecutor(max_workers=5) as pool:
        list(pool.map(fetch, tasks))
    print(f"Verified {len(tasks)} source files from {len(sources)} pinned repositories.")


if __name__ == "__main__":
    main()
