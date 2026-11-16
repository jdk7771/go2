"""Download selected members of the official public KITTI archive using HTTP Range.

Never falls back to downloading the 84 GB archive. ZIP member CRCs and SHA-256
hashes are verified. Data remain subject to the KITTI dataset terms.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
import json
from pathlib import Path
import struct
import time
from urllib.request import Request, urlopen
import zipfile
import zlib


BASE = "https://s3.eu-central-1.amazonaws.com/avg-kitti/"
ROOT = Path(__file__).resolve().parent


def request(url, start=None, end=None):
    headers = {"User-Agent": "quadric-paper-reproduction/1.0"}
    if start is not None:
        headers["Range"] = f"bytes={start}-{end}"
    for attempt in range(4):
        try:
            with urlopen(Request(url, headers=headers), timeout=45) as response:
                if start is not None:
                    if response.status != 206:
                        raise ValueError("server ignored HTTP Range; refusing full archive download")
                    expected = end-start+1
                    payload = response.read(expected+1)
                    if len(payload) != expected:
                        raise ValueError(f"invalid range payload: {len(payload)} != {expected}")
                    return payload
                return response.read()
        except Exception:
            if attempt == 3:
                raise
            time.sleep(.4*(attempt+1))


class RemoteZip:
    def __init__(self, url):
        self.url = url
        with urlopen(Request(url, method="HEAD"), timeout=30) as response:
            self.size = int(response.headers["Content-Length"])
        tail_start = max(0, self.size-65557)
        tail = request(url, tail_start, self.size-1)
        eocd = tail.rfind(b"PK\x05\x06")
        if eocd < 0:
            raise ValueError("ZIP end-of-central-directory not found")
        fields = struct.unpack_from("<4s4H2LH", tail, eocd)
        central_size, central_offset = fields[5:7]
        if central_offset == 0xffffffff or central_size == 0xffffffff:
            # ZIP64 locator immediately precedes the conventional EOCD.
            _, _, zip64_offset, _ = struct.unpack_from("<4sLQL", tail, eocd-20)
            header = request(url, zip64_offset, zip64_offset+55)
            central_size, central_offset = struct.unpack_from("<QQ", header, 40)
        central = request(url, central_offset, central_offset+central_size-1)
        self.members = {}
        offset = 0
        while offset < len(central):
            if central[offset:offset+4] != b"PK\x01\x02":
                raise ValueError("malformed central directory")
            values = struct.unpack_from("<4s6H3L5H2L", central, offset)
            _, _, _, flags, method, _, _, crc, compressed, size, nlen, xlen, clen, _, _, _, local = values
            name = central[offset+46:offset+46+nlen].decode("utf-8" if flags & 0x800 else "cp437")
            extra = central[offset+46+nlen:offset+46+nlen+xlen]
            pos = 0
            while pos+4 <= len(extra):
                tag, length = struct.unpack_from("<HH", extra, pos)
                if tag == 1:
                    idx = pos+4
                    if size == 0xffffffff:
                        size, = struct.unpack_from("<Q", extra, idx); idx += 8
                    if compressed == 0xffffffff:
                        compressed, = struct.unpack_from("<Q", extra, idx); idx += 8
                    if local == 0xffffffff:
                        local, = struct.unpack_from("<Q", extra, idx)
                pos += 4+length
            self.members[name] = dict(name=name, flags=flags, method=method, crc32=crc,
                                      compressed_bytes=compressed, bytes=size, offset=local)
            offset += 46+nlen+xlen+clen

    def extract(self, name, destination):
        member = self.members[name]
        destination = Path(destination)
        if destination.exists():
            payload = destination.read_bytes()
            if len(payload) == member["bytes"] and zlib.crc32(payload) & 0xffffffff == member["crc32"]:
                return {**member, "sha256": hashlib.sha256(payload).hexdigest()}
        start = member["offset"]
        header = request(self.url, start, start+29)
        if header[:4] != b"PK\x03\x04":
            raise ValueError("invalid local member header")
        nlen, xlen = struct.unpack_from("<HH", header, 26)
        start += 30+nlen+xlen
        raw = request(self.url, start, start+member["compressed_bytes"]-1)
        payload = zlib.decompress(raw, -15) if member["method"] == 8 else raw
        if member["method"] not in (0, 8):
            raise ValueError("unsupported ZIP compression")
        if len(payload) != member["bytes"] or zlib.crc32(payload) & 0xffffffff != member["crc32"]:
            raise ValueError(f"member CRC/size mismatch: {name}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(payload)
        return {**member, "sha256": hashlib.sha256(payload).hexdigest()}


def fetch(sequence="04", frames=None, output=ROOT / "data/kitti"):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    archive = RemoteZip(BASE+"data_odometry_velodyne.zip")
    names = sorted(n for n in archive.members if n.startswith(f"dataset/sequences/{sequence}/velodyne/") and n.endswith(".bin"))
    if frames is not None:
        names = names[:frames]
    if not names:
        raise ValueError("no selected KITTI frames")
    print(f"Downloading {len(names)} frames from official archive using HTTP ranges", flush=True)
    def one(name):
        return archive.extract(name, output / name.removeprefix("dataset/"))
    records = []
    with ThreadPoolExecutor(max_workers=4) as pool:
        for index, record in enumerate(pool.map(one, names)):
            records.append(record)
            if (index+1) % 10 == 0 or index == len(names)-1:
                print(f"KITTI {sequence}: {index+1}/{len(names)} frames", flush=True)
    auxiliary = {}
    for file, prefix in (("data_odometry_calib.zip", f"dataset/sequences/{sequence}/"),
                         ("data_odometry_poses.zip", f"dataset/poses/{sequence}.txt"),
                         ("devkit_odometry.zip", "devkit/")):
        payload = request(BASE+file)
        with zipfile.ZipFile(io.BytesIO(payload)) as zipped:
            selected = [n for n in zipped.namelist() if n.startswith(prefix) and not n.endswith("/")]
            for name in selected:
                target = output / name.removeprefix("dataset/")
                target.parent.mkdir(parents=True, exist_ok=True)
                data = zipped.read(name)
                target.write_bytes(data)
                auxiliary[str(target.relative_to(output))] = {"url": BASE+file, "member": name,
                                                              "sha256": hashlib.sha256(data).hexdigest()}
    manifest = {"dataset": "KITTI odometry", "sequence": sequence, "frames": len(names),
                "url": archive.url, "archive_bytes": archive.size, "access": "HTTP Range + ZIP CRC32",
                "members": records, "auxiliary": auxiliary,
                "terms": "https://www.cvlibs.net/datasets/kitti/", "data_owned_by": "KITTI dataset authors"}
    (output / f"sequence_{sequence}_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", default="04")
    parser.add_argument("--frames", type=int)
    parser.add_argument("--output", type=Path, default=ROOT / "data/kitti")
    args = parser.parse_args()
    fetch(args.sequence, args.frames, args.output)
