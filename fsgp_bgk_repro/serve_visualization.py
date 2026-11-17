"""Serve all local point-cloud experiments on the agreed port 42482."""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from urllib.parse import urlsplit

PORT = 42482
ROOT = Path(__file__).resolve().parent / "outputs/visualization"


class Handler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def do_GET(self):
        if urlsplit(self.path).path == "/health.json":
            catalog = ROOT / "scene-fits/catalog.json"
            data = json.loads(catalog.read_text()) if catalog.exists() else {"scenes": []}
            payload = json.dumps({"ok": True, "host": "127.0.0.1", "port": PORT,
                                  "scenes": len(data["scenes"]), "root": str(ROOT)}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        super().do_GET()


if __name__ == "__main__":
    print(f"Point cloud viewer: http://127.0.0.1:{PORT}/", flush=True)
    ThreadingHTTPServer(("127.0.0.1", PORT), partial(Handler, directory=str(ROOT))).serve_forever()
