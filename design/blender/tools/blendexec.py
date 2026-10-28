#!/usr/bin/env python3
"""Run Python in the open Blender through Blender Lab's MCP add-on bridge (localhost:9876).

    blendexec.py SCRIPT.py      or      blendexec.py -c "code"
"""
import json
import socket
import sys

code = sys.argv[2] if sys.argv[1] == "-c" else open(sys.argv[1]).read()
s = socket.create_connection(("127.0.0.1", 9876), timeout=3000)
s.sendall((json.dumps({"type": "execute", "code": code, "strict_json": False}) + "\0").encode())
buf = b""
while not buf.endswith(b"\0"):
    chunk = s.recv(65536)
    if not chunk:
        break
    buf += chunk
r = json.loads(buf.rstrip(b"\0") or b"{}")
for k in ("status", "message", "result", "stdout", "stderr"):
    if r.get(k):
        print(f"[{k}]", r[k] if isinstance(r[k], str) else json.dumps(r[k])[:4000])
