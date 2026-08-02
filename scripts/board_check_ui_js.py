#!/usr/bin/env python3
import pathlib
import re
import subprocess
import tempfile

html = pathlib.Path("/opt/car-x/cat_follow/web_ui/templates/main.html").read_text(encoding="utf-8")
m = re.search(r"<script>\s*(function catFollow\(\).*?)\s*</script>", html, re.S)
print("match", bool(m), "LIVE" in html, "alpha: false" in html, "streamOverlay" in html)
if not m:
    raise SystemExit(1)
js = m.group(1)
path = pathlib.Path("/tmp/catfollow_check.js")
path.write_text(js, encoding="utf-8")
# syntax check with node if present else python compile via dukpy? just look for obvious issues
try:
    subprocess.check_call(["node", "--check", str(path)])
    print("node_syntax_ok")
except Exception as e:
    print("node_check", e)
print("startH264", js.count("startH264"))
print("getContext", [line.strip() for line in js.splitlines() if "getContext" in line])
