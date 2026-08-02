#!/bin/bash
# Generate a self-signed TLS cert for the cat-follow web UI (WebCodecs).
set -euo pipefail
DIR=/etc/car-x
CRT="$DIR/web-ui.crt"
KEY="$DIR/web-ui.key"
HOST="${1:-192.168.7.67}"

sudo mkdir -p "$DIR"
if [[ -f "$CRT" && -f "$KEY" ]]; then
  echo "cert already exists: $CRT"
  exit 0
fi

sudo openssl req -x509 -newkey rsa:2048 -sha256 -days 825 -nodes \
  -keyout "$KEY" -out "$CRT" \
  -subj "/CN=cat-follow-web-ui" \
  -addext "subjectAltName=IP:${HOST},DNS:localhost"
sudo chmod 644 "$CRT"
sudo chmod 640 "$KEY"
sudo chown root:picarx "$KEY"
echo "wrote $CRT and $KEY"
