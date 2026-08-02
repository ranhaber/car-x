#!/bin/bash
set -euo pipefail
ENV=/etc/car-x/car-x.env

set_kv() {
  local key="$1" val="$2"
  if sudo grep -q "^${key}=" "$ENV"; then
    sudo sed -i "s|^${key}=.*|${key}=${val}|" "$ENV"
  else
    printf '%s=%s\n' "$key" "$val" | sudo tee -a "$ENV" >/dev/null
  fi
}

set_kv CAT_FOLLOW_WEB_SSL_CERTFILE /etc/car-x/web-ui.crt
set_kv CAT_FOLLOW_WEB_SSL_KEYFILE /etc/car-x/web-ui.key
sudo grep SSL "$ENV"
sudo systemctl restart cat-follow
sleep 8
systemctl is-active cat-follow
curl -sk --max-time 3 https://127.0.0.1:5000/api/stream/capabilities
echo
sudo journalctl -u cat-follow -n 15 --no-pager | grep -E 'Running on|SSL|Error|Traceback|h264' || true
