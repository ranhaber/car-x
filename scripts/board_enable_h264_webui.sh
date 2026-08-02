#!/bin/bash
# Run on board as picarx (uses passwordless sudo where configured, else prompt).
set -euo pipefail

ENV_FILE=/etc/car-x/car-x.env
UNIT=/etc/systemd/system/cat-follow.service

if grep -q '^CAT_FOLLOW_WEB_REQUIRE_H264=' "$ENV_FILE" 2>/dev/null; then
  sudo sed -i 's/^CAT_FOLLOW_WEB_REQUIRE_H264=.*/CAT_FOLLOW_WEB_REQUIRE_H264=1/' "$ENV_FILE"
else
  echo 'CAT_FOLLOW_WEB_REQUIRE_H264=1' | sudo tee -a "$ENV_FILE" >/dev/null
fi

if grep -q '^ExecStart=.*--web-ui' "$UNIT"; then
  echo "web-ui already enabled in $UNIT"
else
  sudo sed -i \
    's|^ExecStart=/opt/car-x/venv/bin/python -m cat_follow.runtime.app --picarx --with-prototype-perception$|ExecStart=/opt/car-x/venv/bin/python -m cat_follow.runtime.app --picarx --with-prototype-perception --web-ui --web-ui-port 5000|' \
    "$UNIT"
  sudo sed -i '/^# ExecStart=.*--web-ui/d' "$UNIT"
  sudo sed -i '/^# Optional monitoring UI/d' "$UNIT"
fi

sudo systemctl daemon-reload
sudo systemctl enable cat-follow.service
sudo systemctl restart cat-follow.service
sleep 5
systemctl is-active cat-follow.service || { sudo journalctl -u cat-follow -n 40 --no-pager; exit 1; }
curl -sf http://127.0.0.1:5000/api/stream/capabilities
echo
curl -sf -o /dev/null -w 'home_http=%{http_code}\n' http://127.0.0.1:5000/
curl -sf -o /dev/null -w 'mjpeg_http=%{http_code}\n' http://127.0.0.1:5000/stream || true
