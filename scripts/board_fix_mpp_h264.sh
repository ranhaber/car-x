#!/bin/bash
# Install Rockchip GStreamer MPP plugin + device permissions for H.264 encode.
set -euo pipefail

SO=/tmp/mirrors-gstreamer-rockchip/build/gst/rockchipmpp/libgstrockchipmpp.so
DEST=/usr/lib/aarch64-linux-gnu/gstreamer-1.0
DEVICE_GROUP=video

if [[ "$EUID" -ne 0 ]]; then
  echo "run as root (for example: sudo $0)" >&2
  exit 1
fi

if ! getent group "$DEVICE_GROUP" >/dev/null; then
  echo "required device group does not exist: $DEVICE_GROUP" >&2
  exit 1
fi

if [[ ! -f "$SO" ]]; then
  echo "missing plugin build: $SO" >&2
  exit 1
fi

install -m 0755 -v "$SO" "$DEST/"
for device in /dev/mpp_service /dev/dma_heap/system \
  /dev/dma_heap/system-uncached /dev/dma_heap/reserved; do
  if [[ -e "$device" ]]; then
    chgrp "$DEVICE_GROUP" "$device"
    chmod 0660 "$device"
  fi
done

cat > /etc/udev/rules.d/99-rockchip-mpp.rules << 'EOF'
KERNEL=="mpp_service", GROUP="video", MODE="0660"
KERNEL=="system", SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"
KERNEL=="system-uncached", SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"
KERNEL=="reserved", SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"
EOF

udevadm control --reload-rules || true
udevadm trigger || true
rm -f /home/picarx/.cache/gstreamer-1.0/registry.*.bin || true
rm -f /root/.cache/gstreamer-1.0/registry.*.bin || true

ls -l "$DEST/libgstrockchipmpp.so" /dev/mpp_service /dev/dma_heap/* || true
echo "fix_mpp_h264 done"
