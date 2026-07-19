#!/bin/bash
# One-shot sample: liveness + stress-ng + max SoC temp
set -euo pipefail

echo "=== SAMPLE $(date -Is) ==="
echo "--- ALIVE ---"
uptime
echo "--- LOAD / MEM ---"
free -h | head -n 2
echo "--- STRESS ---"
if pgrep -af stress-ng >/tmp/stress_ps.txt 2>/dev/null; then
  cat /tmp/stress_ps.txt
  echo "stress_alive=yes"
else
  echo "stress-ng not running"
  echo "stress_alive=no"
fi
echo "--- TEMP ---"
max_c=0
max_name=""
for z in /sys/class/thermal/thermal_zone*/temp; do
  [ -f "$z" ] || continue
  name=$(cat "${z%/temp}/type" 2>/dev/null || echo unknown)
  raw=$(cat "$z" 2>/dev/null || echo 0)
  c=$((raw / 1000))
  printf "%s: %sC\n" "$name" "$c"
  if [ "$c" -gt "$max_c" ]; then
    max_c=$c
    max_name=$name
  fi
done
echo "max_temp_c=${max_c} (${max_name})"
echo "=== END SAMPLE ==="
