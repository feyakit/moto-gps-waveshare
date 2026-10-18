#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
symbols='建国路东三环中朝阳门外大街进入岛，第二出口已到达目的地前方拥堵缓行畅通偏航重新规划断网恢复直左右转掉头米公里分钟高速辅主道匝隧桥请保持车驶向终点当前路线热点在线离线获取定位无信号等待距下一动作打开手机应用正在建立连接成功选择'

cd "${repo_dir}"
npx --yes lv_font_conv \
  --font third_party/lvgl/scripts/built_in_font/SourceHanSansSC-Normal.otf \
  --size 16 \
  --bpp 4 \
  --format lvgl \
  --no-compress \
  --symbols "${symbols}" \
  -o shared/nav_ui/assets/moto_font_nav_16.c

# Large digits for the Clock page. Montserrat keeps the same type family as the
# built-in instrument fonts; only the glyphs the dial needs are embedded. The
# hyphen is required for the "--:--" placeholder state: with
# LV_USE_FONT_PLACEHOLDER (default on) a missing glyph is drawn as a solid box.
npx --yes lv_font_conv \
  --font third_party/lvgl/scripts/built_in_font/Montserrat-Medium.ttf \
  --size 96 \
  --bpp 4 \
  --format lvgl \
  --no-compress \
  --symbols '0123456789:- ' \
  -o shared/nav_ui/assets/moto_font_clock_96.c

# Notification text is arbitrary user content, so the notification fonts embed
# the GB2312 level-1 Hanzi set (plus ASCII and its punctuation) instead of a
# fixed phrase list: 99.7% of everyday Chinese text is covered without pulling
# in the whole CJK block.
notify_symbols_file="$(mktemp)"
trap 'rm -f "${notify_symbols_file}"' EXIT
python3 - "${notify_symbols_file}" <<'PYTHON'
import sys

symbols = []
for hi in range(0xA1, 0xAA):
    for lo in range(0xA1, 0xFF):
        try:
            symbols.append(bytes([hi, lo]).decode("gb2312"))
        except UnicodeDecodeError:
            pass
for hi in range(0xB0, 0xD8):
    for lo in range(0xA1, 0xFF):
        try:
            symbols.append(bytes([hi, lo]).decode("gb2312"))
        except UnicodeDecodeError:
            pass
ascii_chars = "".join(chr(c) for c in range(0x20, 0x7F))
with open(sys.argv[1], "w") as handle:
    handle.write("".join(dict.fromkeys(ascii_chars + "".join(symbols))))
PYTHON
notify_symbols="$(cat "${notify_symbols_file}")"

# 16/20 are the compact rows (page chrome, app name, music labels). 28/34 are
# the sizes the full-screen notification card and the message history page
# need: the card is read at a glance at speed, so 20 px body text was
# unreadable.
# The two card sizes cover ~3.7k glyphs; at 4 bpp their read-only copy pushed
# the PSRAM pools past boot (ESP_ERR_NO_MEM reserving the LVGL layer pool
# right after enabling them). They drop to 2 bpp instead, which halves the
# bitmaps and stays perfectly legible at these sizes on an AMOLED panel.
for size in 16 20 28 34; do
  bpp=4
  if [ "${size}" -ge 28 ]; then bpp=2; fi
  npx --yes lv_font_conv \
    --font third_party/lvgl/scripts/built_in_font/SourceHanSansSC-Normal.otf \
    --size "${size}" \
    --bpp "${bpp}" \
    --format lvgl \
    --no-compress \
    --symbols "${notify_symbols}" \
    -o "shared/nav_ui/assets/moto_font_notify_${size}.c"
done
