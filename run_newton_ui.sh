#!/bin/bash

# Desktop launcher entry point for the Newton UI.
#
# The script mirrors the environment that previously lived in
# ``start_wunderbar.sh`` so the Qt UI looks correct on the Raspberry Pi while
# giving preference to the dedicated virtualenv interpreter when present.

set -euo pipefail
cd /home/admin/Newton

export QT_QPA_PLATFORMTHEME=${QT_QPA_PLATFORMTHEME:-gtk2}
export QT_STYLE_OVERRIDE=${QT_STYLE_OVERRIDE:-Fusion}
export DISPLAY=${DISPLAY:-:0}

if [ -x /home/admin/ice_shaver_env/bin/python3 ]; then
    exec /home/admin/ice_shaver_env/bin/python3 -m simple_control.ui_simple "$@"
else
    exec python3 -m simple_control.ui_simple "$@"
fi
