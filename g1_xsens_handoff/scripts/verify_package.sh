#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

required=(
  "START_HERE.md"
  "STATUS_AND_PLAN.md"
  "RUNBOOK.md"
  "xsens_bridge/xudp.py"
  "xsens_bridge/g1_retarget.py"
  "xsens_bridge/x2_retarget.py"
  "xsens_bridge/live_reference.py"
  "xsens_bridge/global_pelvis.py"
  "xsens_bridge/global_position.py"
  "xsens_bridge/g1_retarget_working_20260820.py"
  "xsens_bridge/live_retarget_working_20260820.py"
  "tools/export_protomotions_cache.py"
  "tools/validate_live_onnx_watchdog.py"
  "tools/live_xsens_global_locomotion.py"
  "tools/live_xsens_global_position.py"
  "tools/preview_xsens_x2.py"
  "scripts/run_development_jetson.sh"
  "scripts/run_jetson_global_position_controller.sh"
  "scripts/fetch_agibot_x2_models.sh"
  "requirements-jetson.txt"
  "G1_INSTALLATION.md"
  "GLOBAL_PELVIS_LOCOMOTION.md"
  "X2_SIMULATION.md"
  "requirements-x2-sim.txt"
  "data/xsens_20260723_114126.xudp"
  "data/xsens_20260723_114126_g1_reference.npz"
)

for path in "${required[@]}"; do
  test -f "$path" || {
    echo "MISSING: $path"
    exit 1
  }
done

if command -v python >/dev/null 2>&1; then
  python_cmd=python
elif command -v python3 >/dev/null 2>&1; then
  python_cmd=python3
else
  echo "MISSING: Python 3"
  exit 1
fi

"$python_cmd" -c "import numpy; import xsens_bridge; print('Python imports: OK')"
"$python_cmd" -m xsens_bridge summary data/xsens_20260723_114126.xudp
echo "Handoff package: OK"
