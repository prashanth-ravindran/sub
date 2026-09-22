#!/usr/bin/env bash
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_root"
venv_dir="$project_root/.venv"
base_python=${PYTHON_BIN:-python3}
if [[ ! -x "$venv_dir/bin/python" ]]; then
    "$base_python" -m venv "$venv_dir"
fi
python_bin="$venv_dir/bin/python"
"$python_bin" -m pip install -r requirements.txt
streamlit_bin=${STREAMLIT_BIN:-"$project_root/.venv/bin/streamlit"}
api_port=${SIMULATOR_API_PORT:-8765}
ui_port=${SIMULATOR_UI_PORT:-8501}

cleanup() {
    kill "$api_pid" "$ui_pid" 2>/dev/null || true
    wait "$api_pid" "$ui_pid" 2>/dev/null || true
}
trap cleanup EXIT

"$python_bin" -m simulator.http_api --port "$api_port" &
api_pid=$!
PYTHONPATH="$project_root${PYTHONPATH:+:$PYTHONPATH}" \
    VEHICLE_SIMULATOR_API_URL="http://127.0.0.1:$api_port" \
    "$streamlit_bin" run ui/simulator_app.py --server.address 127.0.0.1 --server.port "$ui_port" &
ui_pid=$!
wait -n "$api_pid" "$ui_pid"
