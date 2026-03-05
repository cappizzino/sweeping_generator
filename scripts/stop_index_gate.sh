#!/usr/bin/env bash
set -euo pipefail

NODE_NAME="${1:-leader_index_gate}"

if ! rosnode list 2>/dev/null | grep -qx "/${NODE_NAME}"; then
  echo "Gate node '/${NODE_NAME}' is not running."
  exit 0
fi

echo "Stopping gate node '/${NODE_NAME}'..."
rosnode kill "/${NODE_NAME}" >/dev/null
echo "Stopped."
