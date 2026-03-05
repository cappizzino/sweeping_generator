#!/usr/bin/env bash
set -euo pipefail

SRC_TOPIC="${1:-/uav1/leader_index}"
DST_TOPIC="${2:-/uav1/leader_index_gated}"
NODE_NAME="${3:-leader_index_gate}"
LOG_FILE="/tmp/${NODE_NAME}.log"

if rosnode list 2>/dev/null | grep -qx "/${NODE_NAME}"; then
  echo "Gate node '/${NODE_NAME}' is already running."
  echo "Source: ${SRC_TOPIC}"
  echo "Dest:   ${DST_TOPIC}"
  exit 0
fi

echo "Starting gate node '/${NODE_NAME}'..."
echo "Forwarding ${SRC_TOPIC} -> ${DST_TOPIC}"
nohup rosrun topic_tools drop "${SRC_TOPIC}" 0 1.0 "${DST_TOPIC}" "__name:=${NODE_NAME}" > "${LOG_FILE}" 2>&1 &

sleep 1

if rosnode list 2>/dev/null | grep -qx "/${NODE_NAME}"; then
  echo "Gate started: /${NODE_NAME}"
  echo "Log: ${LOG_FILE}"
  exit 0
fi

echo "Failed to start gate. Check log: ${LOG_FILE}"
exit 1
