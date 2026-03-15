#!/usr/bin/env bash
set -euo pipefail

MANUS_SDK_CLIENT_DIR="${MANUS_SDK_CLIENT_DIR:-/data/Yin_ws/Bidex_Manus_Teleop/MANUS_Core_2.4.0_SDK/SDKClient_Linux}"
cd "$MANUS_SDK_CLIENT_DIR"

echo "Starting external MANUS SDK client from: $MANUS_SDK_CLIENT_DIR"
echo "When the MANUS menu appears, choose option 1."
exec sudo ./SDKClient_Linux.out
