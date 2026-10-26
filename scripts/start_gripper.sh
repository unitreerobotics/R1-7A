#!/usr/bin/env bash
# Start the right Dex1 (ID7) using the same duplicate-service checks as dual mode.
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$PROJECT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec python -m dual.grippers --side right "$@"
