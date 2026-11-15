#!/usr/bin/env bash
# Preflight Verification for Trailhawk Port (StarPilot Dom RC / 7.0)
# Run before pushing or releasing to vehicle to ensure full autonomy engagement.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SCRIPT_DIR"

PYTHON_BIN="python3"
if [ -x "/usr/local/venv/bin/python3" ]; then
  PYTHON_BIN="/usr/local/venv/bin/python3"
fi
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

echo "=========================================="
echo "  Trailhawk Port Preflight Verification"
echo "=========================================="

# 1. CarParams & Autonomy Checks
echo -n "[1/4] Checking JEEP_CHEROKEE_5TH_GEN CarParams... "
"$PYTHON_BIN" - << 'EOF'
import sys
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car import structs

cp = CarInterface.get_non_essential_params("JEEP_CHEROKEE_5TH_GEN")
assert not cp.dashcamOnly, "FAIL: dashcamOnly is True!"
assert not cp.passive, "FAIL: passive is True!"
assert cp.safetyConfigs and cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.chryslerCusw, \
    f"FAIL: safetyModel is not chryslerCusw ({cp.safetyConfigs})"
print("OK (dashcamOnly=False, passive=False, safety=chryslerCusw)")
EOF

# 2. Torque Config Duplicate Check
echo -n "[2/4] Verifying torque override / substitute sanity... "
"$PYTHON_BIN" - << 'EOF'
from opendbc.car.interfaces import get_torque_params
tp = get_torque_params()
assert "JEEP_CHEROKEE_5TH_GEN" in tp, "FAIL: JEEP_CHEROKEE_5TH_GEN missing from torque config"
print(f"OK (measured lat accel: {tp['JEEP_CHEROKEE_5TH_GEN']['MAX_LAT_ACCEL_MEASURED']})")
EOF

# 3. Panda Firmware Signature Check
echo -n "[3/4] Verifying signed Panda H7 binary... "
if [ -f "panda/board/obj/panda_h7.bin.signed" ]; then
  "$PYTHON_BIN" - << 'EOF'
from panda import Panda
sig = Panda.get_signature_from_firmware("panda/board/obj/panda_h7.bin.signed")
print(f"OK (sig: {sig.hex()[:16]})")
EOF
else
  echo "FAIL: panda/board/obj/panda_h7.bin.signed missing!"
  exit 1
fi

# 4. Critical Daemon Imports (Galaxy, Card, etc.)
echo -n "[4/4] Verifying critical daemons import cleanly... "
"$PYTHON_BIN" - << 'EOF'
import openpilot.starpilot.galaxy.server
import openpilot.starpilot.galaxy.telemetry
import openpilot.selfdrive.car.card
print("OK (galaxy server & card importable without missing modules)")
EOF

echo "=========================================="
echo "  PREFLIGHT PASSED: Ready for Trailhawk!"
echo "=========================================="
