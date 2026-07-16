#!/bin/bash
# ---------------------------------------------------------------------------
# Change the CANopen node-ID of a Danfoss PVED-CC Series 5 valve over LSS.
#
# The PVED sets its node-ID via LSS (CiA 305), per manual BC180386484705en-000802
# ("Configure nodeID" = LSS CS 0x11, "store" = CS 0x17). Because other nodes share
# the bus (e.g. the inclinometer), this uses LSS *selective* switch (by the valve's
# vendor/product/revision/serial identity) so ONLY the PVED enters config mode.
#
# Requires can-utils (cansend/candump). Run on the target (e.g. the iot-gate).
#
# Usage:
#   ./set_pved_nodeid.sh <iface> <new_node_id_hex> [vendor product revision serial]
# Example (change our unit to node 0x11 = 17):
#   ./set_pved_nodeid.sh canE 0x11
#   ./set_pved_nodeid.sh canE 0x11 0x01000019 0x53373235 0x00010100 0x0AD4021C
# ---------------------------------------------------------------------------
set -euo pipefail

IF="${1:?usage: set_pved_nodeid.sh <iface> <new_node_id_hex> [vendor product rev serial]}"
NEWID="${2:?missing new node id (e.g. 0x11)}"
VENDOR="${3:-0x01000019}"   # 0x1018:1  (read from this unit 2026-06-29)
PRODUCT="${4:-0x53373235}"  # 0x1018:2
REVISION="${5:-0x00010100}" # 0x1018:3
SERIAL="${6:-0x0AD4021C}"   # 0x1018:4  <-- per-unit; override for a different valve

# little-endian dotted byte string for a 32-bit value, e.g. 0x01000019 -> 19.00.00.01
le32() { printf '%02X.%02X.%02X.%02X' $(( ($1) & 0xFF )) $(( (($1)>>8) & 0xFF )) $(( (($1)>>16) & 0xFF )) $(( (($1)>>24) & 0xFF )); }
b() { printf '%02X' $(( ($1) & 0xFF )); }

echo "Interface : $IF"
echo "New nodeID: $(b "$NEWID")"
echo "Identity  : vendor=$VENDOR product=$PRODUCT rev=$REVISION serial=$SERIAL"
echo

# capture LSS slave replies (0x7E4) in the background
LOG=$(mktemp)
candump -T 2000 "$IF,7E4:7FF" > "$LOG" &
CD=$!
trap 'kill $CD 2>/dev/null || true; rm -f "$LOG"' EXIT
sleep 0.3

send() { echo "  -> 7E5#$1"; cansend "$IF" "7E5#$1"; sleep 0.2; }

echo "1) LSS selective switch to configuration mode (by identity)"
send "40.$(le32 "$VENDOR")"      # vendor-id
send "41.$(le32 "$PRODUCT")"     # product-code
send "42.$(le32 "$REVISION")"    # revision-number
send "43.$(le32 "$SERIAL")"      # serial-number  -> slave replies 7E4#44 if all match
sleep 0.3
if ! grep -q " 7E4 .* 44 " "$LOG"; then
  echo "!! No 0x44 confirmation from the valve -- identity mismatch or not present."
  echo "   Captured replies:"; cat "$LOG"; exit 1
fi
echo "   confirmed in config mode."

echo "2) Configure node-ID ($(b "$NEWID"))"
send "11.$(b "$NEWID").00.00.00.00.00.00"   # response 7E4#11.<err>  (00=ok, 01=out of range)

echo "3) Store configuration to EEPROM"
send "17.00.00.00.00.00.00.00"              # response 7E4#17.<err>

echo "4) LSS switch back to operation mode (global)"
send "04.00.00.00.00.00.00.00"

echo "5) NMT reset node so the new node-ID takes effect"
cansend "$IF" "000#81.00"                   # reset-node, all nodes

echo
echo "Done. Verify with:  candump -tz $IF,700:780   (expect boot-up 0x7$(b "$NEWID") ...)"
echo "Then update node_id in bus.yml to $(b "$NEWID")."
