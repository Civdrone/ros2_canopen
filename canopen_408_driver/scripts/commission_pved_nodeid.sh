#!/bin/bash
# ---------------------------------------------------------------------------
# Assign a node-ID to a SINGLE Danfoss PVED-CC via LSS (CiA 305), for
# commissioning several identical valves that all ship with the same default ID.
#
# Because the valves are identical (same vendor/product/revision, and we zero the
# serial), they can't be told apart by identity -- so commission them ONE AT A
# TIME: connect exactly one PVED to the bus, run this, power-cycle it, repeat.
#
# This uses an LSS *global* switch to configuration mode. That's safe here because
# the only other node, the DST X720 inclinometer, is not LSS-capable
# (LSS_Supported=0) and won't respond. Do NOT run this with more than one PVED
# connected, or they'll all take the same new ID.
#
# Requires can-utils. Run on the target.
#
# Usage:  ./commission_pved_nodeid.sh <iface> <new_node_id>
#   new_node_id may be decimal (17) or hex (0x11). Range 1..126.
# Example (set the connected valve to node 17 = mast_pitch):
#   ./commission_pved_nodeid.sh canE 17
# ---------------------------------------------------------------------------
set -euo pipefail

IF="${1:?usage: commission_pved_nodeid.sh <iface> <new_node_id>}"
NEWID=$(( ${2:?missing new node id} ))
if (( NEWID < 1 || NEWID > 126 )); then echo "node id must be 1..126"; exit 1; fi
HEXID=$(printf '%02X' "$NEWID")

echo "Interface : $IF"
echo "New nodeID: 0x$HEXID ($NEWID)"
echo "Make sure EXACTLY ONE PVED-CC is connected to the bus."
echo

LOG=$(mktemp); candump -T 2000 "$IF,7E4:7FF" > "$LOG" &
CD=$!; trap 'kill $CD 2>/dev/null || true; rm -f "$LOG"' EXIT
sleep 0.3
send() { echo "  -> 7E5#$1"; cansend "$IF" "7E5#$1"; sleep 0.25; }

echo "1) LSS switch ALL LSS slaves to configuration mode (only the PVED responds)"
send "04.01.00.00.00.00.00.00"          # switch-state-global, mode=1 (configuration)

echo "2) Set node-ID to 0x$HEXID"
send "11.$HEXID.00.00.00.00.00.00"      # configure node-ID -> reply 7E4#11.<err> (00 ok, 01 out of range)

echo "3) Store LSS configuration to EEPROM"
send "17.00.00.00.00.00.00.00"          # store configuration -> reply 7E4#17.<err>

echo "4) LSS switch back to operation mode"
send "04.00.00.00.00.00.00.00"          # switch-state-global, mode=0 (operation)

echo "5) Reset node to apply the new ID"
cansend "$IF" "000#81.00"               # NMT reset-node, all

sleep 0.3; kill $CD 2>/dev/null || true
echo; echo "===== LSS replies (0x7E4) ====="; cat "$LOG"
echo
echo "Verify:  candump -tz $IF,700:780   (expect boot-up 0x7$HEXID ...)"
echo "Then set node_id: 0x$HEXID for the matching valve in bus.yml."
