#!/usr/bin/env python3
"""Decode the frames written by test_logic with pymavlink and check every field.

pymavlink verifies the MAVLink 2 header, CRC and CRC_EXTRA, so this proves the hand-written
encoder in src/mavlink_lite.c produces frames a real MAVLink stack (ArduPilot, Mission Planner)
accepts.
"""
import sys

from pymavlink.dialects.v20 import common as mavlink


def main(path):
    data = open(path, "rb").read()
    mav = mavlink.MAVLink(None)
    mav.robust_parsing = False
    msgs = mav.parse_buffer(data) or []
    errors = []

    def expect(cond, what):
        if not cond:
            errors.append(what)

    expect(len(msgs) == 6, f"expected 6 messages, got {len(msgs)}")
    for i, m in enumerate(msgs):
        expect(m.get_srcSystem() == 1 and m.get_srcComponent() == 25, f"msg {i}: wrong sysid/compid")
        expect(m.get_seq() == i, f"msg {i}: seq {m.get_seq()}")

    if len(msgs) == 6:
        hb, spd, arm, strpos, st, longst = msgs
        expect(hb.get_type() == "HEARTBEAT", "msg 0 not HEARTBEAT")
        expect(hb.type == 18 and hb.autopilot == 8 and hb.system_status == 4
               and hb.mavlink_version == 3, f"heartbeat fields {hb}")

        expect(spd.get_type() == "NAMED_VALUE_FLOAT", "msg 1 type")
        expect(spd.name == "DBW_SPEED" and abs(spd.value - 612.5) < 1e-6
               and spd.time_boot_ms == 123456, f"speed fields {spd}")

        expect(arm.name == "DBW_ARM" and arm.value == 0.0 and arm.time_boot_ms == 0,
               f"arm fields {arm}")
        expect(strpos.name == "DBW_STRPOS" and strpos.value == -137.0, f"strpos fields {strpos}")

        expect(st.get_type() == "STATUSTEXT", "msg 4 type")
        expect(st.severity == 4 and st.text == "DBW: steer cal ok, centre 50 range +/-180",
               f"statustext fields {st}")
        expect(longst.text == "x" * 50, "long statustext not truncated to 50 chars")

    if errors:
        for e in errors:
            print("FAIL", e)
        return 1
    print(f"pymavlink decoded {len(msgs)} frames, all fields OK")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
