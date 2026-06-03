#!/usr/bin/env python3
"""Smoke test for a running brainco_bridge — queries once and verifies touch_ok.

Exits 0 on success, non-zero on failure.
"""
import json
import socket
import sys


def main():
    s = socket.socket()
    s.settimeout(5.0)
    try:
        s.connect(("127.0.0.1", 9877))
    except Exception as e:
        print(f"    FAIL: cannot connect to 127.0.0.1:9877 ({e})")
        sys.exit(2)

    s.sendall(b'{"cmd":"get"}\n')
    buf = b""
    while b"\n" not in buf:
        chunk = s.recv(4096)
        if not chunk:
            break
        buf += chunk
    s.close()
    r = json.loads(buf.split(b"\n", 1)[0])

    l_ok = r.get("left_touch_ok")
    r_ok = r.get("right_touch_ok")
    print(f"    left_touch_ok     = {l_ok}")
    print(f"    right_touch_ok    = {r_ok}")
    print(f"    left_touch_force  = {r.get('left_touch_force')}")
    print(f"    right_touch_force = {r.get('right_touch_force')}")

    if not (l_ok and r_ok):
        print("    FAIL: touch enable did not succeed on at least one hand")
        sys.exit(1)
    print("    SMOKE TEST PASS")


if __name__ == "__main__":
    main()
