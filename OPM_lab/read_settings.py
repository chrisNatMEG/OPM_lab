#!/usr/bin/env python3
"""
Dump FASTRAK configuration/settings over serial to a JSON file.

This script:
  • Connects to the FASTRAK over a serial port (RS-232/USB-serial),
  • Disables continuous output, enforces ASCII format,
  • Reads system-wide settings (S, X, y, v, x),
  • Detects active stations (l) and queries per-station settings (A, H, G, I, N, O, Q, r, V),
  • Saves everything to JSON.

USAGE:
  python fastrak_dump_settings.py --port COM3 --baud 115200 --out fastrak_settings.json
  python fastrak_dump_settings.py --port /dev/ttyUSB0 --baud 115200 --out fastrak_settings.json

Notes:
  • Commands are terminated with CR '\r' per manual (“< >” = CR). 
  • We send 'c' (disable continuous) and 'F' (ASCII output) up front to keep responses parseable.
  • The script is read-only: it does NOT save changes or reset the device.

Tested with pyserial >= 3.x
"""

import argparse
import json
import sys
import time
import re
from datetime import datetime

import serial  # pip install pyserial

# ------------- Utilities ------------- #

def open_port(port: str, baud: int, timeout: float = 1.0) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=baud,  # Baud rate
        stopbits=serial.STOPBITS_ONE,  # Stop bits (1 stop bit)
        parity=serial.PARITY_NONE,  # No parity
        bytesize=serial.EIGHTBITS,  # 8 data bits
        rtscts=False,  # No hardware flow control
        timeout=1,  # Read timeout in seconds
        write_timeout=1,  # Write timeout in seconds
        xonxoff=False  # No software flow control
    )
    # Small settle time for USB-serial adapters
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def send_cmd(ser: serial.Serial, cmd: str, expect=None, tries: int = 3, read_timeout: float = 1.0):
    """
    Send one command (without CR) and read lines for a short while.
    If expect is given, return the first line that matches the regex expect.
    Otherwise return all non-empty lines received.
    """
    full = cmd + "\r"
    last_exc = None
    for _ in range(tries):
        try:
            ser.reset_input_buffer()
            ser.write(full.encode("ascii"))
            ser.flush()
            lines = []
            t0 = time.time()
            while time.time() - t0 < read_timeout:
                line = ser.readline()  # reads until '\n' or timeout
                if not line:
                    continue
                try:
                    s = line.decode("ascii", errors="ignore").strip()
                except UnicodeDecodeError:
                    s = line.decode("latin1", errors="ignore").strip()
                if not s:
                    continue
                lines.append(s)
                if expect:
                    if re.search(expect, s):
                        return s
            if expect:
                # try again
                continue
            else:
                return lines
        except Exception as e:
            last_exc = e
            time.sleep(0.05)
    if expect:
        raise TimeoutError(f"Command '{cmd}' did not return a line matching {expect!r} in time.") from last_exc
    return []


def ensure_ascii_and_quiet(ser: serial.Serial):
    """
    Disable continuous print and force ASCII output (safe, read-only).
    """
    # Suspend continuous if any ('^S') and 'c' (disable continuous output mode)
    for cm in ["\x13", "c", "F"]:  # ^S, c, F
        try:
            send_cmd(ser, cm, tries=1, read_timeout=0.3)
        except Exception:
            pass
    # Short pause for device to settle
    time.sleep(0.1)


def parse_S_record(s_line: str):
    """
    Parse 'S' system status line.

    Manual format: 2 a S fff bbb F3 c ... version ... config_string
    Where fff (hex) packs flags (bit 0: ASCII/Binary, 1: Units IN/CM, 2: Compensation, 3: Continuous, ...)
    """
    # Keep the raw line too
    out = {"raw": s_line}
    # Tokens: "2aSfff bbb F3 c  ..version..  ..config.."
    # Split on whitespace but keep compacted segments
    parts = s_line.split()
    if len(parts) >= 3 and parts[2].endswith("S"):
        # Try to find the hex flags (H3) contiguous to 'S' or following
        # Common layouts: "2aS3F0" or "2 a S 3F0"
        # Extract after 'S'
        after_S = s_line.split("S", 1)[1].strip()
        # The first 3 hex chars are flags per manual
        hex_flags = after_S[:3]
        out["flags_hex"] = hex_flags
        try:
            flags_val = int(hex_flags, 16)
        except ValueError:
            flags_val = None

        if flags_val is not None:
            # Bits: LSBit -> 0 Output Format (0=ASCII,1=Binary)
            #        1 Units (0=Inches,1=Centimeters)
            #        2 Compensation (0=Off,1=On)
            #        3 Transmit Mode (0=Non-Cont,1=Cont)
            flags = {
                "output_format": "Binary" if (flags_val & (1 << 0)) else "ASCII",
                "units": "Centimeters" if (flags_val & (1 << 1)) else "Inches",
                "compensation": bool(flags_val & (1 << 2)),
                "continuous_mode": bool(flags_val & (1 << 3)),
            }
            out["flags"] = flags

        # BIT error code: next 3 characters after the 3 hex flags (may be spaces for 'no error')
        rest = after_S[3:]
        bit_err = rest[:3]
        out["bit_error_code_raw"] = bit_err

        # ID tag (usually 'F3') and sensor map char follow; try to scrape simply
        m = re.search(r"\b([0-9A-F]{2})\b\s+([^\s])", rest[3:].strip())
        if m:
            out["id_tag"] = m.group(1)
            out["sensor_map_raw"] = m.group(2)

        # Firmware version often looks like n.n.nn (varies)
        mver = re.search(r"(\d+\.\d+\.\d+|\d+\.\d+)", s_line)
        if mver:
            out["firmware_version"] = mver.group(1)
    return out


def parse_vector_triplet_fields(payload: str, count=3):
    """
    Helper to parse space-delimited floats, expecting 3 values (Ox, Oy, Oz) etc.
    """
    toks = payload.replace(",", " ").split()
    floats = []
    for t in toks[:count]:
        try:
            floats.append(float(t))
        except Exception:
            pass
    if len(floats) == count:
        return floats
    return None


def query_simple_value(ser: serial.Serial, cmd: str, expect_tag: str, parser=float, fallback=None):
    """
    Send a command and parse a tagged '2[station][tag]' record line for a single or few numeric fields.
    """
    # The manual prefixes most replies with '2' then station/blank then sub-record letter.
    line = send_cmd(ser, cmd, expect=rf"\b{expect_tag}\b", read_timeout=0.8)
    if not line:
        return fallback, None
    # Return raw as well
    val = None
    try:
        # Extract numbers in line
        nums = re.findall(r"[-+]?\d+\.\d+|[-+]?\d+", line)
        if nums:
            if parser is float:
                val = [float(x) for x in nums]
            elif parser is int:
                val = [int(x) for x in nums]
            else:
                val = nums
            if len(val) == 1:
                val = val[0]
    except Exception:
        val = fallback
    return val, line


def query_station_list_ints(ser: serial.Serial, cmd: str, expect_tag: str):
    """
    For 'O' (output data list) which returns pairs of two-digit IDs.
    """
    line = send_cmd(ser, cmd, expect=rf"\b{expect_tag}\b", read_timeout=0.8)
    if not line:
        return None, None
    ids = re.findall(r"\b(\d{1,3})\b", line)
    # The first numbers could include station record fields; keep all numeric tokens after the tag
    # Find the position of the tag and re-scan after it
    try:
        p = line.index(expect_tag)
        tail = line[p+1:]
        ids = re.findall(r"\b(\d{1,3})\b", tail)
    except ValueError:
        pass
    ints = [int(x) for x in ids] if ids else []
    return ints, line


def query_filters(ser: serial.Serial):
    # 'v' (attitude) and 'x' (position): calling without parameters returns current values
    v_vals, v_raw = query_simple_value(ser, "v", "v", parser=float, fallback=None)
    x_vals, x_raw = query_simple_value(ser, "x", "x", parser=float, fallback=None)

    def to_named(vals):
        if isinstance(vals, list) and len(vals) >= 4:
            F, FLOW, FHIGH, FACTOR = vals[:4]
            return dict(F=F, FLOW=FLOW, FHIGH=FHIGH, FACTOR=FACTOR)
        return None

    return {
        "attitude": to_named(v_vals) if v_vals else None,
        "attitude_raw": v_raw,
        "position": to_named(x_vals) if x_vals else None,
        "position_raw": x_raw,
    }


def query_system(ser: serial.Serial):
    sys_info = {}

    # System status 'S'
    s_line = send_cmd(ser, "S", expect=r"\bS\b", read_timeout=0.8)
    if s_line:
        sys_info["status"] = parse_S_record(s_line)
    else:
        sys_info["status"] = None

    # Config user string 'X'
    x_line = send_cmd(ser, "X", expect=r"\bX\b", read_timeout=0.8)
    if x_line:
        # After the 'X' tag comes the ASCII configuration string
        # Keep the whole line; also try to extract the trailing string.
        sys_info["config_string_raw"] = x_line
        # Extract everything after 'X'
        try:
            idx = x_line.index("X")
            cfg = x_line[idx+1:].strip()
        except ValueError:
            cfg = x_line
        sys_info["config_string"] = cfg
    else:
        sys_info["config_string_raw"] = None
        sys_info["config_string"] = None

    # Sync mode 'y'
    y_line = send_cmd(ser, "y", expect=r"\by\b", read_timeout=0.8)
    if y_line:
        # The mode code is a small integer after 'y': 0=Internal, 1=External, 2=CRT/Video
        m = re.search(r"\by\b\s+([0-2])", y_line)
        if m:
            mode_map = {"0": "Internal", "1": "External", "2": "Video"}
            sys_info["sync_mode"] = mode_map.get(m.group(1), m.group(1))
        else:
            sys_info["sync_mode"] = None
        sys_info["sync_mode_raw"] = y_line
    else:
        sys_info["sync_mode"] = None
        sys_info["sync_mode_raw"] = None

    # Filters
    sys_info["filters"] = query_filters(ser)

    return sys_info


def query_active_stations(ser: serial.Serial):
    # 'l_station<>' returns station map within the record; we'll use station 1 to read it
    line = send_cmd(ser, "l1", expect=r"\bl\b", read_timeout=0.8)
    active = [False, False, False, False]
    if line:
        # The record lists four 0/1 values (stations 1..4)
        vals = re.findall(r"\b[01]\b", line)
        # Keep last four 0/1 tokens
        ones = [int(v) for v in vals[-4:]] if len(vals) >= 4 else []
        for i, v in enumerate(ones[:4]):
            active[i] = bool(v)
    return active, line


def query_station(ser: serial.Serial, s_idx: int):
    s = str(s_idx)
    st = {"station": s_idx}

    # Hemisphere 'H'
    H_line = send_cmd(ser, f"H{s}", expect=r"\bH\b", read_timeout=0.8)
    st["hemisphere_raw"] = H_line
    st["hemisphere_vector"] = parse_vector_triplet_fields(H_line, 3) if H_line else None

    # Alignment 'A'
    A_line = send_cmd(ser, f"A{s}", expect=r"\bA\b", read_timeout=0.8)
    st["alignment_raw"] = A_line
    # Expect 9 floats: Ox Oy Oz, Xx Xy Xz, Yx Yy Yz
    vals = re.findall(r"[-+]?\d+\.\d+|[-+]?\d+", A_line or "")
    st["alignment"] = None
    if vals and len(vals) >= 9:
        nums = list(map(float, vals[:9]))
        st["alignment"] = {
            "origin": nums[0:3],
            "x_axis_point": nums[3:6],
            "y_axis_point": nums[6:9],
        }

    # Boresight reference 'G'
    G_vals, G_raw = query_simple_value(ser, f"G{s}", "G", parser=float, fallback=None)
    st["boresight_reference_raw"] = G_raw
    st["boresight_reference_angles"] = G_vals[:3] if isinstance(G_vals, list) else None

    # Increment 'I'
    I_vals, I_raw = query_simple_value(ser, f"I{s}", "I", parser=float, fallback=None)
    st["increment_raw"] = I_raw
    st["increment_distance"] = I_vals[0] if isinstance(I_vals, list) and I_vals else None

    # Output list 'O'
    O_list, O_raw = query_station_list_ints(ser, f"O{s}", "O")
    st["output_data_list_raw"] = O_raw
    st["output_data_list_ids"] = O_list

    # Position envelope 'V'
    V_vals, V_raw = query_simple_value(ser, f"V{s}", "V", parser=float, fallback=None)
    st["position_envelope_raw"] = V_raw
    if isinstance(V_vals, list) and len(V_vals) >= 6:
        st["position_envelope"] = {
            "x_max": V_vals[0], "y_max": V_vals[1], "z_max": V_vals[2],
            "x_min": V_vals[3], "y_min": V_vals[4], "z_min": V_vals[5],
        }
    else:
        st["position_envelope"] = None

    # Angular envelope 'Q'
    Q_vals, Q_raw = query_simple_value(ser, f"Q{s}", "Q", parser=float, fallback=None)
    st["angular_envelope_raw"] = Q_raw
    if isinstance(Q_vals, list) and len(Q_vals) >= 6:
        st["angular_envelope"] = {
            "az_max": Q_vals[0], "el_max": Q_vals[1], "rl_max": Q_vals[2],
            "az_min": Q_vals[3], "el_min": Q_vals[4], "rl_min": Q_vals[5],
        }
    else:
        st["angular_envelope"] = None

    # Transmitter mounting frame 'r'
    r_vals, r_raw = query_simple_value(ser, f"r{s}", "r", parser=float, fallback=None)
    st["transmitter_mounting_raw"] = r_raw
    st["transmitter_mounting_AER"] = r_vals[:3] if isinstance(r_vals, list) else None

    # Tip offsets 'N' (may be meaningful when a stylus is used)
    N_vals, N_raw = query_simple_value(ser, f"N{s}", "N", parser=float, fallback=None)
    st["tip_offsets_raw"] = N_raw
    st["tip_offsets"] = N_vals[:3] if isinstance(N_vals, list) else None

    return st


def main():
    ap = argparse.ArgumentParser(description="Read FASTRAK settings and save to JSON.")
    ap.add_argument("--port", required=True, help="Serial port (e.g., COM3 or /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=9600, help="Baud rate (default: 115200)")
    ap.add_argument("--out", required=True, help="Output JSON file path")
    ap.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout (s)")
    args = ap.parse_args()

    try:
        ser = open_port(args.port, args.baud, timeout=args.timeout)
    except Exception as e:
        print(f"ERROR: Could not open port {args.port} @ {args.baud}: {e}", file=sys.stderr)
        sys.exit(2)

    try:
        ensure_ascii_and_quiet(ser)

        data = {
            "collected_at_utc": datetime.utcnow().isoformat() + "Z",
            "serial_port": args.port,
            "serial_baud": args.baud,
            "system": query_system(ser),
        }

        active, l_raw = query_active_stations(ser)
        data["stations_active_raw"] = l_raw
        data["stations_active"] = active

        data["stations"] = {}
        for i in range(1, 5):
            if active[i-1]:
                try:
                    data["stations"][str(i)] = query_station(ser, i)
                except TimeoutError as te:
                    data["stations"][str(i)] = {"station": i, "error": str(te)}
                except Exception as ex:
                    data["stations"][str(i)] = {"station": i, "error": repr(ex)}
            else:
                data["stations"][str(i)] = {"station": i, "active": False}

        # Done—write JSON
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        print(f"Saved settings to {args.out}")

    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()