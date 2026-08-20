#!/usr/bin/env python3
"""Guard: the COBS wire protocol cannot change incompatibly without a version bump.

The host↔firmware compatibility key is MOWGLI_PROTOCOL_VERSION (firmware
mowgli_protocol.h) which the host mirrors as kMowgliProtocolVersion
(ll_datatypes.hpp). If a `pkt_*_t` struct or a `PKT_ID_*` id changes but the
version is not bumped, an old host silently mis-parses a new firmware's packets.
This mirrors the codegen-drift guard (sync_ros_lib.py): it fingerprints the
wire-defining region of mowgli_protocol.h and stores it in protocol_baseline.json.

Usage (from repo root):
    protocol_version_guard.py           # refresh baseline (requires a version bump)
    protocol_version_guard.py --check   # CI: FAIL on un-versioned wire drift

--check fails if:
  * the compatibility fingerprint changed vs the baseline (bump
    MOWGLI_PROTOCOL_VERSION and re-run without --check to refresh the baseline),
    OR
  * firmware MOWGLI_PROTOCOL_VERSION != host kMowgliProtocolVersion (lockstep).
"""

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
FW_HEADER = (
    REPO_ROOT
    / "firmware/stm32/ros_usbnode/include/mowgli_protocol.h"
)
HOST_HEADER = (
    REPO_ROOT
    / "ros2/src/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp"
)
BASELINE = Path(__file__).resolve().parent / "protocol_baseline.json"

# Additive packet IDs are allowed when old firmware safely ignores new host
# commands and old hosts safely ignore new telemetry. Keep this list small and
# only use it for optional extensions that do not alter any existing packet
# layout or semantics.
COMPAT_OPTIONAL_PACKET_IDS = {
    "PKT_ID_PERIMETER_WIRE",
    "PKT_ID_PERIMETER_CAPABILITY_RSP",
    "PKT_ID_SET_PERIMETER_LISTEN",
    "PKT_ID_PERIMETER_CAPABILITY_REQ",
}

COMPAT_OPTIONAL_STRUCTS = {
    "pkt_perimeter_wire_t",
    "pkt_perimeter_capability_rsp_t",
    "pkt_set_perimeter_listen_t",
    "pkt_perimeter_capability_req_t",
}


def _strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//[^\n]*", "", text)
    return text


def read_version(header, macro):
    text = header.read_text()
    # Accept both `#define MACRO N` (firmware) and `... MACRO = N;` (host constexpr).
    m = re.search(rf"#define\s+{macro}\s+(\d+)", text) or re.search(
        rf"{macro}\s*=\s*(\d+)", text
    )
    if not m:
        sys.exit(f"ERROR: {macro} not found in {header}")
    return int(m.group(1))


def wire_fingerprint(header_text):
    """sha256 over normalized compatibility tokens.

    The compatibility hash includes every non-optional PKT_ID_* id and
    pkt_*_t struct body. Optional additive extension packets are excluded so
    they can be added without forcing a mower-stopping protocol-version bump.
    Comments + whitespace are stripped and tokens are sorted so cosmetic
    reordering of unrelated lines is stable.
    """
    src = _strip_comments(header_text)
    ids = re.findall(r"#define\s+(PKT_ID_\w+)\s+(0x[0-9A-Fa-f]+u?|\d+u?)", src)
    struct_blocks = re.findall(
        r"typedef\s+struct\s*\{.*?\}\s*pkt_\w+_t\s*;", src, flags=re.DOTALL
    )
    tokens = []
    for name, value in ids:
        if name in COMPAT_OPTIONAL_PACKET_IDS:
            continue
        tokens.append(f"{name}={value.rstrip('u')}")
    for block in struct_blocks:
        m = re.search(r"}\s*(pkt_\w+_t)\s*;", block)
        if m and m.group(1) in COMPAT_OPTIONAL_STRUCTS:
            continue
        tokens.append(re.sub(r"\s+", " ", block).strip())
    tokens.sort()
    joined = "\n".join(tokens)
    return hashlib.sha256(joined.encode()).hexdigest(), len(tokens), len(struct_blocks), len(ids)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="CI mode: no writes, fail on drift")
    args = ap.parse_args()

    fw_version = read_version(FW_HEADER, "MOWGLI_PROTOCOL_VERSION")
    host_version = read_version(HOST_HEADER, "kMowgliProtocolVersion")
    digest, n_tokens, n_structs, n_ids = wire_fingerprint(FW_HEADER.read_text())

    baseline = json.loads(BASELINE.read_text()) if BASELINE.exists() else None

    if args.check:
        ok = True
        if fw_version != host_version:
            print(
                f"FAIL: protocol-version lockstep broken — firmware "
                f"MOWGLI_PROTOCOL_VERSION={fw_version} but host "
                f"kMowgliProtocolVersion={host_version}."
            )
            ok = False
        if baseline is None:
            print("FAIL: no protocol_baseline.json — run the guard without --check.")
            ok = False
        elif digest != baseline["hash"]:
            print(
                "FAIL: COBS wire protocol changed vs baseline (pkt_* struct or "
                f"PKT_ID_* id) at recorded version {baseline['version']}.\n"
                "  -> bump MOWGLI_PROTOCOL_VERSION (firmware) AND "
                "kMowgliProtocolVersion (host) in lockstep, then run "
                "`protocol_version_guard.py` (no --check) to refresh the baseline."
            )
            ok = False
        if ok:
            print(
                f"OK: protocol v{fw_version} (host+firmware), "
                f"{n_tokens} compatibility tokens "
                f"({n_structs} structs / {n_ids} ids total), fingerprint matches baseline."
            )
        return 0 if ok else 1

    # Refresh mode: only permitted alongside a version bump.
    if baseline is not None and digest != baseline["hash"] and fw_version == baseline["version"]:
        sys.exit(
            f"ERROR: refusing to refresh the baseline without a version bump — "
            f"the wire changed but MOWGLI_PROTOCOL_VERSION is still "
            f"{fw_version}. Bump it (and host kMowgliProtocolVersion) first."
        )
    if fw_version != host_version:
        sys.exit(
            f"ERROR: firmware ({fw_version}) and host ({host_version}) protocol "
            "versions differ; align them before refreshing the baseline."
        )
    BASELINE.write_text(
        json.dumps({"version": fw_version, "hash": digest}, indent=2) + "\n"
    )
    print(f"wrote baseline: version {fw_version}, fingerprint {digest[:12]}…")
    return 0


if __name__ == "__main__":
    sys.exit(main())
