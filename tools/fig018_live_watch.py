#!/usr/bin/env python3
"""Tail welle-cli dump.fic and report any FIG 0/18 (Announcement support)."""
from __future__ import annotations

import json
import os
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

WATCH = Path(os.environ.get("WATCH_DIR", ".")).resolve()
DUMP = WATCH / "dump.fic"
STATUS = WATCH / "status.json"
HITS = WATCH / "fig018_hits.jsonl"
SUMMARY = WATCH / "summary.json"

# EN 300 401 Table 15 — ASu flags, bit 0 = LSB
ASU_BITS = {
    0: "Alarm",
    1: "Road Traffic flash",
    2: "Transport flash",
    3: "Warning/Service",
    4: "News flash",
    5: "Weather flash",
    6: "Event announcement",
    7: "Special event",
    8: "Programme Information",
    9: "Sport report",
    10: "Financial report",
    11: "Reserved",
    12: "Reserved",
    13: "Reserved",
    14: "Reserved",
    15: "Reserved",
}

DURATION_S = int(os.environ.get("WATCH_SECONDS", "3900"))  # >= 1h
POLL_S = 2.0


def asu_names(flags: int) -> list[str]:
    return [name for bit, name in ASU_BITS.items() if flags & (1 << bit) and not name.startswith("Reserved")]


def parse_fig018_payload(pd: int, payload: bytes) -> list[dict]:
    out = []
    i = 0
    while True:
        sid_len = 4 if pd else 2
        if i + sid_len + 3 > len(payload):
            break
        if pd:
            sid = int.from_bytes(payload[i : i + 4], "big")
            i += 4
        else:
            sid = int.from_bytes(payload[i : i + 2], "big")
            i += 2
        if i + 3 > len(payload):
            break
        asu = int.from_bytes(payload[i : i + 2], "big")
        i += 2
        n_clusters = payload[i] & 0x1F
        i += 1
        clusters = list(payload[i : i + n_clusters])
        i += n_clusters
        out.append(
            {
                "sid": sid,
                "sid_hex": f"0x{sid:04X}" if sid <= 0xFFFF else f"0x{sid:08X}",
                "asu_flags": asu,
                "asu_hex": f"0x{asu:04X}",
                "announcement_types": asu_names(asu),
                "clusters": clusters,
            }
        )
    return out


def scan_fibs(data: bytes, offset: int) -> tuple[int, list[dict], Counter]:
    """Return (new_offset, fig018_entries, ext_counts_delta)."""
    entries: list[dict] = []
    ext_counts: Counter = Counter()
    # Align to 32-byte FIB boundary from start of file
    start = offset - (offset % 32) if offset % 32 else offset
    pos = start
    end = len(data) - (len(data) % 32)
    while pos + 32 <= end:
        fib = data[pos : pos + 32]
        pos += 32
        p = 0
        while p < 30:
            t = (fib[p] >> 5) & 7
            ln = fib[p] & 0x1F
            if t == 7 or ln == 0:
                break
            if p + 1 + ln > 30:
                break
            body = fib[p + 1 : p + 1 + ln]
            p += 1 + ln
            if t != 0 or not body:
                continue
            ext = body[0] & 0x1F
            pd = (body[0] >> 5) & 1
            ext_counts[ext] += 1
            if ext != 18:
                continue
            for item in parse_fig018_payload(pd, body[1:]):
                item["pd"] = pd
                item["raw"] = body[1:].hex()
                entries.append(item)
    return end, entries, ext_counts


def main() -> None:
    t0 = time.time()
    deadline = t0 + DURATION_S
    offset = 0
    fibs_seen = 0
    fig018_total = 0
    unique: dict[str, dict] = {}
    ext_totals: Counter = Counter()
    last_cleanup = 0.0
    dump_appeared = False
    ensemble_locked = False

    print(f"watch start utc={datetime.now(timezone.utc).isoformat()} duration_s={DURATION_S}", flush=True)

    while time.time() < deadline:
        # scrub bulky -D side products
        now = time.time()
        if now - last_cleanup > 15:
            for pat in ("*.wav", "*.msc"):
                for f in WATCH.glob(pat):
                    try:
                        f.unlink()
                    except OSError:
                        pass
            last_cleanup = now

        if not DUMP.exists():
            time.sleep(POLL_S)
            _write_status(
                t0,
                deadline,
                fibs_seen,
                fig018_total,
                unique,
                ext_totals,
                dump_appeared,
                "waiting_for_dump.fic",
            )
            continue

        dump_appeared = True
        size = DUMP.stat().st_size
        if size < offset:
            # truncated/rotated
            offset = 0
        if size - offset < 32:
            time.sleep(POLL_S)
            _write_status(
                t0,
                deadline,
                fibs_seen,
                fig018_total,
                unique,
                ext_totals,
                dump_appeared,
                "locked_receiving" if fibs_seen else "dump_growing",
            )
            continue

        with DUMP.open("rb") as f:
            f.seek(offset)
            chunk = f.read()
        new_off, entries, ext_delta = scan_fibs(chunk, 0)
        # new_off is relative to chunk start; absolute = offset + new_off only if offset%32==0
        abs_new = offset + new_off
        fibs_seen += (abs_new - offset) // 32
        offset = abs_new
        ext_totals.update(ext_delta)

        if ext_totals.get(0) or ext_totals.get(1) or ext_totals.get(2):
            ensemble_locked = True

        for e in entries:
            fig018_total += 1
            key = f"{e['sid_hex']}:{e['asu_hex']}"
            if key not in unique:
                unique[key] = {**e, "first_seen_utc": datetime.now(timezone.utc).isoformat(), "count": 1}
                with HITS.open("a") as hf:
                    hf.write(json.dumps(unique[key]) + "\n")
                print(
                    f"FIG0/18 HIT sid={e['sid_hex']} asu={e['asu_hex']} types={e['announcement_types']} clusters={e['clusters']}",
                    flush=True,
                )
            else:
                unique[key]["count"] += 1

        # Do NOT truncate/rewrite dump.fic while welle-cli holds it open.
        # Rewriting races with welle's FD offset and creates a sparse/zero-padded
        # gap; every 32-byte block then inflates fibs_scanned while FIG extension
        # counters stay real. Prefer a dedicated dump path and delete the file
        # between runs instead.

        _write_status(
            t0,
            deadline,
            fibs_seen,
            fig018_total,
            unique,
            ext_totals,
            dump_appeared,
            "receiving" if ensemble_locked else "acquiring",
        )
        time.sleep(POLL_S)

    elapsed = time.time() - t0
    appeared = fig018_total > 0
    summary = {
        "question": "Did FIG 0/18 appear at any point during the watch?",
        "answer": "yes" if appeared else "no",
        "fig_0_18_appeared": appeared,
        "fig_0_18_occurrence_count": fig018_total,
        "unique_service_bitmaps": list(unique.values()),
        "elapsed_seconds": round(elapsed, 1),
        "fibs_scanned": fibs_seen,
        "fig0_extension_totals": {str(k): v for k, v in sorted(ext_totals.items())},
        "fig_0_18_count_in_totals": ext_totals.get(18, 0),
        "fig_0_19_count_in_totals": ext_totals.get(19, 0),
        "fig_0_24_count_in_totals": ext_totals.get(24, 0),
        "ended_utc": datetime.now(timezone.utc).isoformat(),
        "channel": "13E",
        "expected_eid": "0xF501",
    }
    SUMMARY.write_text(json.dumps(summary, indent=2) + "\n")
    _write_status(t0, deadline, fibs_seen, fig018_total, unique, ext_totals, dump_appeared, "done")
    print(json.dumps(summary, indent=2), flush=True)
    print(f"ANSWER: {'YES' if appeared else 'NO'}", flush=True)


def _write_status(t0, deadline, fibs, hits, unique, ext_totals, dump_ok, phase):
    STATUS.write_text(
        json.dumps(
            {
                "phase": phase,
                "elapsed_s": round(time.time() - t0, 1),
                "remaining_s": round(max(0, deadline - time.time()), 1),
                "fibs_scanned": fibs,
                "fig018_hits": hits,
                "unique": list(unique.values()),
                "ext18": ext_totals.get(18, 0),
                "ext19": ext_totals.get(19, 0),
                "ext24": ext_totals.get(24, 0),
                "dump_ok": dump_ok,
                "utc": datetime.now(timezone.utc).isoformat(),
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
