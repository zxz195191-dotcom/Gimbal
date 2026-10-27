"""Merge a frozen #YD_A/B/C serial capture into one CSV; reject missing rows."""
import argparse
import csv
import json
from pathlib import Path


def decode(path):
    captures = []
    current = None
    headers = {}
    rows = {}
    for number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        parts = next(csv.reader([line]))
        if not parts:
            continue
        tag = parts[0]
        if tag == "#YD_BEGIN":
            if current is not None:
                raise ValueError(f"line {number}: previous capture has no #YD_END")
            current = dict(item.split("=", 1) for item in parts[1:])
            if current.get("v") != "1":
                raise ValueError("unsupported diagnostic schema")
            headers, rows = {}, {}
        elif tag.startswith("#YD_") and tag.endswith("_FIELDS"):
            if current is None:
                raise ValueError(f"line {number}: header outside capture")
            headers[tag[4]] = parts[1:]
        elif tag in ("#YD_A", "#YD_B", "#YD_C"):
            group = tag[-1]
            if current is None or group not in headers:
                raise ValueError(f"line {number}: data without header")
            values = parts[1:]
            if len(values) != len(headers[group]):
                raise ValueError(f"line {number}: incorrect column count")
            index = int(values[0])
            if group in rows.setdefault(index, {}):
                raise ValueError(f"line {number}: duplicate index/group")
            rows[index][group] = dict(zip(headers[group], values))
        elif tag == "#YD_END":
            if current is None:
                raise ValueError(f"line {number}: unexpected end")
            count = int(current["count"])
            if set(rows) != set(range(count)):
                raise ValueError("capture has missing or out-of-range frames")
            merged = []
            for index in range(count):
                if set(rows[index]) != set("ABC"):
                    raise ValueError(f"frame {index}: missing A/B/C group")
                row = {"capture": len(captures)}
                for group in "ABC":
                    row.update(rows[index][group])
                merged.append(row)
            captures.append((current, merged))
            current = None
    if current is not None:
        raise ValueError("truncated capture: no #YD_END")
    if not captures:
        raise ValueError("no complete diagnostic capture found")
    return captures


def summarize(meta, rows):
    end = float(rows[-1]["time_ms"])
    stable = [r for r in rows if float(r["time_ms"]) >= end - 500]
    errors = [float(r["target_deg"]) - float(r["encoder_deg"]) for r in stable]
    return {
        **meta,
        "last_500ms_mean_error_deg": sum(errors) / len(errors),
        "last_500ms_max_abs_rate_deg_s": max(abs(float(r["rate_deg_s"])) for r in stable),
        "last_500ms_mean_total_est_nm": sum(float(r["total_est_nm"]) for r in stable) / len(stable),
        "max_control_dt_ms": max(int(r["dt_ms"]) for r in rows),
        "friction_cut_frames": sum(bool(int(r["flags"]) & 64) for r in rows),
        "torque_clip_frames": sum(bool(int(r["flags"]) & 32) for r in rows),
        "unsent_frames": sum(int(r["tx_result"]) != 0 for r in rows),
        "note": "PD estimates are computed from packed commands and captured motor feedback, not measured output torque.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    args = parser.parse_args()
    captures = decode(args.capture)
    rows = [row for _, items in captures for row in items]
    with args.output.open("w", encoding="utf-8-sig", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = [summarize(meta, items) for meta, items in captures]
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
