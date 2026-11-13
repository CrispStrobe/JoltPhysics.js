#!/usr/bin/env python3
"""Repair the known Emscripten 6.0.2 Node pthread footer precedence error.

Only the exact trailing detector is changed. The factory/body (including any
embedded Wasm) remains byte-identical. Refuse unknown or ambiguous output.
"""
import argparse
import hashlib
import json
from pathlib import Path

EXPORT = b"export default Jolt;"
BAD = (
    b'await import("node:worker_threads").workerData==="em-pthread"',
    b"await import('node:worker_threads').workerData==='em-pthread'",
)
GOOD = (
    b'(await import("node:worker_threads")).workerData==="em-pthread"',
    b"(await import('node:worker_threads')).workerData==='em-pthread'",
    b"(await import('node:worker_threads')).workerData === 'em-pthread'",
)

def repair(data):
    if data.count(EXPORT) != 1:
        raise ValueError("expected exactly one Jolt default export")
    body, footer = data.split(EXPORT)
    # Restrict the match to the generated bootstrap, never an embedded module.
    if footer.count(b"isPthread") != 3 or not footer.rstrip().endswith((b"isPthread&&Jolt();", b"isPthread && Jolt();")):
        raise ValueError("unknown Node pthread bootstrap shape")
    bad = sum(footer.count(token) for token in BAD)
    good = sum(footer.count(token) for token in GOOD)
    if bad == 1 and good == 0:
        for before, after in zip(BAD, GOOD):
            if before in footer:
                footer = footer.replace(before, after, 1)
                break
        result = body + EXPORT + footer
        if result.split(EXPORT)[0] != body:
            raise AssertionError("factory/body changed")
        return result, "corrected"
    if bad == 0 and good == 1:
        return data, "already-correct"
    raise ValueError("missing or ambiguous Node pthread detector")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("module", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    original = args.module.read_bytes()
    corrected, status = repair(original)
    target = args.output or args.module
    if target != args.module or corrected != original:
        target.write_bytes(corrected)
    print(json.dumps({"file": target.name, "status": status,
                      "beforeSha256": hashlib.sha256(original).hexdigest(),
                      "afterSha256": hashlib.sha256(corrected).hexdigest(),
                      "factoryBodyUnchanged": True}))

if __name__ == "__main__":
    main()
