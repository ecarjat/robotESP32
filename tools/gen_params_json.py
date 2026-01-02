#!/usr/bin/env python3
import argparse
import os
import shutil
import subprocess
import sys
import tempfile


def find_compiler():
    for name in ("cc", "clang", "gcc"):
        path = shutil.which(name)
        if path:
            return path
    return None


def main():
    parser = argparse.ArgumentParser(description="Generate cli/params.json from param_storage.h")
    parser.add_argument("--out", default="cli/params.json", help="Output JSON path")
    parser.add_argument("--check", action="store_true", help="Fail if output differs")
    args = parser.parse_args()

    compiler = find_compiler()
    if not compiler:
        print("No C compiler found (cc/clang/gcc).", file=sys.stderr)
        return 1

    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    src = os.path.join(root, "tools", "gen_params_json.c")
    if not os.path.exists(src):
        print("Missing tools/gen_params_json.c", file=sys.stderr)
        return 1

    out_path = os.path.join(root, args.out)
    with tempfile.TemporaryDirectory(dir=root) as tmpdir:
        bin_path = os.path.join(tmpdir, "gen_params_json")
        cmd = [compiler, "-std=c11", "-Wall", "-Wextra", src, "-o", bin_path]
        try:
            subprocess.check_call(cmd, cwd=root)
        except subprocess.CalledProcessError:
            return 1

        try:
            generated = subprocess.check_output([bin_path], cwd=root)
        except subprocess.CalledProcessError:
            return 1

    if os.path.exists(out_path):
        with open(out_path, "rb") as f:
            current = f.read()
        if args.check:
            if current != generated:
                print("params.json out of date. Run tools/gen_params_json.py.", file=sys.stderr)
                return 1
            print("params.json up to date.")
            return 0
        if current == generated:
            print("params.json up to date.")
            return 0

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "wb") as f:
        f.write(generated)
    print("Wrote {}".format(os.path.relpath(out_path, root)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
