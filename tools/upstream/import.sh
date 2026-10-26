#!/bin/sh
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Spencer Fowler
# import.sh UPSTREAM_DIR OUT_DIR: the "sed" files of files.txt from a RadioFoundry Gateway tree, through
# transform.sed, into OUT_DIR at their paths here. "hand" files are copied as they are, under OUT_DIR/hand/.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
src=$1
out=$2
grep -v '^#' "$here/files.txt" | while read -r up ours how; do
    [ -n "$up" ] || continue
    [ -f "$src/$up" ] || { echo "missing upstream: $up" >&2; continue; }
    if [ "$how" = sed ]; then
        mkdir -p "$out/$(dirname "$ours")"
        sed -f "$here/transform.sed" "$src/$up" >"$out/$ours"
    else
        mkdir -p "$out/hand/$(dirname "$up")"
        cp "$src/$up" "$out/hand/$up"
    fi
done
