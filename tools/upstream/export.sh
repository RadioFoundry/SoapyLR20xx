#!/bin/sh
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Spencer Fowler
# export.sh UPSTREAM_DIR: this library's "sed" files of files.txt, through export.sed, into a RadioFoundry Gateway
# tree at their paths there (overwriting them). Then check that import.sh gives them back unchanged.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
dst=$1
grep -v '^#' "$here/files.txt" | while read -r up ours how; do
    [ -n "$up" ] && [ "$how" = sed ] || continue
    mkdir -p "$dst/$(dirname "$up")"
    sed -f "$here/export.sed" "$root/$ours" >"$dst/$up"
    sed -f "$here/transform.sed" "$dst/$up" | cmp -s - "$root/$ours" || echo "not round trip: $ours" >&2
done
