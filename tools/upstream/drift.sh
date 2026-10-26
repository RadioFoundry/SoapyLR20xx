#!/bin/sh
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2026 Spencer Fowler
# drift.sh [UPSTREAM_REPO [REV]]: how this library and RadioFoundry Gateway's lr2021 component have drifted since
# the import (BASE). Read-only: upstream's trees come from `git archive`, nothing is checked out.
#
#   upstream.diff  upstream's changes since BASE, renamed: to bring here (sed files apply nearly as they are; hand
#                  files are upstream's own diff, to port by hand)
#   ours.diff      our changes to the sed files since BASE: to offer upstream (reverse the rename first)
#
# Default repo ../RadioFoundryGateway next to this one; default REV its HEAD. Output in build/drift/.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
repo=${1:-$root/../RadioFoundryGateway}
rev=${2:-HEAD}
base=$(sed -n 's/^commit //p' "$here/BASE")
work=$root/build/drift
rm -rf "$work"
mkdir -p "$work/base" "$work/head" "$work/base-t" "$work/head-t" "$work/ours"
paths=$(grep -v '^#' "$here/files.txt" | awk 'NF { print $1 }')
# shellcheck disable=SC2086
git -C "$repo" archive "$base" -- $paths | tar -x -C "$work/base"
# shellcheck disable=SC2086
git -C "$repo" archive "$rev" -- $paths | tar -x -C "$work/head"
"$here/import.sh" "$work/base" "$work/base-t"
"$here/import.sh" "$work/head" "$work/head-t"
grep -v '^#' "$here/files.txt" | awk '$3 == "sed" { print $2 }' | while read -r f; do
    mkdir -p "$work/ours/$(dirname "$f")"
    cp "$root/$f" "$work/ours/$f"
done
(cd "$work" && diff -ruN base-t head-t >upstream.diff || true)
(cd "$work" && diff -ruN --exclude=hand base-t ours >ours.diff || true)
echo "BASE $base, upstream $(git -C "$repo" rev-parse --short "$rev")"
printf 'upstream.diff: %s lines\nours.diff:     %s lines\n' "$(wc -l <"$work/upstream.diff")" "$(wc -l <"$work/ours.diff")"
echo "in $work"
