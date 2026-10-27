# The rename from RadioFoundry Gateway's lr2021 component to this library (tools/upstream/README.md); export.sed is
# its inverse. import.sh runs every "sed" file of files.txt through it; keep it mechanical, so a fresh import diffs
# clean.

# Copyright: the same author, named.
s/SPDX-FileCopyrightText: 2026 The RadioFoundry Gateway authors/SPDX-FileCopyrightText: 2026 Spencer Fowler/

# Paths the comments cite, and the component's archive (linker.lf).
s#docs/spec/lr2021-iq\.md#docs/iq.md#g
s#components/lr2021/##g
s/archive: liblr2021\.a/archive: liblib.a/

# Names: the family, not the part. "LR2021" alone (the chip) stays.
s/lr2021/lr20xx/g
s/LR2021_/LR20XX_/g
