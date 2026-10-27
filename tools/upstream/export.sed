# transform.sed's inverse: this library's files as RadioFoundry Gateway's lr2021 component has them (export.sh).

s/SPDX-FileCopyrightText: 2026 Spencer Fowler/SPDX-FileCopyrightText: 2026 The RadioFoundry Gateway authors/
s#docs/iq\.md#docs/spec/lr2021-iq.md#g
s/archive: liblib\.a/archive: liblr2021.a/
s/lr20xx/lr2021/g
s/LR20XX_/LR2021_/g
