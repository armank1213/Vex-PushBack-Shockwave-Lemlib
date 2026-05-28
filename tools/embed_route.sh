#!/usr/bin/env bash
# Bake a route file (planner export) into a C++ header so the robot can
# replay it without an SD card. Re-run after every new export.
#
#   tools/embed_route.sh [src] [out]
#   defaults: src=path/dtData.txt  out=include/robot/embedded_route.hpp
set -euo pipefail

SRC="${1:-path/dtData.txt}"
OUT="${2:-include/robot/embedded_route.hpp}"

if [[ ! -f "$SRC" ]]; then
  echo "error: $SRC not found" >&2
  exit 1
fi

{
  echo '#pragma once'
  echo "// AUTO-GENERATED from $SRC by tools/embed_route.sh — do not hand-edit."
  echo '// Re-run tools/embed_route.sh after exporting a new route.'
  echo 'namespace route {'
  echo 'inline const char* EMBEDDED_DATA = R"DTDATA('
  cat "$SRC"
  echo ')DTDATA";'
  echo '} // namespace route'
} > "$OUT"

echo "wrote $OUT  ($(grep -c . "$SRC") non-empty lines from $SRC)"
