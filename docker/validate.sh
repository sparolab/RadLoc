#!/bin/bash
# Runs every check the repository ships, against the mounted datasets.
#
#   validate.sh [sequence ...]     default: KAIST_03 Riverside_03
set -uo pipefail

CORE=${RADLOC_ROOT:-/radloc}/cpp/radloc
BIN=${RADLOC_CORE_BUILD:-/opt/radloc/core-build}
MULRAN=${RADLOC_MULRAN:-/data/referee/Mulran}
SEQS=("${@:-}")
[ -z "${SEQS[*]}" ] && SEQS=(KAIST_03 Riverside_03)

failures=0
step() { printf '\n\033[1m== %s\033[0m\n' "$1"; }
check() { if [ "$1" -eq 0 ]; then echo "   PASS"; else echo "   FAIL"; failures=$((failures+1)); fi; }

for seq in "${SEQS[@]}"; do
  polar=$MULRAN/$seq/polar
  npy=$MULRAN/$seq/refereepp_84x4
  if [ ! -d "$polar" ] || [ ! -d "$npy" ]; then
    echo "skip $seq (missing $polar or $npy)"; continue
  fi

  step "$seq — descriptor vs archived .npy"
  "$BIN/validate_descriptor" "$polar" "$npy" 50
  check $?

  step "$seq — retrieval vs Python reference"
  for dims in 20 8; do
    for k in 10 5; do
      "$BIN/validate_retrieval" "$npy" 400 "$dims" "$k" > /tmp/cpp.txt
      python3 "$CORE/tools/reference_retrieval.py" "$npy" 400 "$dims" "$k" > /tmp/py.txt
      if diff -q /tmp/cpp.txt /tmp/py.txt > /dev/null; then
        echo "   coarse_dims=$dims top_k=$k  identical"
      else
        echo "   coarse_dims=$dims top_k=$k  DIFFER"; failures=$((failures+1))
      fi
    done
  done

  step "$seq — phase-correlation registration"
  first=$(find "$polar" -name '*.png' | sort | head -1)
  "$BIN/test_phase_corr" "$first"
  check $?
done

printf '\n'
if [ "$failures" -eq 0 ]; then echo "all checks passed"; else echo "$failures check(s) failed"; fi
exit $((failures == 0 ? 0 : 1))
