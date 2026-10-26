#!/bin/bash
# Run the TinyRex stages in order, from START (default 0), stopping at the first stage that saves nothing.
#   POLYKROM_CLI=/path/to/polykrom-cli harness/run_all.sh      or      harness/run_all.sh /path/to/polykrom-cli
# The work directory is TINYREX_WORK (default: work/ in the repo).
set -u
H="$(cd "$(dirname "$0")" && pwd)"
CLI="${1:-${POLYKROM_CLI:-}}"
[ -n "$CLI" ] || { echo "usage: $0 /path/to/polykrom-cli   (or set POLYKROM_CLI)" >&2; exit 2; }
WORK="${TINYREX_WORK:-$H/../work}"; mkdir -p "$WORK"; export TINYREX_WORK="$WORK"
stage() { python3 -c "import sys; sys.path.insert(0, '$H/../spec'); import stages; print($1)"; }
n=$(stage "len(stages.STAGES)")
for ((i=${START:-0}; i<n; i++)); do
  python3 "$H/run_stage.py" "$CLI" $i > "$WORK/stage$i.out" 2>&1
  id=$(stage "stages.STAGES[$i]['id']")
  python3 -c "import json,sys; r=json.load(open('$WORK/$id.json')); print(r['stage'], r['seconds'], 's', r['cost'], 'USD saved', r['saved'], 'errors', r['tool_errors']); sys.exit(0 if r['saved'] else 1)" || exit 1
done
