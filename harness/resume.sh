#!/bin/bash
# Continue stage index $1 from its partial save, then run the stages after it.
#   POLYKROM_CLI=/path/to/polykrom-cli harness/resume.sh 3      or      harness/resume.sh 3 /path/to/polykrom-cli
set -u
H="$(cd "$(dirname "$0")" && pwd)"
i="${1:?usage: $0 STAGE_INDEX [/path/to/polykrom-cli]}"
CLI="${2:-${POLYKROM_CLI:-}}"
[ -n "$CLI" ] || { echo "usage: $0 STAGE_INDEX /path/to/polykrom-cli   (or set POLYKROM_CLI)" >&2; exit 2; }
WORK="${TINYREX_WORK:-$H/../work}"; mkdir -p "$WORK"; export TINYREX_WORK="$WORK"
id=$(python3 -c "import sys; sys.path.insert(0, '$H/../spec'); import stages; print(stages.STAGES[$i]['id'])")
CONTINUE=1 python3 "$H/run_stage.py" "$CLI" $i > "$WORK/stage${i}c.out" 2>&1
python3 -c "import json,sys; r=json.load(open('$WORK/$id.json')); print(r['stage'], r['seconds'], 's', r['cost'], 'saved', r['saved']); sys.exit(0 if r['saved'] and 'limit' not in (r['reply'] or '') else 1)" || exit 1
START=$((i+1)) "$H/run_all.sh" "$CLI"
