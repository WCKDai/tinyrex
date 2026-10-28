#!/usr/bin/env python3
"""Run one TinyRex stage: Claude Code headless with Polykrom's MCP server as its only tool.

    run_stage.py POLYKROM_CLI STAGE_INDEX [--work DIR]

The stage reads the previous stage's design from the work directory and writes its own design, a JSON report
(<stage>.json) and a transcript (<stage>.transcript.jsonl) there. Environment:

    TINYREX_WORK            work directory (default: ./work next to this repo's harness/); --work overrides it
    POLYKROM_MUJOCO_PYTHON  a Python with mujoco and numpy, for sim.mujoco (Polykrom's default: python3)
    TINYREX_TIMEOUT         seconds before a stage is killed (default 10800)
    TINYREX_MAX_TURNS       the agent's turn limit per stage (default 300)
    CONTINUE=1              carry on from this stage's own partial save instead of starting it over
"""
import argparse, json, os, subprocess, sys, time
from pathlib import Path

K = Path(__file__).resolve().parent
sys.path.insert(0, str(K.parent / "spec")); import stages

ap = argparse.ArgumentParser(description="Run one TinyRex stage headless against polykrom-cli mcp.")
ap.add_argument("cli", help="path to the polykrom-cli binary")
ap.add_argument("index", type=int, help=f"stage index, 0..{len(stages.STAGES) - 1}")
ap.add_argument("--work", default=os.environ.get("TINYREX_WORK", str(K.parent / "work")), help="work directory")
a = ap.parse_args()
cli, idx = str(Path(a.cli).resolve()), a.index
st = stages.STAGES[idx]
work = Path(a.work).resolve(); work.mkdir(parents=True, exist_ok=True)
save = work / f"{st['id']}.polykrom"
prev = work / f"{stages.STAGES[idx - 1]['id']}.polykrom" if idx else None
cfg = work / "mcp.json"
cfg.write_text(json.dumps({"mcpServers": {"polykrom": {"command": cli, "args": ["mcp", "--headless", "--allow-path", str(work)]}}}))
if os.environ.get("CONTINUE") and save.exists():
    # The stage was cut short: carry on from what it saved.
    prev = save
    st = dict(st, prompt="This stage was interrupted before it finished; its work so far is saved in {input}. Open it, inspect it "
              "(inspect_design, joint.list) to see exactly what is already done and verified, keep that, and finish the rest of the task: "
              + st["prompt"].replace("Open {input}", "The task, from the start: open the design"))
prompt = (st["prompt"] + stages.COMMON).replace("{save}", str(save)).replace("{input}", str(prev or "")) \
    .replace("{gif}", str(work / "rex-walk.gif")).replace("{png}", str(work / "rex-render.png")).replace("{bom}", str(work / "rex-bom.csv")).replace("{mjcf}", str(work / "rex-mjcf" / "rex.xml")).replace("{polished}", str(work / "rex-polished.png"))

env = {k: v for k, v in os.environ.items() if k != "CLAUDECODE"}  # POLYKROM_MUJOCO_PYTHON passes through to sim.mujoco
timeout = int(os.environ.get("TINYREX_TIMEOUT", "10800"))
t0 = time.time()
log = work / f"{st['id']}.transcript.jsonl"
if os.environ.get("CONTINUE") and log.exists():
    # Keep the interrupted run's transcript: the continuation writes its own.
    n = 1
    while (part := work / f"{st['id']}.transcript.part{n}.jsonl").exists():
        n += 1
    log.rename(part)
cmd = ["claude", "-p", prompt, "--mcp-config", str(cfg), "--strict-mcp-config", "--allowedTools", "mcp__polykrom", "--disallowedTools", "Bash,Read,Write,Edit,MultiEdit,NotebookEdit,Glob,Grep,WebFetch,WebSearch,Task,LSP",
       "--output-format", "stream-json", "--verbose", "--max-turns", os.environ.get("TINYREX_MAX_TURNS", "300")]
# Streamed to disk as it runs, so a stage that times out keeps its transcript.
with open(log, "w") as out:
    proc = subprocess.Popen(cmd, stdout=out, stderr=subprocess.PIPE, text=True, env=env, cwd=work)
    try:
        _, err = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        _, err = proc.communicate()
        err = f"timed out after {timeout} s; " + (err or "")
class R: pass
r = R(); r.stdout = log.read_text(); r.stderr = err or ""
ev = [json.loads(l) for l in r.stdout.splitlines() if l.startswith("{")]
res = next((e for e in reversed(ev) if e.get("type") == "result"), {})
calls = errs = 0
for e in ev:
    m = e.get("message") if isinstance(e.get("message"), dict) else {}
    for c in m.get("content", []) if isinstance(m.get("content"), list) else []:
        calls += c.get("type") == "tool_use"; errs += bool(c.get("type") == "tool_result" and c.get("is_error"))
rec = {"stage": st["id"], "seconds": round(time.time() - t0), "turns": res.get("num_turns"), "cost": res.get("total_cost_usd"),
       "tool_calls": calls, "tool_errors": errs, "saved": save.exists(), "reply": res.get("result", r.stderr[-500:])}
(work / f"{st['id']}.json").write_text(json.dumps(rec, indent=1))
print(json.dumps(rec, indent=1))
