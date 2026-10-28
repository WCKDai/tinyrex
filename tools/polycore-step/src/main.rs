//! polycore-step RECIPE.json OUT.step [--report REPORT.json] [--threads N] [--only NAME[,NAME...]] [--timeout-s SECONDS]
//!
//! Exit status: 0 when every part was built (exactly or faceted) and the STEP validated and read
//! back with the expected solids; 2 when the STEP was written but a part failed or the check did
//! not pass; 1 on a usage, recipe or write error.

use std::process::ExitCode;
use std::time::{Duration, Instant};

use polycore_step as ps;

const USAGE: &str =
    "usage: polycore-step RECIPE.json OUT.step [--report REPORT.json] [--threads N] [--only NAME[,NAME...]] [--timeout-s SECONDS]";

struct Args {
    recipe: String,
    out: String,
    report: Option<String>,
    opts: ps::Options,
}

fn parse_args() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    let mut pos = Vec::new();
    let mut report = None;
    let mut opts = ps::Options::default();
    while let Some(a) = it.next() {
        let (key, inline) = match a.split_once('=') {
            Some((k, v)) if k.starts_with("--") => (k.to_string(), Some(v.to_string())),
            _ => (a.clone(), None),
        };
        let mut value = |name: &str| inline.clone().or_else(|| it.next()).ok_or_else(|| format!("{name} needs a value"));
        match key.as_str() {
            "-h" | "--help" => return Err(String::new()),
            "--report" => report = Some(value("--report")?),
            "--threads" => {
                opts.threads = value("--threads")?.parse::<usize>().map_err(|e| format!("--threads: {e}"))?.max(1);
            }
            "--only" => {
                let v = value("--only")?;
                opts.only = Some(v.split(',').map(str::trim).filter(|s| !s.is_empty()).map(String::from).collect());
            }
            "--timeout-s" => {
                let s: f64 = value("--timeout-s")?.parse().map_err(|e| format!("--timeout-s: {e}"))?;
                if !(s.is_finite() && s > 0.0) {
                    return Err("--timeout-s must be positive".into());
                }
                opts.timeout = Duration::from_secs_f64(s);
            }
            k if k.starts_with("--") => return Err(format!("unknown option {k}")),
            _ => pos.push(a),
        }
    }
    let [recipe, out] = <[String; 2]>::try_from(pos).map_err(|_| "expected RECIPE.json and OUT.step".to_string())?;
    Ok(Args { recipe, out, report, opts })
}

fn main() -> ExitCode {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            if !e.is_empty() {
                eprintln!("polycore-step: {e}");
            }
            eprintln!("{USAGE}");
            return ExitCode::from(1);
        }
    };
    // Panics are caught and reported per part; keep stderr to the summary unless debugging.
    if std::env::var_os("POLYCORE_STEP_DEBUG").is_none() {
        std::panic::set_hook(Box::new(|_| {}));
    }
    match run(&args) {
        Ok(code) => {
            // Abandoned (timed-out) workers may still be running: end the process without them.
            std::process::exit(code)
        }
        Err(e) => {
            eprintln!("polycore-step: {e}");
            ExitCode::from(1)
        }
    }
}

fn run(args: &Args) -> Result<i32, String> {
    let t0 = Instant::now();
    let text = std::fs::read_to_string(&args.recipe).map_err(|e| format!("{}: {e}", args.recipe))?;
    let recipe = ps::parse_recipe(&text)?;
    if let Some(only) = &args.opts.only {
        for n in only.iter().filter(|n| !recipe.parts.iter().any(|p| &p.name == *n)) {
            eprintln!("polycore-step: warning: --only {n}: no such part");
        }
    }
    let mut results = ps::build_parts(&recipe, &args.opts);
    let file_name = std::path::Path::new(&args.out).file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
    // The report is written whatever happens to the STEP file.
    let (asm_warnings, export_error, check) = match ps::assemble_checked(&recipe, &mut results, &file_name) {
        Ok(asm) => {
            for w in &asm.warnings {
                eprintln!("polycore-step: warning: {w}");
            }
            std::fs::write(&args.out, &asm.text).map_err(|e| format!("{}: {e}", args.out))?;
            let check = ps::validate(&asm.text, asm.solids);
            eprintln!(
                "step: {} ({} bytes) validate: {} reimport: {}/{} solids{}",
                args.out,
                asm.text.len(),
                match (&check.entities, &check.error) {
                    (Some(n), _) => format!("ok ({n} entities)"),
                    (_, Some(e)) => format!("FAILED ({e})"),
                    _ => "?".into(),
                },
                check.solids_read.map_or("-".into(), |n| n.to_string()),
                check.solids_expected,
                check.import_error.as_ref().map_or(String::new(), |e| format!(" (import error: {e})")),
            );
            (asm.warnings, None, Some(check))
        }
        Err(e) => {
            eprintln!("polycore-step: {e}; no STEP written");
            (vec![], Some(e), None)
        }
    };
    for r in &results {
        eprintln!("{}", ps::summary_line(r));
    }
    let totals = ps::totals(&results, t0.elapsed().as_secs_f64());
    eprintln!(
        "total: {} parts: {} exact, {} exact_with_warnings, {} faceted, {} failed; {:.1} mm3; {:.2}s",
        totals.parts, totals.exact, totals.exact_with_warnings, totals.faceted, totals.failed, totals.volume_mm3, totals.seconds
    );
    if let Some(path) = &args.report {
        let report = ps::Report {
            name: &recipe.name,
            output: &args.out,
            parts: &results,
            totals: totals.clone(),
            assembly_warnings: &asm_warnings,
            export_error: export_error.as_deref(),
            validation: check.as_ref(),
        };
        let json = serde_json::to_string_pretty(&report).map_err(|e| format!("report: {e}"))?;
        std::fs::write(path, json + "\n").map_err(|e| format!("{path}: {e}"))?;
    }
    let ok = totals.failed == 0 && check.as_ref().is_some_and(|c| c.ok());
    Ok(if ok { 0 } else { 2 })
}
