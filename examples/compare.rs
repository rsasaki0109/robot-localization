//! Every method on every condition over several seeds: a Markdown table of
//! errors, of how often an arbiter hands over, and of what the estimate
//! rested on.
//!
//! ```bash
//! cargo run --release --example compare            # 3 seeds
//! cargo run --release --example compare -- 10      # 10 seeds
//! ```

use robo_localization::methods::Start;
use robo_localization::sim::{all_methods, run, Condition, Report};

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn main() {
    let seeds: u64 = std::env::args().nth(1).map_or(3, |s| s.parse().expect("seed count"));
    let method_count = all_methods(Start::known(Default::default())).len();
    // reports[condition][method][seed]
    let reports: Vec<Vec<Vec<Report>>> = Condition::ALL
        .iter()
        .map(|condition| {
            let city = condition.city();
            (0..method_count)
                .map(|m| {
                    (0..seeds)
                        .map(|seed| {
                            let mut localizer = all_methods(condition.start(&city, seed)).swap_remove(m);
                            run(localizer.as_mut(), &city, *condition, seed, false)
                        })
                        .collect()
                })
                .collect()
        })
        .collect();
    println!("RMSE, median over {seeds} seeds (worst seed's in brackets):\n");
    let names: Vec<&str> = Condition::ALL.iter().map(|c| c.name).collect();
    println!("| Method | {} |", names.join(" | "));
    println!("|---|{}", "---|".repeat(names.len()));
    for m in 0..method_count {
        let cells: Vec<String> = reports
            .iter()
            .map(|per_method| {
                let rmse: Vec<f64> = per_method[m].iter().map(Report::rmse).collect();
                let worst = rmse.iter().copied().fold(0.0, f64::max);
                format!("{} ({})", metres(median(rmse)), metres(worst))
            })
            .collect();
        println!("| {} | {} |", reports[0][m][0].method, cells.join(" | "));
    }
    println!("\nDetails (medians):\n");
    println!(
        "| Method | Condition | RMSE | 95 % | Max | Last over 1 m | Hand-overs | GNSS / LiDAR / both / dead reckoning |"
    );
    println!("|---|---|---|---|---|---|---|---|");
    for m in 0..method_count {
        for (condition, per_method) in Condition::ALL.iter().zip(&reports) {
            let runs = &per_method[m];
            let total: f64 = runs[0].source_s.iter().sum();
            let share: Vec<String> = (0..4)
                .map(|i| {
                    let s = runs.iter().map(|r| r.source_s[i]).sum::<f64>() / runs.len() as f64;
                    format!("{:.0} %", 100.0 * s / total)
                })
                .collect();
            println!(
                "| {} | {} | {} | {} | {} | {} | {} | {} |",
                runs[0].method,
                condition.name,
                metres(median(runs.iter().map(Report::rmse).collect())),
                metres(median(runs.iter().map(Report::p95).collect())),
                metres(median(runs.iter().map(Report::max).collect())),
                match median(runs.iter().map(|r| r.last_over_s(1.0)).collect()) {
                    0.0 => "never".to_string(),
                    s => format!("{s:.0} s"),
                },
                if runs[0].arbitrates {
                    format!("{:.0}", median(runs.iter().map(|r| r.switches as f64).collect()))
                } else {
                    "–".to_string()
                },
                share.join(" / ")
            );
        }
    }
}

/// Metres with two decimals below 10 m, one below 100 m, none above.
fn metres(v: f64) -> String {
    if v < 10.0 {
        format!("{v:.2} m")
    } else if v < 100.0 {
        format!("{v:.1} m")
    } else {
        format!("{v:.0} m")
    }
}
