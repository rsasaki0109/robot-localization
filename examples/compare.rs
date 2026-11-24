//! Every method on every condition over several seeds: a Markdown table of
//! errors, of how often an arbiter hands over, and of what the estimate
//! rested on.
//!
//! ```bash
//! cargo run --release --example compare            # 3 seeds
//! cargo run --release --example compare -- 10      # 10 seeds
//! ```

use robo_localization::city::City;
use robo_localization::sim::{all_methods, run, start_pose, Condition};

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn main() {
    let seeds: u64 = std::env::args().nth(1).map_or(3, |s| s.parse().expect("seed count"));
    let city = City::district(1);
    let start = start_pose(&city);
    println!(
        "Route {:.0} m, {} buildings, {} seeds per cell (median over seeds; worst seed in brackets).\n",
        city.route_length_m(),
        city.buildings.len(),
        seeds
    );
    println!("| Method | Condition | RMSE | 95 % | Max | Hand-overs | GNSS / LiDAR / both / dead reckoning |");
    println!("|---|---|---|---|---|---|---|");
    let methods = all_methods(start).len();
    for m in 0..methods {
        for condition in Condition::ALL {
            let reports: Vec<_> = (0..seeds)
                .map(|seed| {
                    let mut localizer = all_methods(start).swap_remove(m);
                    run(localizer.as_mut(), &city, condition, seed, false)
                })
                .collect();
            let rmse: Vec<f64> = reports.iter().map(|r| r.rmse()).collect();
            let worst = rmse.iter().copied().fold(0.0, f64::max);
            let total: f64 = reports[0].source_s.iter().sum();
            let share: Vec<String> = (0..4)
                .map(|i| {
                    let s = reports.iter().map(|r| r.source_s[i]).sum::<f64>() / reports.len() as f64;
                    format!("{:.0} %", 100.0 * s / total)
                })
                .collect();
            println!(
                "| {} | {} | {:.2} m ({:.2}) | {:.2} m | {:.2} m | {} | {} |",
                reports[0].method,
                condition.name,
                median(rmse),
                worst,
                median(reports.iter().map(|r| r.p95()).collect()),
                median(reports.iter().map(|r| r.max()).collect()),
                if reports[0].arbitrates {
                    format!("{:.0}", median(reports.iter().map(|r| r.switches as f64).collect()))
                } else {
                    "–".to_string()
                },
                share.join(" / ")
            );
        }
    }
}
