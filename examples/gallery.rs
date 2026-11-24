//! Animated GIFs of every method driving the route side by side, one GIF
//! per condition, and a static overview of the city and its GNSS sky.
//!
//! ```bash
//! cargo run --release --features render --example gallery            # all -> out/
//! cargo run --release --features render --example gallery crowd      # one condition
//! ```
//!
//! Each panel follows the robot (north up, 60 m across): buildings shaded
//! by height (the LiDAR map's footprints in purple where the map differs
//! from the world), people, the LiDAR scan (cyan), the latest GNSS fix (a cross:
//! green RTK fixed, yellow float, red single), the true track (black) and
//! the estimate's, coloured by what it rested on. The strip under each panel
//! is the whole run's source timeline so far: every colour change is a
//! change of source.

use robo_localization::city::{Building, City};
use robo_localization::geometry::{Pose2, Vec2};
use robo_localization::gnss::{FixKind, Receiver};
use robo_localization::methods::Source;
use robo_localization::sim::{all_methods, drive, run, start_pose, Condition, MapState, Report};
use robo_planning::geometry as pg;
use robo_planning::render::{colors, Canvas, GifWriter, Rgba, View};

const WIDTH: u32 = 960;
const PANEL_W: f32 = 320.0;
const PANEL_H: f32 = 282.0;
const TITLE_H: f32 = 36.0;
const HEIGHT: u32 = (TITLE_H + 2.0 * PANEL_H) as u32;
/// Panel window, metres across.
const WINDOW_M: f64 = 60.0;
/// Every this many recorded frames (0.5 s each) becomes a GIF frame.
const STRIDE: usize = 8;

fn v(p: Vec2) -> pg::Vec2 {
    pg::Vec2::new(p.x, p.y)
}

fn pose(p: Pose2) -> pg::Pose2 {
    pg::Pose2::new(p.x, p.y, p.theta_rad)
}

fn source_color(s: Source) -> Rgba {
    match s {
        Source::Gnss => Rgba::rgb(31, 119, 180),
        Source::Lidar => Rgba::rgb(200, 60, 170),
        Source::Both => Rgba::rgb(44, 160, 44),
        Source::DeadReckoning => Rgba::rgb(255, 127, 14),
    }
}

fn fix_color(k: FixKind) -> Rgba {
    match k {
        FixKind::Fixed => Rgba::rgb(30, 170, 60),
        FixKind::Float => Rgba::rgb(230, 180, 0),
        FixKind::Single => Rgba::rgb(220, 40, 40),
    }
}

fn building_color(b: &Building) -> Rgba {
    // Pure greys survive the GIF palette; tinted ones turn purple.
    let g = (205.0 - b.height_m.min(200.0) * 0.6) as u8;
    Rgba::rgb(g, g, g)
}

fn footprint(b: &Building) -> [pg::Vec2; 4] {
    b.edges().map(|(a, _)| v(a))
}

/// What a panel draws besides the method's run.
struct Scene<'a> {
    city: &'a City,
    /// The out-of-date map, when the condition has one.
    map: Option<&'a [Building]>,
    /// Recorded frames in the run.
    total: usize,
}

/// One panel: a method at recorded frame `k`, on its own canvas so nothing
/// spills into the neighbours.
fn panel(scene: &Scene, report: &Report, k: usize) -> Canvas {
    let mut canvas = Canvas::new(PANEL_W as u32, PANEL_H as u32);
    let c = &mut canvas;
    let (x, y) = (0.0, 0.0);
    let f = &report.frames[k];
    let half = Vec2::new(WINDOW_M * 0.5, WINDOW_M * 0.5 * 0.78);
    let center = f.truth.pos();
    let view_y = y + 44.0;
    let view_h = PANEL_H - 44.0 - 22.0;
    let view = View::fit(
        v(center - half),
        v(center + half),
        x + 6.0,
        view_y,
        PANEL_W - 12.0,
        view_h,
    );
    c.rect_px(x + 6.0, view_y, PANEL_W - 12.0, view_h, Rgba::rgb(236, 236, 232));
    for b in &scene.city.buildings {
        c.polygon(&view, &footprint(b), building_color(b));
    }
    if let Some(map) = scene.map {
        // The out-of-date map's footprints.
        for b in map {
            let mut ring: Vec<pg::Vec2> = footprint(b).to_vec();
            ring.push(ring[0]);
            c.polyline(&view, &ring, Rgba::rgb(170, 60, 200).alpha(200), 1.2);
        }
    }
    for &p in &f.people {
        c.disc(&view, v(p), 0.35, Rgba::rgb(150, 100, 70));
    }
    for &p in &f.scan {
        c.dot(&view, v(p), 1.3, Rgba::rgb(0, 170, 200));
    }
    // The last minute of the true track and the estimate's.
    let from = k.saturating_sub(120);
    let truth: Vec<pg::Vec2> = report.frames[from..=k].iter().map(|f| v(f.truth.pos())).collect();
    c.polyline(&view, &truth, colors::INK.alpha(160), 1.5);
    for w in report.frames[from..=k].windows(2) {
        c.polyline(
            &view,
            &[v(w[0].estimate.pos()), v(w[1].estimate.pos())],
            source_color(w[1].source),
            3.0,
        );
    }
    if let Some(fix) = f.epoch.as_ref().and_then(|e| e.fix) {
        let (px, py) = view.px(v(fix.pos));
        if px > x + 8.0 && px < x + PANEL_W - 8.0 && py > view_y + 2.0 && py < view_y + view_h - 2.0 {
            let col = fix_color(fix.kind);
            c.polyline_px(&[(px - 5.0, py - 5.0), (px + 5.0, py + 5.0)], col, 2.5);
            c.polyline_px(&[(px - 5.0, py + 5.0), (px + 5.0, py - 5.0)], col, 2.5);
        }
    }
    c.disc_robot(&view, pose(f.truth), 0.7, colors::INK);
    c.arrow(&view, pose(f.estimate), 2.5, source_color(f.source), 2.5);
    // Header.
    c.rect_px(x + 6.0, y + 4.0, PANEL_W - 12.0, 38.0, colors::WHITE);
    c.text(x + 10.0, y + 8.0, report.method, 2.0, colors::INK);
    let err = f.estimate.pos().dist(f.truth.pos());
    let line = format!("{}  err {:.2} m", f.source.name(), err);
    c.text(x + 10.0, y + 28.0, &line, 1.0, source_color(f.source));
    let changes = report.frames[..=k]
        .windows(2)
        .filter(|w| w[0].source != w[1].source)
        .count();
    let tag = format!("{changes} changes");
    c.text(
        x + PANEL_W - 12.0 - Canvas::text_width(&tag, 1.0) - 4.0,
        y + 28.0,
        &tag,
        1.0,
        colors::INK,
    );
    // Source timeline of the whole run so far.
    let (sx, sy, sw) = (x + 8.0, y + PANEL_H - 16.0, PANEL_W - 16.0);
    c.rect_px(sx, sy, sw, 9.0, Rgba::rgb(225, 225, 225));
    for i in 0..sw as usize {
        let j = i * scene.total / sw as usize;
        if j > k {
            break;
        }
        c.rect_px(sx + i as f32, sy, 1.0, 9.0, source_color(report.frames[j].source));
    }
    canvas
}

fn condition_gif(condition: Condition, out: &std::path::Path) -> std::io::Result<()> {
    let city = &condition.city();
    let reports: Vec<Report> = all_methods(condition.start(city, 0))
        .into_iter()
        .map(|mut m| run(m.as_mut(), city, condition, 0, true))
        .collect();
    // A map that differs from the world is drawn as purple outlines.
    let map = (condition.map != MapState::Exact).then(|| condition.map(city));
    let scene = Scene {
        city,
        map: map.as_deref(),
        total: reports[0].frames.len(),
    };
    let mut gif = GifWriter::create(out, WIDTH, HEIGHT)?;
    for k in (0..scene.total).step_by(STRIDE) {
        let mut c = Canvas::new(WIDTH, HEIGHT);
        let t = reports[0].frames[k].t;
        let title = format!("{}  t = {:3.0} s", condition.name, t);
        c.text(12.0, 10.0, &title, 2.0, colors::INK);
        let mut lx = WIDTH as f32 - 12.0;
        for s in [Source::DeadReckoning, Source::Both, Source::Lidar, Source::Gnss] {
            let w = Canvas::text_width(s.name(), 1.0);
            lx -= w + 22.0;
            c.rect_px(lx, 13.0, 10.0, 10.0, source_color(s));
            c.text(lx + 14.0, 14.0, s.name(), 1.0, colors::INK);
        }
        for (i, r) in reports.iter().enumerate() {
            let (x, y) = ((i % 3) as f32 * PANEL_W, TITLE_H + (i / 3) as f32 * PANEL_H);
            c.blit(&panel(&scene, r, k), x as i32, y as i32);
        }
        gif.frame(&c, 8)?;
    }
    for r in &reports {
        println!(
            "  {:13} RMSE {:.2} m, max {:.2} m, {} source changes",
            r.method,
            r.rmse(),
            r.max(),
            r.switches
        );
    }
    Ok(())
}

/// The city from above: buildings by height, the route coloured by the
/// solution the receiver gets there.
fn overview(city: &City, out: &std::path::Path) -> std::io::Result<()> {
    let (w, h) = (960, 720);
    let mut c = Canvas::new(w, h);
    let view = View::fit(v(city.min), v(city.max), 20.0, 60.0, w as f32 - 40.0, h as f32 - 80.0);
    let (a, b) = (view.px(v(city.min)), view.px(v(city.max)));
    c.rect_px(a.0, b.1, b.0 - a.0, a.1 - b.1, Rgba::rgb(236, 236, 232));
    for b in &city.buildings {
        c.polygon(&view, &footprint(b), building_color(b));
        let center = (b.min + b.max) * 0.5;
        let (px, py) = view.px(v(center));
        let label = format!("{:.0} m", b.height_m);
        let ink = if b.height_m < 60.0 { colors::INK } else { colors::WHITE };
        c.text(px - Canvas::text_width(&label, 1.0) * 0.5, py - 3.0, &label, 1.0, ink);
    }
    let mut receiver = Receiver::new(30, 7);
    let path = drive(city);
    let samples: Vec<Pose2> = path.iter().step_by(20).map(|p| p.0).collect();
    for pair in samples.windows(2) {
        let (p, q) = (pair[0].pos(), pair[1].pos());
        let col = match receiver.epoch(city, p).fix.map(|f| f.kind) {
            Some(k) => fix_color(k),
            None => colors::INK,
        };
        c.polyline(&view, &[v(p), v(q)], col, 6.0);
    }
    c.disc_robot(&view, pose(start_pose(city)), 4.0, colors::INK);
    c.text(
        20.0,
        14.0,
        "The district: building heights and the receiver along the route",
        2.0,
        colors::INK,
    );
    let mut lx = 20.0;
    for (name, col) in [
        ("RTK fixed", fix_color(FixKind::Fixed)),
        ("RTK float", fix_color(FixKind::Float)),
        ("single point", fix_color(FixKind::Single)),
        ("no solution", colors::INK),
    ] {
        c.rect_px(lx, 38.0, 14.0, 10.0, col);
        c.text(lx + 18.0, 39.0, name, 1.0, colors::INK);
        lx += Canvas::text_width(name, 1.0) + 40.0;
    }
    let mut gif = GifWriter::create(out, w, h)?;
    gif.frame(&c, 100)
}

fn main() -> std::io::Result<()> {
    let only = std::env::args().nth(1);
    let city = City::district(1);
    let dir = std::path::Path::new("out");
    if only.is_none() || only.as_deref() == Some("overview") {
        overview(&city, &dir.join("overview.gif"))?;
        println!("wrote out/overview.gif");
    }
    for condition in Condition::ALL {
        let slug = condition.name.replace(' ', "_");
        if only.as_deref().is_some_and(|o| o != slug) {
            continue;
        }
        let out = dir.join(format!("methods_{slug}.gif"));
        println!("{}:", condition.name);
        condition_gif(condition, &out)?;
        println!("wrote {}", out.display());
    }
    Ok(())
}
