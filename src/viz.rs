use bevy::prelude::*; // Bevy 0.16
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use k::nalgebra as na;

pub struct VizConfig {
    pub dof: usize,
    pub q_min: Vec<f64>,
    pub q_max: Vec<f64>,
}

#[derive(Resource)]
pub struct AppState {
    pub q: Vec<f64>,
    pub target: na::Isometry3<f64>,
}

pub fn run_viz<Fk, Ik>(
    title: &str,
    cfg: VizConfig,
    fk: Fk,
    ik: Ik,
) where
    Fk: 'static + Send + Sync + FnMut(&[f64]) -> Vec<(String, na::Isometry3<f64>)>,
    Ik: 'static + Send + Sync + FnMut(&na::Isometry3<f64>) -> Vec<f64>,
{
    let mut app = App::new();
    app.insert_resource(ClearColor(Color::srgb(0.02, 0.02, 0.02)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: title.to_string(),
                ..Default::default()
            }),
            ..Default::default()
        }))
        .add_plugins(EguiPlugin::default()) // multipass is enabled for the primary context by default
        .insert_resource(AppState {
            q: (0..cfg.dof).map(|_| 0.0).collect(),
            target: na::Isometry3::identity(),
        })
        .add_systems(Startup, setup)
        // >>> Run the UI inside the egui pass so a frame is guaranteed to be active.
        .add_systems(EguiPrimaryContextPass, ui_panel(cfg, ik))
        // Draw gizmos in the normal Update schedule.
        .add_systems(Update, draw_frames(fk));
    app.run();
}

fn setup(mut commands: Commands) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(2.5, 2.0, 3.5).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Directional light
    commands.spawn((
        DirectionalLight::default(),
        Transform::default(),
    ));
}

fn ui_panel<Ik>(cfg: VizConfig, mut ik: Ik) -> impl FnMut(ResMut<AppState>, EguiContexts)
where
    Ik: FnMut(&na::Isometry3<f64>) -> Vec<f64> + Send + Sync + 'static,
{
    move |mut state: ResMut<AppState>, mut egui_ctx: EguiContexts| {
        // In this pass the context must exist; unwrap with a clear message if not.
        let ctx = egui_ctx
            .ctx_mut()
            .expect("primary Egui context missing inside EguiPrimaryContextPass");

        egui::Window::new("Controls").show(ctx, |ui| {
            ui.heading("Target pose (m / rad)");

            // Copy out to avoid overlapping borrows.
            let mut tx = state.target.translation.x;
            let mut ty = state.target.translation.y;
            let mut tz = state.target.translation.z;
            let (mut rx, mut ry, mut rz) = state.target.rotation.euler_angles();

            ui.add(egui::DragValue::new(&mut tx).speed(0.001).prefix("x: "));
            ui.add(egui::DragValue::new(&mut ty).speed(0.001).prefix("y: "));
            ui.add(egui::DragValue::new(&mut tz).speed(0.001).prefix("z: "));
            ui.add(egui::DragValue::new(&mut rx).speed(0.001).prefix("roll: "));
            ui.add(egui::DragValue::new(&mut ry).speed(0.001).prefix("pitch: "));
            ui.add(egui::DragValue::new(&mut rz).speed(0.001).prefix("yaw: "));

            state.target = na::Isometry3::from_parts(
                na::Translation3::new(tx, ty, tz),
                na::UnitQuaternion::from_euler_angles(rx, ry, rz),
            );

            ui.separator();
            ui.heading("Joints");
            for i in 0..cfg.dof {
                let mut val = state.q[i];
                let range = cfg.q_min[i]..=cfg.q_max[i];
                ui.add(egui::Slider::new(&mut val, range).text(format!("q[{i}]")));
                state.q[i] = val;
            }

            if ui.button("Solve IK to Target").clicked() {
                state.q = ik(&state.target);
            }
        });
    }
}

fn draw_frames<Fk>(mut fk: Fk) -> impl FnMut(Res<AppState>, Gizmos)
where
    Fk: FnMut(&[f64]) -> Vec<(String, na::Isometry3<f64>)> + Send + Sync + 'static,
{
    move |state: Res<AppState>, mut gizmos: Gizmos| {
        // Robot link frames
        for (_name, pose) in fk(&state.q) {
            draw_axis(&mut gizmos, pose, 0.1);
        }
        // World axis
        draw_axis(&mut gizmos, na::Isometry3::identity(), 0.2);
        // Target axis
        draw_axis(&mut gizmos, state.target, 0.15);
    }
}

fn draw_axis(g: &mut Gizmos, iso: na::Isometry3<f64>, len: f32) {
    let o = to_v3(iso.translation.vector);
    let x = to_v3(iso.translation.vector + (iso.rotation * na::Vector3::x()) * len as f64);
    let y = to_v3(iso.translation.vector + (iso.rotation * na::Vector3::y()) * len as f64);
    let z = to_v3(iso.translation.vector + (iso.rotation * na::Vector3::z()) * len as f64);

    g.line(o, x, Color::srgb(1.0, 0.0, 0.0)); // X (red)
    g.line(o, y, Color::srgb(0.0, 1.0, 0.0)); // Y (green)
    g.line(o, z, Color::srgb(0.0, 0.0, 1.0)); // Z (blue)
}

fn to_v3(v: na::Vector3<f64>) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}
