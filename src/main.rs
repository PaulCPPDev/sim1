
mod urdf_io;
mod kin;
mod viz;

use anyhow::Result;
use clap::Parser;
use k::nalgebra as na;
use std::sync::{Arc, Mutex};

use bevy::prelude::*;

// CLI args parsing
#[derive(Parser, Debug)]
struct Args{
    #[arg(long)] urdf: String, //(mandatory) --urdf path_to_urdf
    #[arg(long)] ee: Option<String>, //(optional) --ee endeffector_name
}

fn main() -> anyhow::Result<()>{
    // test cmd: cargo run -- --urdf="urdf/rr_arm.urdf"

    // CLI Args parsing
    let args = Args::parse();
    println!("path to urdf = {0}", args.urdf);

    // URDF test
    let urdf = urdf_io::load_urdf(&args.urdf)?;
    let ee = urdf_io::end_effector_name(args.ee, &urdf);
    println!("robot name = {0}", urdf.name);
    println!("end effector name (optional) = {0}", ee);

    // Kin
    let kin = Arc::new(Mutex::new(kin::Kin::from_urdf_file(&args.urdf, &ee)?));

    // Prepare viz config
    let (dof, q_min, q_max) = {
        let k = kin.lock().unwrap();     // lock once
        (k.dof(), k.q_min.clone(), k.q_max.clone())
    };
    let cfg = viz::VizConfig { dof, q_min, q_max };

    // FK closure
    let kin_fk = kin.clone();
    let fk = move |q: &[f64]| {
        let mut k = kin_fk.lock().unwrap();
        k.chain.set_joint_positions(q);
        k.chain.update_transforms();
        k.link_poses()
    };

    // IK closure
    let kin_ik = kin.clone();
    let ik = move |target: &na::Isometry3<f64>| -> Vec<f64> {
        let mut k = kin_ik.lock().unwrap();
        k.ik(target, 200, 1e-3)
    };

    viz::run_viz("Sim FK/IK (MVP)", cfg, fk, ik);

    Ok(())
}
