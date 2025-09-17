
mod urdf_io;

use clap::Parser;

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

    Ok(())
}
