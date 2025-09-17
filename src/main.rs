
mod urdf_io;

use clap::Parser;

#[derive(Parser, Debug)]
struct Args{
    #[arg(long)] urdf: String, //(mandatory) --urdf path_to_urdf
    #[arg(long)] ee: Option<String>, //(optional) --ee endeffector_name
}

fn main() {
    let args = Args::parse();
    println!("path to urdf = {0}", args.urdf);
}
