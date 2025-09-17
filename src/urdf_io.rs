
use anyhow::Result;
use urdf_rs::Robot;

pub fn load_urdf(path: &str) -> Result<Robot>{
    Ok(urdf_rs::read_file(path)?)
}

pub fn end_effector_name(cli_arg: Option<String>, urdf: &Robot) -> String {
    cli_arg.unwrap_or_else(|| urdf.links.last().map(|l| l.name.clone()).unwrap_or("tool0".into()))
}
