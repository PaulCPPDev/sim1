use anyhow::{anyhow, Result};
use k::{self, Chain, SerialChain, node::Node};
use k::InverseKinematicsSolver; // for solve / solve_with_constraints
use k::nalgebra as na;
use std::path::Path;

pub struct Kin {
    pub chain: Chain<f64>,
    pub joint_names: Vec<String>,
    pub q_min: Vec<f64>,
    pub q_max: Vec<f64>,
    pub ee: String,
}

impl Kin {
    /// Build from a URDF *file path* (recommended with k 0.32).
    pub fn from_urdf_file<P: AsRef<Path>>(path: P, ee: &str) -> Result<Self> {
        let chain = Chain::<f64>::from_urdf_file(&path)
            .map_err(|e| anyhow!("k::Chain build failed: {e:?}"))?;

        let joint_names: Vec<String> = chain.iter_joints().map(|j| j.name.clone()).collect();

        // Read limits from the URDF
        let urdf = urdf_rs::read_file(&path)
            .map_err(|e| anyhow!("urdf-rs parse failed: {e:?}"))?;
        let (q_min, q_max) = limits_from_urdf(&joint_names, &urdf);

        Ok(Self { chain, joint_names, q_min, q_max, ee: ee.to_owned() })
    }

    pub fn dof(&self) -> usize { self.joint_names.len() }

    pub fn clamp(&self, q: &mut [f64]) {
        for (i, qi) in q.iter_mut().enumerate() {
            *qi = qi.clamp(self.q_min[i], self.q_max[i]);
        }
    }

    pub fn fk(&self, q: &[f64]) -> na::Isometry3<f64> {
        self.chain.set_joint_positions(q);
        self.chain.update_transforms();
        self.chain
            .find_link(&self.ee)
            .expect("end-effector link not found")
            .world_transform()
            .expect("world_transform None; call update_transforms() first")
    }

    pub fn link_poses(&self) -> Vec<(String, na::Isometry3<f64>)> {
        self.chain.iter().map(|n| {
            let name = name_of(n);
            let iso = n.world_transform().expect("call update_transforms() first");
            (name, iso)
        }).collect()
    }

    pub fn ik(&mut self, target: &na::Isometry3<f64>, max_iter: usize, eps: f64) -> Vec<f64> {
        // Build a serial chain from the end-effector **link** node
        let ee_node = self.chain
            .find_link(&self.ee)
            .expect("ee link not found");
        let arm = SerialChain::from_end(ee_node);
    
        // Solver (k 0.32): use `new(distance, angle, jacobian_multiplier, num_max_try: usize)`
        let solver = k::JacobianIkSolver::<f64>::new(eps, 1e-3, 1.0, max_iter);
    
        // Constraints (default = no special constraints)
        let constraints = k::Constraints::default();
    
        // Solve on the serial chain
        let _ = solver.solve_with_constraints(&arm, &target, &constraints);
    
        // Optionally clamp and write back
        let mut q = self.chain.joint_positions();
        self.clamp(&mut q);
        self.chain.set_joint_positions(&q);
        q
    }
    
    
}    

/// In your urdf-rs version `Joint.limit` exists and `lower/upper` are `f64` (not Option).
fn limits_from_urdf(joint_names: &[String], urdf: &urdf_rs::Robot) -> (Vec<f64>, Vec<f64>) {
    use std::f64::{INFINITY as INF, consts::PI};

    let mut lo = Vec::with_capacity(joint_names.len());
    let mut hi = Vec::with_capacity(joint_names.len());

    for name in joint_names {
        let j = urdf.joints.iter()
            .find(|j| j.name == *name)
            .unwrap_or_else(|| panic!("joint `{name}` not found in URDF"));

        // In some URDFs for continuous joints, authors set [-PI, PI] or similar.
        // If you actually have unbounded joints, set them to (-INF, INF) here.
        let l = &j.limit;
        let mut lower = l.lower;
        let mut upper = l.upper;

        // Heuristic: if limits look degenerate (lower >= upper), treat as unbounded
        if !(lower < upper) {
            lower = -INF;
            upper = INF;
        } else {
            // If your model uses 0/0 for continuous, you can special-case here.
            // Keep as-is by default.
        }

        // Clamp to something sane if still weird
        if !lower.is_finite() && !upper.is_finite() && j.joint_type == urdf_rs::JointType::Revolute {
            lower = -PI;
            upper = PI;
        }

        lo.push(lower);
        hi.push(upper);
    }
    (lo, hi)
}

/// Extract a readable name: prefer link name; else joint name.
fn name_of(n: &Node<f64>) -> String {
    // n.link() returns an Option-like guard. Deref to get Option.
    if let Some(l) = (*n.link()).as_ref() {
        return l.name.clone();
    }
    // n.joint() returns a guard (not Option) in k 0.32
    let j = n.joint();
    j.name.clone()
}
