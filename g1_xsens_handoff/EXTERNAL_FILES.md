# External Files and Dependencies

The official ProtoMotions repository and pretrained model are intentionally not copied
into this handoff package.

Clone:

```bash
git clone https://github.com/NVlabs/ProtoMotions.git ~/protomotions
cd ~/protomotions
git lfs install
git lfs pull
```

Required files after Git LFS completes:

```text
data/pretrained_models/motion_tracker/g1-bones-deploy/compiled_models/unified_pipeline.onnx
data/pretrained_models/motion_tracker/g1-bones-deploy/compiled_models/unified_pipeline.yaml
data/pretrained_models/motion_tracker/g1-bones-deploy/last.ckpt
data/motion_for_trackers/g1_bones_seed_mini.pt
protomotions/data/assets/mjcf/g1_holo_compat.xml
deployment/test_tracker_mujoco.py
```

Previously verified environment:

```text
Python       3.10
torch        2.12.1+cu130
CUDA build   13.0
mujoco       3.10.0
onnxruntime  1.23.2
```

The workstation driver reported NVIDIA `595.71.05`. Do not install a Linux NVIDIA
display driver inside WSL2; WSL uses the Windows host driver.

The original `g1-moves`/RoboJuDo directory is also excluded because it is approximately
2.9 GB and is not required for the current ProtoMotions live-tracker milestone.

## G1 Jetson SONIC runtime

Physical operation additionally requires an operator-supplied, robot-compatible
SONIC deployment at:

```text
/home/unitree/GR00T-WholeBodyControl/gear_sonic_deploy
```

The repository does not redistribute the compiled controller, ONNX policy
models, Unitree SDK libraries, or vendor configuration. The global-position
launcher currently expects:

```text
target/release/g1_deploy_onnx_ref
policy/release/model_decoder.onnx
policy/release/model_encoder.onnx
policy/release/observation_config.yaml
planner/target_vel/V2/planner_sonic.onnx
reference/example/
thirdparty/unitree_sdk2/thirdparty/lib/aarch64/
```

These files must be obtained through their authorized vendor/project source.
Runtime and model sets are robot-specific: do not copy a working robot's
vendor tree onto another G1 without confirming compatibility and licensing.
