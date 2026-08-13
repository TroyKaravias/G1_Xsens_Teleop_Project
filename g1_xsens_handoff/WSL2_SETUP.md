# Windows + WSL2 Setup

## 1. Install WSL2

In Administrator PowerShell:

```powershell
wsl --status
wsl --list --verbose
```

If Ubuntu is absent:

```powershell
wsl --install -d Ubuntu-24.04
```

Restart Windows if requested. This does not remove or replace native dual-boot Ubuntu.

## 2. Verify WSL

In the Ubuntu/WSL terminal:

```bash
uname -a
nvidia-smi
```

Confirm the RTX 5070 appears.

## 3. Put this package in WSL

If the ZIP is in Windows Downloads:

```bash
mkdir -p ~/g1_xsens_handoff
cd ~/g1_xsens_handoff
unzip /mnt/c/Users/REPLACE_WITH_WINDOWS_USER/Downloads/g1_xsens_handoff.zip
cd g1_xsens_handoff
```

Do not run performance-sensitive simulation from `/mnt/c`; copy it into the WSL home
directory as shown above.

## 4. Install basic packages

```bash
sudo apt update
sudo apt install -y git git-lfs unzip build-essential
git lfs install
```

Then follow `EXTERNAL_FILES.md` and the official ProtoMotions installation instructions.

## 5. Xsens destination

For live MVN streaming, first try:

```text
Destination: 127.0.0.1
Port:        9763
Protocol:    UDP / MXTP02
```

Run the WSL receiver with `--bind 0.0.0.0`. If localhost forwarding is unavailable in
the installed WSL networking mode, use the WSL IP shown by `hostname -I` as the MVN
destination. Never use the robot IP during simulation testing.

