#!/usr/bin/env bash
set -Eeuo pipefail

MILESTONE_DATE="2026-08-11"
ARCHIVE_NAME="G1_Jetson_State_Milestone_${MILESTONE_DATE}.tar.gz"
ARCHIVE_PATH="$HOME/$ARCHIVE_NAME"
CHECKSUM_PATH="$ARCHIVE_PATH.sha256"

HANDOFF_REL="g1_xsens_direct/g1_xsens_handoff"
GROOT_REL="GR00T-WholeBodyControl"
VENV_REL=".venvs/g1_xsens"

required_paths=(
  "$HOME/$HANDOFF_REL"
  "$HOME/$GROOT_REL"
  "$HOME/$VENV_REL"
)

echo "G1 Xsens/SONIC Jetson milestone collector"
echo "Archive: $ARCHIVE_PATH"
echo

for path in "${required_paths[@]}"; do
  if [[ ! -e "$path" ]]; then
    echo "ERROR: required path is missing: $path" >&2
    exit 1
  fi
done

if [[ -e "$ARCHIVE_PATH" || -e "$CHECKSUM_PATH" ]]; then
  echo "ERROR: milestone output already exists; nothing was overwritten:" >&2
  echo "  $ARCHIVE_PATH" >&2
  echo "  $CHECKSUM_PATH" >&2
  exit 1
fi

echo "Content sizes before compression:"
du -sh "${required_paths[@]}"
if [[ -d /opt/onnxruntime ]]; then
  du -sh /opt/onnxruntime
else
  echo "WARNING: /opt/onnxruntime was not found and cannot be included." >&2
fi
echo
df -h "$HOME"
echo
read -r -p "Type BACKUP to create the archive (or anything else to stop): " answer
if [[ "$answer" != "BACKUP" ]]; then
  echo "Cancelled; no archive was created."
  exit 0
fi

metadata_dir="$(mktemp -d "$HOME/.g1_milestone_metadata.XXXXXX")"
cleanup() {
  rm -rf -- "$metadata_dir"
}
trap cleanup EXIT

run_report() {
  local output="$1"
  shift
  {
    echo "+ $*"
    "$@"
  } >"$metadata_dir/$output" 2>&1 || true
}

{
  echo "Milestone date: $MILESTONE_DATE"
  echo "Created: $(date -Is)"
  echo "Host: $(hostname)"
  echo "User: $(id -un)"
  echo "Included home paths:"
  printf '  %s\n' "$HANDOFF_REL" "$GROOT_REL" "$VENV_REL"
  if [[ -d /opt/onnxruntime ]]; then
    echo "Included system path: opt/onnxruntime"
  fi
} >"$metadata_dir/README.txt"

run_report uname.txt uname -a
run_report hostnamectl.txt hostnamectl
run_report os-release.txt cat /etc/os-release
run_report jetson-release.txt cat /etc/nv_tegra_release
run_report identity.txt id
run_report disk-space.txt df -h
run_report memory.txt free -h
run_report network-addresses.txt ip -br addr
run_report network-routes.txt ip route
run_report listening-ports.txt ss -lntup
run_report running-teleop-processes.txt pgrep -af 'g1_deploy_onnx_ref|live_xsens_sonic|xsens_bridge'
run_report dpkg-packages.txt dpkg-query -W -f='${binary:Package}\t${Version}\n'
run_report python-system-version.txt python3 --version
run_report python-venv-version.txt "$HOME/$VENV_REL/bin/python" --version
run_report python-venv-packages.txt "$HOME/$VENV_REL/bin/python" -m pip freeze
run_report controller-file.txt file "$HOME/$GROOT_REL/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
run_report controller-ldd.txt ldd "$HOME/$GROOT_REL/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"

for repo in "$HOME/$HANDOFF_REL" "$HOME/$GROOT_REL"; do
  repo_name="$(basename "$repo")"
  if git -C "$repo" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    run_report "git-${repo_name}-head.txt" git -C "$repo" rev-parse HEAD
    run_report "git-${repo_name}-status.txt" git -C "$repo" status --short --branch
    run_report "git-${repo_name}-remotes.txt" git -C "$repo" remote -v
    run_report "git-${repo_name}-diff.patch" git -C "$repo" diff --binary --no-ext-diff
    run_report "git-${repo_name}-staged.patch" git -C "$repo" diff --cached --binary --no-ext-diff
  fi
done

hash_targets=(
  "$HOME/$HANDOFF_REL/scripts/run_jetson_current_proven_arms.sh"
  "$HOME/$HANDOFF_REL/scripts/run_jetson_controller.sh"
  "$HOME/$GROOT_REL/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
  "$HOME/$GROOT_REL/gear_sonic_deploy/policy/low_latency/model_decoder.onnx"
  "$HOME/$GROOT_REL/gear_sonic_deploy/policy/low_latency/model_encoder.onnx"
  "$HOME/$GROOT_REL/gear_sonic_deploy/policy/low_latency/observation_config.yaml"
)
{
  for target in "${hash_targets[@]}"; do
    if [[ -f "$target" ]]; then
      sha256sum "$target"
    else
      echo "MISSING  $target"
    fi
  done
} >"$metadata_dir/critical-file-sha256.txt"

archive_args=(
  --numeric-owner
  -czf "$ARCHIVE_PATH"
  -C "$HOME"
  "$HANDOFF_REL"
  "$GROOT_REL"
  "$VENV_REL"
)
if [[ -d /opt/onnxruntime ]]; then
  archive_args+=( -C / opt/onnxruntime )
fi
archive_args+=( -C "$(dirname "$metadata_dir")" "$(basename "$metadata_dir")" )

echo
echo "Creating archive. This may take several minutes..."
tar "${archive_args[@]}"
( cd "$HOME" && sha256sum "$ARCHIVE_NAME" >"$CHECKSUM_PATH" )

echo
echo "Milestone archive created successfully:"
ls -lh "$ARCHIVE_PATH" "$CHECKSUM_PATH"
echo
cat "$CHECKSUM_PATH"
echo
echo "Leave these files in place until the Windows transfer verifies them."
