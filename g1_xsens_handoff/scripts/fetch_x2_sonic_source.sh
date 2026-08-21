#!/usr/bin/env bash
set -euo pipefail

# Experimental X2 embodiment from NVlabs/GR00T-WholeBodyControl PR #112.
# Keep it external: this source is not an official release and contains no X2
# policy checkpoint.
REPO_URL="https://github.com/devsolotech/GR00T-WholeBodyControl.git"
PINNED_COMMIT="489d6dffc6e327b2e28f995d21e26ab300227ada"
DESTINATION="${1:-$(cd "$(dirname "$0")/../.." && pwd)/external/groot_wbc_x2}"

if [[ -e "$DESTINATION" ]]; then
  echo "Destination already exists: $DESTINATION" >&2
  echo "Move it aside or pass a different destination." >&2
  exit 1
fi

git clone "$REPO_URL" "$DESTINATION"
git -C "$DESTINATION" checkout --detach "$PINNED_COMMIT"

actual_commit="$(git -C "$DESTINATION" rev-parse HEAD)"
if [[ "$actual_commit" != "$PINNED_COMMIT" ]]; then
  echo "Unexpected X2 SONIC source commit: $actual_commit" >&2
  exit 1
fi

echo "Experimental X2 SONIC source ready at $DESTINATION"
echo "Pinned PR #112 commit: $actual_commit"
echo "No trained X2 checkpoint is included; obtain it separately."
