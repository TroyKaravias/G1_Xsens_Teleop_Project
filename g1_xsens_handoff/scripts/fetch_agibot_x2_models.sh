#!/usr/bin/env bash
set -euo pipefail

REPO_URL="https://github.com/AgibotTech/agibot_x2_urdf.git"
PINNED_COMMIT="77f43eb0904dae4c48ccd9154fee824f8ffd4d38"
DESTINATION="${1:-$(cd "$(dirname "$0")/../.." && pwd)/external/agibot_x2_urdf}"

if [[ -e "$DESTINATION" ]]; then
  echo "Destination already exists: $DESTINATION" >&2
  echo "Move it aside or pass a different destination." >&2
  exit 1
fi

git clone "$REPO_URL" "$DESTINATION"
git -C "$DESTINATION" checkout --detach "$PINNED_COMMIT"

actual_commit="$(git -C "$DESTINATION" rev-parse HEAD)"
if [[ "$actual_commit" != "$PINNED_COMMIT" ]]; then
  echo "Unexpected AgiBot model commit: $actual_commit" >&2
  exit 1
fi

echo "AgiBot X2 models ready at $DESTINATION"
echo "Pinned commit: $actual_commit"

