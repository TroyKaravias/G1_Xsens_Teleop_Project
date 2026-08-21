#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 POLICY.onnx [MOTION.pkl]" >&2
  exit 2
fi

policy="$1"
motion="${2:-}"

if [[ ! -s "$policy" ]]; then
  echo "Missing or empty X2 SONIC policy: $policy" >&2
  exit 1
fi
if [[ "$policy" != *.onnx ]]; then
  echo "Expected the fused X2 SONIC .onnx policy: $policy" >&2
  exit 1
fi
if [[ -n "$motion" ]]; then
  if [[ ! -s "$motion" ]]; then
    echo "Missing or empty X2 SONIC motion PKL: $motion" >&2
    exit 1
  fi
  if [[ "$motion" != *.pkl ]]; then
    echo "Expected an X2-retargeted .pkl motion reference: $motion" >&2
    exit 1
  fi
fi

echo "X2 SONIC input files are present."
echo "This check verifies file presence only; it does not validate policy provenance, tensor layout, simulation behavior, or hardware safety."
