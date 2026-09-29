#!/usr/bin/env bash
# Fail early on a server where CUDA is available but Vulkan graphics is not.
set -euo pipefail

if ! command -v nvidia-smi >/dev/null; then
    echo "ERROR: nvidia-smi is unavailable. Choose a CUDA GPU instance." >&2
    exit 20
fi
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader

if ! command -v vulkaninfo >/dev/null; then
    echo "ERROR: vulkaninfo is unavailable. On Ubuntu run:" >&2
    echo "  apt-get update && apt-get install -y libxt6 libglu1-mesa libvulkan1 vulkan-tools" >&2
    exit 21
fi

autodl_icd=/etc/vulkan/icd.d/autodl_nvidia_egl_icd.json
if [[ -f "$autodl_icd" ]]; then
    # AutoDL documents this EGL ICD for headless Vulkan containers.
    export VK_ICD_FILENAMES="$autodl_icd"
fi

vulkan_summary="$(vulkaninfo --summary 2>&1 || true)"
if ! grep -q 'vendorID[[:space:]]*=[[:space:]]*0x10de' <<<"$vulkan_summary"; then
    echo "ERROR: Vulkan cannot see an NVIDIA GPU. Isaac Sim GPU physics will not run." >&2
    echo "$vulkan_summary" | tail -n 40 >&2
    exit 22
fi

echo "Vulkan NVIDIA GPU check passed."
