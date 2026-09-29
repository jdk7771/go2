#!/usr/bin/env bash
# Configure the headless NVIDIA Vulkan ICD documented by AutoDL.
set -euo pipefail

icd_dir=/etc/vulkan/icd.d
icd_file="$icd_dir/autodl_nvidia_egl_icd.json"
library=/lib/x86_64-linux-gnu/libEGL_nvidia.so.0

if [[ ! -e "$library" ]]; then
    echo "ERROR: NVIDIA EGL library not found: $library" >&2
    exit 1
fi

mkdir -p "$icd_dir"
cat >"$icd_file" <<'EOF'
{
    "file_format_version" : "1.0.0",
    "ICD": {
        "library_path": "/lib/x86_64-linux-gnu/libEGL_nvidia.so.0",
        "api_version" : "1.3.277"
    }
}
EOF

echo "Wrote $icd_file"
echo "Run: export VK_ICD_FILENAMES=$icd_file"
