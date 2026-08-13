#!/usr/bin/env bash

set -euo pipefail

# Files or directories in this list will not be compressed.
# Add or remove items here as needed.
EXCLUDE_LIST=(
  ".history"
  ".catkin_tools"
  ".codex"
  ".agents"
  ".git"
  ".vscode"
  "build"
  "devel"
  "logs"
  "*.zip"
  "*.tar.gz"
  "*.log"
  "*.tmp"
  "*.bak"
  "*.swp"
  "*.swo"
  ".DS_Store"
  "*.egg-info"
  "*.egg"
  "*.pyc"
  "__pycache__"
  "*.pyo"
  "*.pyd"
  "*.bag"
  "*.csv"
)

archive_name="${1:-$(basename "$(pwd)")_$(date +%Y%m%d_%H%M%S).zip}"
if [[ "$archive_name" != *.zip ]]; then
  archive_name="${archive_name}.zip"
fi

archive_path="$archive_name"

if [[ "$archive_path" != /* ]]; then
  archive_path="$(pwd)/$archive_path"
fi

zip_excludes=()
for item in "${EXCLUDE_LIST[@]}"; do
  zip_excludes+=(-x "$item" "./$item" "$item/*" "./$item/*")
done

zip_excludes+=(-x "$(basename "$archive_path")" "./$(basename "$archive_path")")

echo "Compressing current directory: $(pwd)"
echo "Archive: $archive_path"
echo "Excluded:"
printf '  %s\n' "${EXCLUDE_LIST[@]}"

zip -r "$archive_path" . "${zip_excludes[@]}"

echo "Done: $archive_path"
