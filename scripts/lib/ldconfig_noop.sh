#!/usr/bin/env bash
set -euo pipefail

# librealsense v2.56.5 unconditionally runs a bare `ldconfig` during install,
# even for a non-system CMAKE_INSTALL_PREFIX. build_ubuntu.sh installs this
# script as a private executable and prepends only that directory to PATH for
# the repository-local `cmake --install` call.
echo "Skipping ldconfig for repository-local librealsense installation."
