#!/usr/bin/env bash
# Clone, build, and install a CMake-based dependency into /usr/local.
#
# Usage: build-cmake-dep <git-url> [<sha>] [<extra-cmake-args>...]
#
#   <git-url>    Git URL to clone.
#   <sha>        Optional commit SHA to pin. Empty/omitted = clone HEAD shallow.
#   <extra...>   Extra arguments forwarded to the configure step (e.g. -DFOO=ON).
#
# Sources ROS if /opt/ros/$ROS_DISTRO/setup.sh exists. Cleans up on exit so
# the Docker layer stays small.
set -eo pipefail

if [ $# -lt 1 ]; then
    echo "usage: $0 <git-url> [<sha>] [<extra-cmake-args>...]" >&2
    exit 1
fi

url="$1"
shift
sha=""
if [ $# -gt 0 ] && [[ "$1" != -D* ]]; then
    sha="$1"
    shift
fi

name="$(basename "$url" .git)"
src="/src/${name}"

# ROS setup scripts reference unset vars; don't fail on them here.
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.sh" ]; then
    # shellcheck disable=SC1091
    . "/opt/ros/${ROS_DISTRO}/setup.sh"
fi

mkdir -p /src
if [ -n "$sha" ]; then
    git clone --recurse-submodules "$url" "$src"
    git -C "$src" checkout "$sha"
    git -C "$src" submodule update --init --recursive
else
    git clone --depth=1 --recurse-submodules "$url" "$src"
fi

cmake -S "$src" -B "$src/build" -G Ninja \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    -DBUILD_TESTING=OFF \
    -DINSTALL_DOCUMENTATION=OFF \
    "$@"
cmake --build "$src/build"
cmake --install "$src/build"
ldconfig
rm -rf "$src"
