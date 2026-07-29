#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

CMAKE_BIN="/home/hlavacs/vcpkg/downloads/tools/cmake-4.3.3-linux/cmake-4.3.3-linux-x86_64/bin/cmake"
JOBS="${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc 2>/dev/null || printf '8')}"
VARIANT="Release"
CLEAN=0
WITHOUT_VVE=0

usage() {
  printf 'Usage: %s [debug|release] [--clean] [--without-vve]\n' "$0"
  printf '       --without-vve builds and tests only the header-only VPE library.\n'
}

for arg in "$@"; do
  case "$arg" in
    debug|Debug|DEBUG)
      VARIANT="Debug"
      ;;
    release|Release|RELEASE)
      VARIANT="Release"
      ;;
    --clean)
      CLEAN=1
      ;;
    --without-vve|--standalone)
      WITHOUT_VVE=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 1
      ;;
  esac
done

VARIANT_LOWER="$(printf '%s' "$VARIANT" | tr '[:upper:]' '[:lower:]')"
BUILD_SUFFIX=""
if [ "$WITHOUT_VVE" -eq 1 ]; then
  BUILD_SUFFIX="-standalone"
fi
BUILD_DIR="build/${VARIANT_LOWER}-linux${BUILD_SUFFIX}"

if [ ! -x "$CMAKE_BIN" ]; then
  CMAKE_BIN="$(command -v cmake || true)"
fi
if [ -z "$CMAKE_BIN" ]; then
  printf 'cmake not found. Install it with your Linux package manager.\n' >&2
  exit 1
fi
if ! command -v ninja >/dev/null 2>&1; then
  printf 'ninja not found. Install it with your Linux package manager.\n' >&2
  exit 1
fi
COMPILER_CMAKE_ARGS=()
if [ -x /usr/bin/clang++-18 ] && [ -x /usr/bin/clang-scan-deps-18 ]; then
  COMPILER_CMAKE_ARGS=(
    -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18
    -DCMAKE_CXX_COMPILER_CLANG_SCAN_DEPS=/usr/bin/clang-scan-deps-18
  )
elif [ "$WITHOUT_VVE" -eq 0 ]; then
  printf 'Clang 18 and clang-scan-deps-18 are required by the prebuilt VVE modules.\n' >&2
  exit 1
fi

VPE_CMAKE_ARGS=(-DVPE_BUILD_EXAMPLES=ON -DVPE_BUILD_TESTS=OFF)
if [ "$WITHOUT_VVE" -eq 1 ]; then
  VPE_CMAKE_ARGS=(-DVPE_BUILD_EXAMPLES=OFF -DVPE_BUILD_TESTS=ON)
else
  VVE_ROOT="../ViennaVulkanEngine"
  if [ ! -f "$VVE_ROOT/src/Engine.ixx" ] && [ -f "../../ViennaVulkanEngine/src/Engine.ixx" ]; then
    VVE_ROOT="../../ViennaVulkanEngine"
  fi
  if [ ! -f "$VVE_ROOT/src/Engine.ixx" ]; then
    printf 'ViennaVulkanEngine was not found next to ViennaPhysicsEngine.\n' >&2
    printf 'Use --without-vve to build VPE by itself.\n' >&2
    exit 1
  fi

  VVE_LIBRARY="$VVE_ROOT/bin/release/lib/libViennaVulkanEngine.so"
  VVE_MODULE="$VVE_ROOT/build/release-linux/src/CMakeFiles/ViennaVulkanEngine.dir/VEEngine.pcm"
  VVE_STD_MODULE="$VVE_ROOT/build/release-linux/src/CMakeFiles/__cmake_cxx_std_23.dir/std.pcm"
  VVE_VULKAN="$VVE_ROOT/vcpkg_installed/x64-linux/lib/libvulkan.so"
  for artifact in "$VVE_LIBRARY" "$VVE_MODULE" "$VVE_STD_MODULE" "$VVE_VULKAN"; do
    if [ ! -e "$artifact" ]; then
      printf 'Required VVE artifact is missing: %s\n' "$artifact" >&2
      printf 'Build VVE first with: (cd %s && ./build_linux.sh release)\n' "$VVE_ROOT" >&2
      printf 'Or use --without-vve to build VPE by itself.\n' >&2
      exit 1
    fi
  done
fi

remove_build_dir() {
  case "$BUILD_DIR" in
    build/debug-linux|build/release-linux|build/debug-linux-standalone|build/release-linux-standalone)
      "$CMAKE_BIN" -E remove_directory "$BUILD_DIR"
      ;;
    *)
      printf 'Refusing to remove unexpected build directory: %s\n' "$BUILD_DIR" >&2
      exit 1
      ;;
  esac
}

if [ "$CLEAN" -eq 1 ]; then
  remove_build_dir
elif [ "${#COMPILER_CMAKE_ARGS[@]}" -gt 0 ] &&
     [ -f "$BUILD_DIR/CMakeCache.txt" ] &&
     ! grep -q '^CMAKE_CXX_COMPILER:.*clang++-18$' "$BUILD_DIR/CMakeCache.txt"; then
  printf 'Existing build cache does not use LLVM 18; recreating %s.\n' "$BUILD_DIR"
  remove_build_dir
fi

"$CMAKE_BIN" -S . -B "$BUILD_DIR" -G Ninja \
  -DCMAKE_BUILD_TYPE="$VARIANT" \
  "${COMPILER_CMAKE_ARGS[@]}" \
  "${VPE_CMAKE_ARGS[@]}"

"$CMAKE_BIN" --build "$BUILD_DIR" --target all --parallel "$JOBS"

if [ "$WITHOUT_VVE" -eq 1 ]; then
  CTEST_BIN="$(dirname "$CMAKE_BIN")/ctest"
  if [ ! -x "$CTEST_BIN" ]; then
    CTEST_BIN="$(command -v ctest || true)"
  fi
  if [ -z "$CTEST_BIN" ]; then
    printf 'ctest not found; the standalone build succeeded, but its test could not run.\n' >&2
    exit 1
  fi
  "$CTEST_BIN" --test-dir "$BUILD_DIR" --output-on-failure
fi

printf '\n%s build complete.\n' "$VARIANT"
printf 'Executables: bin/%s/exe\n' "$VARIANT_LOWER"
printf 'Libraries:   bin/%s/lib\n' "$VARIANT_LOWER"
if [ "$WITHOUT_VVE" -eq 1 ]; then
  printf 'VPE target:   ViennaPhysicsEngine::ViennaPhysicsEngine (header-only)\n'
fi
