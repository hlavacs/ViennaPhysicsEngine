#!/bin/sh
# Build + test on macOS/Linux. (-A x64 is MSVC-only; on Unix, Makefiles/Ninja
# are single-config, so Release and Debug get separate build directories.)
set -e

cmake -S . -B build/release -DCMAKE_BUILD_TYPE=Release
cmake --build build/release -j
if [ "$(uname)" = "Darwin" ]; then
    # macOS randomizes heap addresses per run and offers no ASLR opt-out, so
    # the pointer-keyed contact ordering makes golden runs non-reproducible
    # here (see REFACTORING.md). The Linux CI job is the golden-run authority.
    ctest --test-dir build/release --output-on-failure -E goldenrun
else
    ctest --test-dir build/release --output-on-failure
fi

cmake -S . -B build/debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build/debug -j
# golden files are Release-reference; Debug float results differ -> unit tests only
ctest --test-dir build/debug --output-on-failure -E goldenrun
