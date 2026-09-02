@echo off
setlocal EnableExtensions EnableDelayedExpansion

rem Builds the Vienna Physics Engine on Windows together with the Vienna Vulkan
rem Engine V3, which is added as a CMake subdirectory from ..\ViennaVulkanEngine.
rem Uses the Ninja generator: VVE needs C++23 `import std`, which the Visual
rem Studio generator does not support. Mirrors ViennaVulkanEngine\build_windows.cmd.
rem
rem Binaries land in ..\ViennaVulkanEngine\bin\<variant>\exe\physicsexample.exe,
rem next to the VVE runtime DLLs and compiled shaders.
rem
rem Usage: build_cmake.cmd [debug|release] [--clean] [--tests] [--without-vve]
rem Requires: Vulkan SDK (VULKAN_SDK set), CMake >= 3.31.8, Visual Studio 2022+
rem           with the C++ workload (provides cl and Ninja), and a VVE checkout
rem           whose vcpkg manifest has been installed (run VVE's
rem           build_windows.cmd once, or `vcpkg install` inside it).

pushd "%~dp0"

set "VARIANT=release"
set "CLEAN=0"
set "RUN_TESTS=0"
set "WITH_VVE=1"

:parse
if "%~1"=="" goto done_parse
if /I "%~1"=="debug" (set "VARIANT=debug") else if /I "%~1"=="release" (set "VARIANT=release") else if /I "%~1"=="--clean" (set "CLEAN=1") else if /I "%~1"=="--tests" (set "RUN_TESTS=1") else if /I "%~1"=="--without-vve" (set "WITH_VVE=0") else if /I "%~1"=="--standalone" (set "WITH_VVE=0") else if /I "%~1"=="-h" (goto usage) else if /I "%~1"=="--help" (goto usage) else (echo Unknown argument: %~1 & goto usage)
shift
goto parse
:done_parse

if /I "%VARIANT%"=="debug" (set "CONFIG=Debug") else (set "CONFIG=Release")
set "BUILD_DIR=build\%VARIANT%-windows"
set "VVE_ROOT=%~dp0..\ViennaVulkanEngine"

if "%WITH_VVE%"=="1" (
    if not defined VULKAN_SDK (echo VULKAN_SDK is not set. Install the Vulkan SDK first. & goto fail)
    if not exist "%VVE_ROOT%\CMakeLists.txt" (
        echo ViennaVulkanEngine was not found at "%VVE_ROOT%".
        echo Clone it next to ViennaPhysicsEngine, or pass --without-vve to build only the VPE module.
        goto fail
    )
    if not exist "%VVE_ROOT%\vcpkg_installed\x64-windows" (
        echo "%VVE_ROOT%\vcpkg_installed\x64-windows" is missing.
        echo Run ViennaVulkanEngine\build_windows.cmd once ^(or `vcpkg install` inside ViennaVulkanEngine^) to install VVE's dependencies.
        goto fail
    )
)

rem --- Ensure the MSVC toolchain (cl + Ninja) is on PATH ---
where cl >nul 2>nul
if errorlevel 1 (
    set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
    if not exist "!VSWHERE!" (echo vswhere.exe not found; run this from a "x64 Native Tools Command Prompt for VS". & goto fail)
    for /f "usebackq tokens=*" %%i in (`"!VSWHERE!" -latest -prerelease -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINSTALL=%%i"
    if not defined VSINSTALL (echo No Visual Studio with the C++ toolchain was found. & goto fail)
    echo Initializing MSVC environment from "!VSINSTALL!" ...
    call "!VSINSTALL!\VC\Auxiliary\Build\vcvars64.bat" >nul
    if errorlevel 1 goto fail
)

where cmake >nul 2>nul
if errorlevel 1 (echo cmake not found on PATH. & goto fail)
where ninja >nul 2>nul
if errorlevel 1 (echo ninja not found. Install the "C++ CMake tools" component in the Visual Studio Installer. & goto fail)

if "%CLEAN%"=="1" (
    if exist "%BUILD_DIR%" (echo Removing %BUILD_DIR% ... & rmdir /s /q "%BUILD_DIR%")
)

rem --- Configure with Ninja (single-config); import-std flags are set by CMakeLists.txt ---
if "%WITH_VVE%"=="1" (
    set "VPE_CMAKE_ARGS=-DVPE_BUILD_EXAMPLES=ON -DVPE_VVE_IN_TREE=ON"
) else (
    set "VPE_CMAKE_ARGS=-DVPE_BUILD_EXAMPLES=OFF"
)
if "%RUN_TESTS%"=="1" (
    set "VPE_CMAKE_ARGS=!VPE_CMAKE_ARGS! -DVPE_BUILD_TESTS=ON"
) else (
    set "VPE_CMAKE_ARGS=!VPE_CMAKE_ARGS! -DVPE_BUILD_TESTS=OFF"
)

cmake -S . -B "%BUILD_DIR%" -G Ninja ^
    -DCMAKE_BUILD_TYPE=%CONFIG% ^
    -DVVE_DEFAULT_VULKAN_ICD=system ^
    -DVVE_VCPKG_TRIPLET=x64-windows ^
    !VPE_CMAKE_ARGS!
if errorlevel 1 goto fail

cmake --build "%BUILD_DIR%"
if errorlevel 1 goto fail

if "%RUN_TESTS%"=="1" (
    ctest --test-dir "%BUILD_DIR%" -C %CONFIG% --output-on-failure
    if errorlevel 1 goto fail
)

echo.
if "%WITH_VVE%"=="1" (
    echo %CONFIG% build complete. Executable: %VVE_ROOT%\bin\%VARIANT%\exe\physicsexample.exe
) else (
    echo %CONFIG% build complete ^(VPE module only^). Library: %BUILD_DIR%\bin\%VARIANT%\lib
)
popd
exit /b 0

:usage
echo Usage: %~nx0 [debug^|release] [--clean] [--tests] [--without-vve]
popd
exit /b 1

:fail
echo.
echo Build failed.
popd
exit /b 1
