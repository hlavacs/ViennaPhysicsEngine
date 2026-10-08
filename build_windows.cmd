@echo off
setlocal EnableExtensions DisableDelayedExpansion

rem Build the C++23 VPE module with Ninja + x64 MSVC. Like the Linux/macOS
rem scripts, standalone builds also compile and run the VPE tests. The default
rem build includes physicsexample and compiles VVE in-tree for compatible IFCs.
rem Requires Visual Studio's C++ workload and CMake tools; the example also
rem requires the Vulkan SDK and vcpkg; its manifest dependencies are installed
rem automatically in the adjacent VVE checkout before CMake is configured.

pushd "%~dp0" || exit /b 1
set "REPO_ROOT=%CD%"
set "VARIANT=release"
set "CLEAN=0"
set "WITHOUT_VVE=0"
set "RUN_TESTS=0"
set "STAGE=Prerequisite checks"
rem vcvars64 may replace VCPKG_ROOT with Visual Studio's bundled installation.
set "REQUESTED_VCPKG_ROOT=%VCPKG_ROOT%"

:parse
if "%~1"=="" goto done_parse
if /I "%~1"=="debug" (set "VARIANT=debug"& goto next_arg)
if /I "%~1"=="release" (set "VARIANT=release"& goto next_arg)
if /I "%~1"=="--clean" (set "CLEAN=1"& goto next_arg)
if /I "%~1"=="--without-vve" (set "WITHOUT_VVE=1"& goto next_arg)
if /I "%~1"=="--standalone" (set "WITHOUT_VVE=1"& goto next_arg)
if /I "%~1"=="--tests" (set "RUN_TESTS=1"& goto next_arg)
if /I "%~1"=="-h" goto help
if /I "%~1"=="--help" goto help
echo Unknown argument: "%~1"
call :usage
goto fail
:next_arg
shift
goto parse

:done_parse
if /I "%VARIANT%"=="debug" (set "CONFIG=Debug") else (set "CONFIG=Release")
set "BUILD_SUFFIX="
if "%WITHOUT_VVE%"=="1" (
    set "BUILD_SUFFIX=-standalone"
    set "RUN_TESTS=1"
)
set "BUILD_DIR=%REPO_ROOT%\build\%VARIANT%-windows%BUILD_SUFFIX%"
set "JOBS=%CMAKE_BUILD_PARALLEL_LEVEL%"
if not defined JOBS set "JOBS=%NUMBER_OF_PROCESSORS%"
if not defined JOBS set "JOBS=8"

rem Initialize all build tools, including Ninja, from ordinary PowerShell/cmd.
set "NEED_VS=0"
if /I not "%VSCMD_ARG_TGT_ARCH%"=="x64" set "NEED_VS=1"
for %%T in (cl.exe cmake.exe ninja.exe) do (
    where %%T >nul 2>nul
    if errorlevel 1 set "NEED_VS=1"
)
if "%NEED_VS%"=="1" (
    call :init_msvc
    if errorlevel 1 goto fail
)
for %%T in (cl.exe cmake.exe ninja.exe) do (
    where %%T >nul 2>nul
    if errorlevel 1 (
        echo %%T not found. Install the C++ workload and C++ CMake tools in Visual Studio Installer.
        goto fail
    )
)
set "CMAKE_BIN="
for /f "delims=" %%T in ('where cmake.exe 2^>nul') do if not defined CMAKE_BIN set "CMAKE_BIN=%%T"

rem Use CTest from the same installation as CMake, even if it is not on PATH.
set "CTEST_BIN="
if "%RUN_TESTS%"=="1" (
    call :find_ctest
    if errorlevel 1 goto fail
)

set "VVE_PREFIX_ARG="
set "VPE_CMAKE_ARGS=-DVPE_BUILD_EXAMPLES=OFF -DVPE_BUILD_TESTS=OFF -DVPE_VVE_IN_TREE=OFF"
if "%WITHOUT_VVE%"=="0" (
    call :check_vve
    if errorlevel 1 goto fail
    set "STAGE=VVE dependency installation"
    call :install_vve_dependencies
    if errorlevel 1 goto fail
    set "VPE_CMAKE_ARGS=-DVPE_BUILD_EXAMPLES=ON -DVPE_BUILD_TESTS=OFF -DVPE_VVE_IN_TREE=ON -DVVE_DEFAULT_VULKAN_ICD=system -DVVE_VCPKG_TRIPLET=x64-windows -DVVE_ENGINE_IMPLEMENTATION_NAMESPACE=simple"
)
if "%RUN_TESTS%"=="1" set "VPE_CMAKE_ARGS=%VPE_CMAKE_ARGS% -DVPE_BUILD_TESTS=ON"

if "%CLEAN%"=="0" call :check_cache
if "%CLEAN%"=="1" (
    set "STAGE=Build directory cleanup"
    call :clean_build
    if errorlevel 1 goto fail
)

set "STAGE=CMake configuration"
"%CMAKE_BIN%" -S "%REPO_ROOT%" -B "%BUILD_DIR%" -G Ninja ^
    -DCMAKE_BUILD_TYPE=%CONFIG% ^
    -DCMAKE_CXX_COMPILER=cl.exe ^
    %VPE_CMAKE_ARGS% %VVE_PREFIX_ARG%
if errorlevel 1 goto fail

set "STAGE=Compilation and linking"
"%CMAKE_BIN%" --build "%BUILD_DIR%" --target all --parallel "%JOBS%"
if errorlevel 1 goto fail

if "%RUN_TESTS%"=="1" (
    set "STAGE=Tests"
    "%CTEST_BIN%" --test-dir "%BUILD_DIR%" -C %CONFIG% --output-on-failure
    if errorlevel 1 goto fail
)

echo.
echo %CONFIG% build complete.
echo Library: "%BUILD_DIR%\bin\%VARIANT%\lib\ViennaPhysicsEngine.lib"
if "%WITHOUT_VVE%"=="1" (
    echo Executables: "%REPO_ROOT%\bin\%VARIANT%\exe"
) else (
    echo Executable: "%VVE_ROOT%\bin\%VARIANT%\exe\physicsexample.exe"
)
popd
exit /b 0

:init_msvc
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
    echo vswhere.exe not found. Run from an x64 Native Tools Command Prompt for Visual Studio.
    exit /b 1
)
set "VSINSTALL="
for /f "usebackq delims=" %%V in (`"%VSWHERE%" -latest -prerelease -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINSTALL=%%V"
if not defined VSINSTALL (
    echo No Visual Studio installation with the C++ toolchain was found.
    exit /b 1
)
echo Initializing x64 MSVC from "%VSINSTALL%" ...
call "%VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
rem Keep an existing CMake first: it may be newer than Visual Studio's copy.
set "PATH=%PATH%;%VSINSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin;%VSINSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja"
exit /b 0

:find_ctest
for %%T in ("%CMAKE_BIN%") do if exist "%%~dpTctest.exe" set "CTEST_BIN=%%~dpTctest.exe"
if not defined CTEST_BIN for /f "delims=" %%T in ('where ctest.exe 2^>nul') do if not defined CTEST_BIN set "CTEST_BIN=%%T"
if defined CTEST_BIN exit /b 0
echo ctest.exe not found. Install CMake with CTest to run the VPE tests.
exit /b 1

:check_vve
rem Match CMake's sibling checkout and one-level-deeper worktree fallback.
for %%D in ("%REPO_ROOT%\..\ViennaVulkanEngine") do set "VVE_ROOT=%%~fD"
if not exist "%VVE_ROOT%\src\Engine.ixx" for %%D in ("%REPO_ROOT%\..\..\ViennaVulkanEngine") do set "VVE_ROOT=%%~fD"
if not exist "%VVE_ROOT%\src\Engine.ixx" (
    echo ViennaVulkanEngine was not found next to ViennaPhysicsEngine.
    echo Clone it there, or use --without-vve to build and test VPE by itself.
    exit /b 1
)
if not defined VULKAN_SDK if defined VK_SDK_PATH set "VULKAN_SDK=%VK_SDK_PATH%"
if not defined VULKAN_SDK (
    echo VULKAN_SDK is not set. Install the Vulkan SDK, or use --without-vve.
    exit /b 1
)
if not exist "%VULKAN_SDK%\Include\vulkan\vulkan.h" (
    echo Vulkan headers were not found under "%VULKAN_SDK%". Check VULKAN_SDK.
    exit /b 1
)
if not exist "%VVE_ROOT%\vcpkg.json" (
    echo VVE's vcpkg manifest was not found at "%VVE_ROOT%\vcpkg.json".
    exit /b 1
)
exit /b 0

:install_vve_dependencies
rem Install the entire VVE manifest, including features and transitive packages.
if defined REQUESTED_VCPKG_ROOT set "VCPKG_ROOT=%REQUESTED_VCPKG_ROOT%"
set "VCPKG_EXE="
if defined VCPKG_ROOT (
    if not exist "%VCPKG_ROOT%\vcpkg.exe" (
        echo vcpkg.exe was not found under VCPKG_ROOT="%VCPKG_ROOT%".
        exit /b 1
    )
    set "VCPKG_EXE=%VCPKG_ROOT%\vcpkg.exe"
)
if not defined VCPKG_EXE for /f "delims=" %%T in ('where vcpkg.exe 2^>nul') do if not defined VCPKG_EXE set "VCPKG_EXE=%%T"
if not defined VCPKG_EXE if exist "C:\vcpkg\vcpkg.exe" set "VCPKG_EXE=C:\vcpkg\vcpkg.exe"
if not defined VCPKG_EXE (
    echo vcpkg was not found. Set VCPKG_ROOT to its directory or add vcpkg.exe to PATH.
    exit /b 1
)
for %%T in ("%VCPKG_EXE%") do for %%D in ("%%~dpT.") do set "VCPKG_ROOT=%%~fD"
rem Run from VVE so vcpkg resolves its manifest and overlay configuration there.
pushd "%VVE_ROOT%" || exit /b 1
echo Synchronizing VVE dependencies using "%VCPKG_EXE%" ...
"%VCPKG_EXE%" install --triplet x64-windows --vcpkg-root "%VCPKG_ROOT%" --x-install-root "%VVE_ROOT%\vcpkg_installed"
set "INSTALL_EXIT_CODE=%errorlevel%"
popd
if not "%INSTALL_EXIT_CODE%"=="0" exit /b %INSTALL_EXIT_CODE%
rem Give the parent project and all subdirectories the same package search root.
set VVE_PREFIX_ARG="-DCMAKE_PREFIX_PATH:PATH=%VVE_ROOT%\vcpkg_installed\x64-windows"
exit /b 0

:check_cache
rem Legacy scripts shared the example and standalone cache; std-module support
rem must be discovered before project(), so recreate incompatible old caches.
if not exist "%BUILD_DIR%\CMakeCache.txt" exit /b 0
findstr /x /c:"CMAKE_GENERATOR:INTERNAL=Ninja" "%BUILD_DIR%\CMakeCache.txt" >nul
if errorlevel 1 set "CLEAN=1"
for /d %%D in ("%BUILD_DIR%\CMakeFiles\*") do if exist "%%D\CMakeCXXCompiler.cmake" (
    findstr /r /c:"^set(CMAKE_CXX_COMPILER_ID.*MSVC" "%%D\CMakeCXXCompiler.cmake" >nul
    if errorlevel 1 set "CLEAN=1"
    if "%WITHOUT_VVE%"=="0" (
        findstr /c:"support not enabled when detecting toolchain" "%%D\CMakeCXXCompiler.cmake" >nul
        if not errorlevel 1 set "CLEAN=1"
    )
)
if "%CLEAN%"=="1" echo Existing build cache is incompatible; recreating "%BUILD_DIR%".
exit /b 0

:clean_build
rem Verify the absolute target against the four script-owned build directories.
set "SAFE_CLEAN=0"
for %%D in (debug-windows release-windows debug-windows-standalone release-windows-standalone) do (
    if /I "%BUILD_DIR%"=="%REPO_ROOT%\build\%%D" set "SAFE_CLEAN=1"
)
if "%SAFE_CLEAN%"=="0" (
    echo Refusing to remove unexpected build directory: "%BUILD_DIR%".
    exit /b 1
)
if not exist "%BUILD_DIR%" exit /b 0
echo Removing "%BUILD_DIR%" ...
"%CMAKE_BIN%" -E remove_directory "%BUILD_DIR%"
exit /b %errorlevel%

:usage
echo Usage: %~nx0 [debug^|release] [--clean] [--without-vve] [--tests]
echo        Default: release, including the VVE-rendered physics example.
echo        --without-vve ^(or --standalone^) builds and tests only VPE.
echo        --tests also enables VPE tests when building the VVE example.
echo        CMAKE_BUILD_PARALLEL_LEVEL overrides the number of build jobs.
exit /b 0

:help
call :usage
popd
exit /b 0

:fail
echo.
echo %STAGE% failed.
popd
exit /b 1
