@echo off
REM Build script for ViennaPhysicsEngine using MSVC
REM Supports Visual Studio 2022 and 2026
REM Compiles physicsexample only if ViennaVulkanEngine1.1 is available in parent directory

setlocal enabledelayedexpansion

echo ========================================
echo ViennaPhysicsEngine MSVC Build Script
echo ========================================
echo.

REM Check for MSVC 2022 first (preferred)
set "MSVC_YEAR="
set "MSVC_VERSION="
set "VCVARSALL="

if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" (
    set "VCVARSALL=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat"
    set "MSVC_YEAR=2022"
    set "MSVC_VERSION=17"
) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvarsall.bat" (
    set "VCVARSALL=C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvarsall.bat"
    set "MSVC_YEAR=2022"
    set "MSVC_VERSION=17"
) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvarsall.bat" (
    set "VCVARSALL=C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvarsall.bat"
    set "MSVC_YEAR=2022"
    set "MSVC_VERSION=17"
) else if exist "C:\Program Files\Microsoft Visual Studio\2026\Community\VC\Auxiliary\Build\vcvarsall.bat" (
    set "VCVARSALL=C:\Program Files\Microsoft Visual Studio\2026\Community\VC\Auxiliary\Build\vcvarsall.bat"
    set "MSVC_YEAR=2026"
    set "MSVC_VERSION=18"
) else if exist "C:\Program Files\Microsoft Visual Studio\2026\Professional\VC\Auxiliary\Build\vcvarsall.bat" (
    set "VCVARSALL=C:\Program Files\Microsoft Visual Studio\2026\Professional\VC\Auxiliary\Build\vcvarsall.bat"
    set "MSVC_YEAR=2026"
    set "MSVC_VERSION=18"
) else if exist "C:\Program Files\Microsoft Visual Studio\2026\Enterprise\VC\Auxiliary\Build\vcvarsall.bat" (
    set "VCVARSALL=C:\Program Files\Microsoft Visual Studio\2026\Enterprise\VC\Auxiliary\Build\vcvarsall.bat"
    set "MSVC_YEAR=2026"
    set "MSVC_VERSION=18"
)

if "%VCVARSALL%"=="" (
    echo ERROR: No supported Visual Studio installation found!
    echo Please install Visual Studio 2022 or 2026.
    exit /b 1
)

echo Found Visual Studio %MSVC_YEAR%
echo Initializing MSVC environment...
call "%VCVARSALL%" x64
if errorlevel 1 (
    echo ERROR: Failed to initialize MSVC environment
    exit /b 1
)
echo.

REM Parse command line arguments
set "BUILD_CONFIG=Release"
if "%1"=="Debug" set "BUILD_CONFIG=Debug"
if "%1"=="debug" set "BUILD_CONFIG=Debug"
if "%1"=="Release" set "BUILD_CONFIG=Release"
if "%1"=="release" set "BUILD_CONFIG=Release"

echo Build Configuration: %BUILD_CONFIG%
echo.

REM Set up paths
set "SCRIPT_DIR=%~dp0"
set "PROJECT_ROOT=%SCRIPT_DIR%"
set "BUILD_DIR=%PROJECT_ROOT%build"
set "BUILD_CONFIG_DIR=%BUILD_DIR%\%BUILD_CONFIG%"
set "INCLUDE_DIR=%PROJECT_ROOT%include"
set "EXAMPLES_DIR=%PROJECT_ROOT%examples"
set "VULKAN_ENGINE=%PROJECT_ROOT%..\ViennaVulkanEngine1.1"

REM Check if ViennaVulkanEngine1.1 exists
if not exist "%VULKAN_ENGINE%" (
    echo WARNING: ViennaVulkanEngine1.1 not found in parent directory
    echo Expected location: %VULKAN_ENGINE%
    echo Skipping physicsexample build.
    echo.
    echo ViennaPhysicsEngine is header-only, nothing to build.
    exit /b 0
)

echo Found ViennaVulkanEngine1.1
echo Proceeding with physicsexample build...
echo.

REM Check for Vulkan SDK
if "%VULKAN_SDK%"=="" (
    echo ERROR: VULKAN_SDK environment variable not set
    echo Please install the Vulkan SDK from https://vulkan.lunarg.com/
    exit /b 1
)

echo Vulkan SDK: %VULKAN_SDK%
echo.

REM Create build directory
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

REM Set compiler flags with parallel build support
set "CXX_FLAGS=/std:c++20 /EHsc /W3 /nologo /MP"
if "%BUILD_CONFIG%"=="Debug" (
    set "CXX_FLAGS=%CXX_FLAGS% /MDd /Zi /Od /D_DEBUG"
    set "LINK_FLAGS=/DEBUG"
) else (
    set "CXX_FLAGS=%CXX_FLAGS% /MD /O2 /Oi /GL /DNDEBUG"
    set "LINK_FLAGS=/LTCG /OPT:REF /OPT:ICF"
)

REM Set include directories
set "INCLUDE_PATHS=/I"%INCLUDE_DIR%" /I"%VULKAN_SDK%\Include" /I"%VULKAN_ENGINE%" /I"%VULKAN_ENGINE%\VulkanEngine""
set "INCLUDE_PATHS=%INCLUDE_PATHS% /I"%VULKAN_ENGINE%\external\Assimp\include""
set "INCLUDE_PATHS=%INCLUDE_PATHS% /I"%VULKAN_ENGINE%\external\irrKlang\include""
set "INCLUDE_PATHS=%INCLUDE_PATHS% /I"%VULKAN_ENGINE%\external\glm""
set "INCLUDE_PATHS=%INCLUDE_PATHS% /I"%VULKAN_ENGINE%\external\threadpool""
set "INCLUDE_PATHS=%INCLUDE_PATHS% /I"%VULKAN_ENGINE%\external\stb""
set "INCLUDE_PATHS=%INCLUDE_PATHS% /I"%VULKAN_ENGINE%\external\nuklear""
set "INCLUDE_PATHS=%INCLUDE_PATHS% /I"%VULKAN_ENGINE%\external\glfw\include""

REM Set library paths and libraries
if "%BUILD_CONFIG%"=="Debug" (
    set "ASSIMP_LIBS=%VULKAN_ENGINE%\external\Assimp\lib\x64\Debug\assimp-vc140-mt.lib"
    set "ASSIMP_LIBS=!ASSIMP_LIBS! %VULKAN_ENGINE%\external\Assimp\lib\x64\Debug\IrrXML.lib"
    set "ASSIMP_LIBS=!ASSIMP_LIBS! %VULKAN_ENGINE%\external\Assimp\lib\x64\Debug\zlibd.lib"
    set "VULKAN_ENGINE_LIB=%VULKAN_ENGINE%\lib\Debug\vulkanengine.lib"
) else (
    set "ASSIMP_LIBS=%VULKAN_ENGINE%\external\Assimp\lib\x64\Release\assimp-vc140-mt.lib"
    set "ASSIMP_LIBS=!ASSIMP_LIBS! %VULKAN_ENGINE%\external\Assimp\lib\x64\Release\IrrXML.lib"
    set "ASSIMP_LIBS=!ASSIMP_LIBS! %VULKAN_ENGINE%\external\Assimp\lib\x64\Release\zlib.lib"
    set "VULKAN_ENGINE_LIB=%VULKAN_ENGINE%\lib\Release\vulkanengine.lib"
)

set "GLFW_LIB=%VULKAN_ENGINE%\external\glfw\lib-vc2022\glfw3.lib"
set "SYSTEM_LIBS=gdi32.lib user32.lib shell32.lib"
set "LIBS=%VULKAN_ENGINE_LIB% !ASSIMP_LIBS! %GLFW_LIB% %SYSTEM_LIBS%"

REM Output directory
set "OUTPUT_DIR=%VULKAN_ENGINE%\bin\%BUILD_CONFIG%"
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"

echo Building physicsexample...
echo Source files:
echo   - %EXAMPLES_DIR%\physicsexample\physicsexample.cpp
echo   - %EXAMPLES_DIR%\physicsexample\VPEConstraintDemos.cpp
echo.
echo Output: %OUTPUT_DIR%\physicsexample.exe
echo.

REM Compile source files
echo Compiling physicsexample.cpp...
cl %CXX_FLAGS% %INCLUDE_PATHS% /c "%EXAMPLES_DIR%\physicsexample\physicsexample.cpp" /Fo"%BUILD_DIR%\physicsexample.obj"
if errorlevel 1 (
    echo ERROR: Failed to compile physicsexample.cpp
    exit /b 1
)

echo Compiling VPEConstraintDemos.cpp...
cl %CXX_FLAGS% %INCLUDE_PATHS% /c "%EXAMPLES_DIR%\physicsexample\VPEConstraintDemos.cpp" /Fo"%BUILD_DIR%\VPEConstraintDemos.obj"
if errorlevel 1 (
    echo ERROR: Failed to compile VPEConstraintDemos.cpp
    exit /b 1
)

REM Link executable
echo Linking physicsexample.exe...
link /NOLOGO %LINK_FLAGS% /OUT:"%OUTPUT_DIR%\physicsexample.exe" "%BUILD_DIR%\physicsexample.obj" "%BUILD_DIR%\VPEConstraintDemos.obj" %LIBS%
if errorlevel 1 (
    echo ERROR: Failed to link physicsexample.exe
    exit /b 1
)

echo.
echo ========================================
echo Build completed successfully!
echo ========================================
echo Configuration: %BUILD_CONFIG%
echo Compiler: Visual Studio %MSVC_YEAR%
echo Output: %OUTPUT_DIR%\physicsexample.exe
echo.
echo To run the example:
echo   cd "%VULKAN_ENGINE%\bin\%BUILD_CONFIG%"
echo   physicsexample.exe
echo.

endlocal
exit /b 0
