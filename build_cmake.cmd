@echo off
rem Compatibility entry point; the Windows build is maintained in one script.
call "%~dp0build_windows.cmd" %*
exit /b %errorlevel%
