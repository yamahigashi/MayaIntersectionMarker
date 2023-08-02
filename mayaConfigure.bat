echo off
setlocal

SET MAYA_VERSION=2024
SET BUILD=mayabuild_%MAYA_VERSION%
rem SET COMPILER=Visual Studio 15 2017 Win64
SET COMPILER=Visual Studio 16 2019
rem SET COMPILER=Visual Studio 17 2022


SET PFX=%~dp0
SET EMBREE_DIR=%PFX%lib\embree
SET GLM_DIR=%PFX%lib\glm
cd /d %PFX%
rmdir %BUILD% /s /q
mkdir %BUILD%
cd %BUILD%


REM I like to move folders and installs around, so I'm adding some code to programmatically get some folder locations
REM If these lines don't work, you can just hard code MYDOCUMENTS and ADLOC

REM And programmatically get the current Maya install location
REM I will get the parent of this location (ADLOC) to use as the MAYA_INSTALL_BASE_PATH
for /f "skip=2 tokens=2*" %%A ^
in ('reg query "HKEY_LOCAL_MACHINE\SOFTWARE\Autodesk\Maya\%MAYA_VERSION%\Setup\InstallPath" /v "MAYA_INSTALL_LOCATION"') ^
do set "MAYALOC=%%B"
set ADLOC=%MAYALOC%..


cmake ^
    -DGLM_DIR=%GLM_DIR% ^
    -DMAYA_VERSION=%MAYA_VERSION% ^
    -DMAYA_INSTALL_BASE_PATH="%ADLOC%" ^
    -DCMAKE_INSTALL_PREFIX="..\modules" ^
    -G "%COMPILER%" ..\

cmake --build . --config Release --target Install
