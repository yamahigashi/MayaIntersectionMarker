@echo off
setlocal EnableDelayedExpansion

SET PFX=%~dp0
SET EMBREE_DIR=%PFX%lib\embree
SET GLM_DIR=%PFX%lib\glm
rem SET COMPILER=Visual Studio 15 2017 Win64
SET COMPILER=Visual Studio 16 2019
rem SET COMPILER=Visual Studio 17 2022

call :build_for_maya 2024
call :build_for_maya 2023
call :build_for_maya 2022
call :build_for_maya 2020

call :deploy
goto :eof


REM --------------------------------------------------------------------------
:build_for_maya
SET MAYA_VERSION=%1
SET BUILD="build\%MAYA_VERSION%"
echo Building for Maya !MAYA_VERSION!

cd /d %PFX%
rmdir !BUILD! /s /q
mkdir !BUILD!
cd %BUILD%

REM Programmatically get the current Maya install location
for /f "skip=2 tokens=2*" %%A ^
in ('reg query "HKEY_LOCAL_MACHINE\SOFTWARE\Autodesk\Maya\%MAYA_VERSION%\Setup\InstallPath" /v "MAYA_INSTALL_LOCATION"') ^
do set "MAYALOC=%%B"
set ADLOC=%MAYALOC%..
cmake ^
    -DGLM_DIR=%GLM_DIR% ^
    -DMAYA_VERSION=%MAYA_VERSION% ^
    -DMAYA_INSTALL_BASE_PATH="%ADLOC%" ^
    -DCMAKE_INSTALL_PREFIX="..\..\modules" ^
    -G "%COMPILER%" ..\..\

cmake --build . --config Release --target Install


exit /b 0
REM --------------------------------------------------------------------------

:deploy
echo Deploying to Maya modules folder
cd /d %PFX%
mkdir modules\MayaIntersectionMarker
cd modules\MayaIntersectionMarker
mkdir scripts
mkdir shared
mkdir shared\win64

copy /y ..\..\MayaIntersectionMarker.mod ..\
copy /y ..\..\scripts\*.mel scripts
copy /y ..\..\lib\embree\bin\tbb12.dll shared\win64
copy /y ..\..\lib\embree\bin\embree4.dll shared\win64
copy /y ..\..\supplimental\colorscope\target\release\colorscope.exe shared\win64
copy /y ..\..\NOTICE.md .\

cd /d %PFX%\modules
set zipfile=MayaIntersectionMarker.zip
set source1=MayaIntersectionMarker
set source2=MayaIntersectionMarker.mod
PowerShell -Command "& {Compress-Archive -LiteralPath '%source1%', '%source2%' -DestinationPath '%zipfile%'}"

:eof
echo Build process completed.
