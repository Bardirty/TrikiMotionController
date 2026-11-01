@echo off
setlocal
cd /d "%~dp0"

 echo ===============================================
 echo  Triki Motion Controller - Windows EXE builder
 echo ===============================================
 echo.

set "PYTHON=py"
if exist "%~dp0.venv\Scripts\python.exe" set "PYTHON=%~dp0.venv\Scripts\python.exe"

 echo [1/4] Installing application dependencies...
"%PYTHON%" -m pip install -r requirements.txt
if errorlevel 1 goto :error

 echo.
 echo [2/4] Installing PyInstaller...
"%PYTHON%" -m pip install --upgrade pyinstaller
if errorlevel 1 goto :error

 echo.
 echo [3/4] Cleaning previous build...
if exist build rmdir /s /q build
if exist dist rmdir /s /q dist
if exist TrikiMotionController.spec del /q TrikiMotionController.spec

 echo.
 echo [4/4] Building application...
"%PYTHON%" -m PyInstaller ^
  --noconfirm ^
  --clean ^
  --windowed ^
  --onedir ^
  --name TrikiMotionController ^
  --collect-all bleak ^
  --collect-submodules winrt ^
  --hidden-import bleak.backends.winrt.scanner ^
  --hidden-import bleak.backends.winrt.client ^
  TrikiMotionController.py
if errorlevel 1 goto :error

 echo.
 echo ===============================================
 echo  BUILD COMPLETE
 echo  EXE: dist\TrikiMotionController\TrikiMotionController.exe
 echo.
 echo  Share the whole dist\TrikiMotionController folder,
 echo  not the EXE file alone.
 echo ===============================================
 pause
 exit /b 0

:error
 echo.
 echo BUILD FAILED. Check the error messages above.
 pause
 exit /b 1
