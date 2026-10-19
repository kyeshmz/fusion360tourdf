@echo off
setlocal DisableDelayedExpansion
set "PYTHON_MANAGER_AUTOMATIC_INSTALL=false"
set "PYLAUNCHER_ALLOW_INSTALL="
set "PYLAUNCHER_ALWAYS_INSTALL="

if not exist "%~dp0tools\install_fusion.py" goto missing_backend

py.exe -3 -c "import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)" >nul 2>&1
if not errorlevel 1 goto run_py
python3.exe -c "import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)" >nul 2>&1
if not errorlevel 1 goto run_python3
python.exe -c "import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)" >nul 2>&1
if not errorlevel 1 goto run_python

>&2 echo Install failed: Python 3.11 or newer is required. Install Python and put py.exe, python3.exe or python.exe on PATH.
exit /b 1

:run_py
py.exe -3 "%~dp0tools\install_fusion.py" %*
exit /b %errorlevel%

:run_python3
python3.exe "%~dp0tools\install_fusion.py" %*
exit /b %errorlevel%

:run_python
python.exe "%~dp0tools\install_fusion.py" %*
exit /b %errorlevel%

:missing_backend
>&2 echo Install failed: missing tools\install_fusion.py. Use a complete checkout.
exit /b 1
