@echo off

setlocal
set OSFMBASE=%~dp0
if not defined OPENCV_LIBS set OPENCV_LIBS=%OSFMBASE%\..\vcpkg\installed\x64-windows\bin

python "%OSFMBASE%\opensfm_main.py" %*

endlocal
