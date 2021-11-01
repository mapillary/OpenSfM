@echo off

setlocal
set OSFMBASE=%~dp0

python "%OSFMBASE%\opensfm_main.py" %*

endlocal
