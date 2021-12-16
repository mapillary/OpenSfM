call %~dp0opensfm.bat extract_metadata %1
call %~dp0opensfm.bat detect_features %1
call %~dp0opensfm.bat match_features %1
call %~dp0opensfm.bat create_tracks %1
call %~dp0opensfm.bat reconstruct %1
call %~dp0opensfm.bat mesh %1
call %~dp0opensfm.bat undistort %1
call %~dp0opensfm.bat compute_depthmaps %1
