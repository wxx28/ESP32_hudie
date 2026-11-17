@echo off
rem 仿生蝴蝶飞行仿真：双击运行。需要 Node.js；第一次运行需要 gcc（MSYS2 mingw64）在 PATH 里。
cd /d "%~dp0"
where gcc >nul 2>nul || if exist "D:\MSYS2\mingw64\bin\gcc.exe" set "PATH=D:\MSYS2\mingw64\bin;%PATH%"
start "" http://localhost:8090
node server.js
pause
