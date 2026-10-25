@echo off
rem Packs the built rollback RPCS3 (bin folder) into t5dr-rpcs3-<version>.zip for a GitHub release, WITHOUT your
rem games, firmware, saves, logs, caches or personal settings. The launcher downloads this zip for new players.
rem Usage: package_t5dr.bat            (version = current git commit)
rem        package_t5dr.bat 0.1.0      (version name of your choice)
setlocal
cd /d "%~dp0"
set VER=%1
if "%VER%"=="" for /f %%c in ('git rev-parse --short HEAD') do set VER=%%c
set OUT=%~dp0t5dr-rpcs3-%VER%.zip
if not exist bin\rpcs3.exe (echo bin\rpcs3.exe not found - build RPCS3 first & pause & exit /b 1)
powershell -NoProfile -Command ^
  "$ErrorActionPreference='Stop';" ^
  "$src = Join-Path (Get-Location) 'bin';" ^
  "$stage = Join-Path $env:TEMP 't5dr-rpcs3-pack';" ^
  "if (Test-Path $stage) { Remove-Item $stage -Recurse -Force };" ^
  "New-Item -ItemType Directory $stage | Out-Null;" ^
  "$skipDirs = 'dev_hdd0','dev_hdd1','dev_flash','dev_flash2','dev_flash3','dev_usb000','dev_bdvd','games','log','cache','shaderlog','captures','screenshots','savestates','patches','config','GuiConfigs','bin2';" ^
  "$skipFiles = '*.log','*.log.gz','t5_test*.bin','hash_*.txt','dump_*.bin','rb_input.bin','*.pdb','games.yml';" ^
  "Get-ChildItem $src | Where-Object { -not ($_.PSIsContainer -and $skipDirs -contains $_.Name) } | ForEach-Object { Copy-Item $_.FullName -Destination $stage -Recurse -Exclude $skipFiles };" ^
  "Get-ChildItem $stage -Recurse -Include $skipFiles | Remove-Item -Force;" ^
  "$cfg = Join-Path $src 'config\custom_configs\config_NPEA00019.yml';" ^
  "if (Test-Path $cfg) { New-Item -ItemType Directory (Join-Path $stage 'config\custom_configs') -Force | Out-Null; Copy-Item $cfg (Join-Path $stage 'config\custom_configs\') };" ^
  "if (Test-Path '%OUT%') { Remove-Item '%OUT%' };" ^
  "Compress-Archive -Path (Join-Path $stage '*') -DestinationPath '%OUT%';" ^
  "Remove-Item $stage -Recurse -Force;" ^
  "Write-Host ('Created ' + '%OUT%' + ' (' + [math]::Round((Get-Item '%OUT%').Length/1MB) + ' MB)')"
if errorlevel 1 (echo Packing failed & pause & exit /b 1)
echo.
echo Upload this zip to a new release on https://github.com/sambin0/rpcs3/releases (tag e.g. t5dr-%VER%).
pause
