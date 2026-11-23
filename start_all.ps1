# 一键启动本机两个服务
#
#   3000  server.py       数据服务器：订阅 MQTT、推给网页
#   8001  静态文件服务器    OTA 用：把 build\sample_project.bin 暴露出去给设备下载
#
# 用法：右键"用 PowerShell 运行"，或者
#     powershell -ExecutionPolicy Bypass -File .\start_all.ps1
#
# 用 pythonw.exe 启动 → 不弹控制台窗口 → 关掉这个脚本窗口服务也不会停。
# 要停止服务：Get-NetTCPConnection -LocalPort 3000 | % { Stop-Process -Id $_.OwningProcess -Force }

$ErrorActionPreference = 'Stop'

$ProjectDir = 'C:\Users\韦兴发\Desktop\smart device'
$BuildDir   = 'C:\ESP32\smart device\build'
$PythonW    = Join-Path $ProjectDir 'venv\Scripts\pythonw.exe'

if (-not (Test-Path $PythonW)) {
    Write-Host "找不到 $PythonW" -ForegroundColor Red
    Write-Host "venv 不在原位？改用 python.exe 也行：把脚本里的 pythonw.exe 换成 python.exe" -ForegroundColor Yellow
    exit 1
}

function Test-Port($port) {
    return [bool](Get-NetTCPConnection -LocalPort $port -State Listen -ErrorAction SilentlyContinue)
}

# ---------- 3000 数据服务器 ----------
if (Test-Port 3000) {
    Write-Host "[跳过] 3000 已经在监听，数据服务器不用重复启动" -ForegroundColor DarkGray
} else {
    Start-Process -FilePath $PythonW -ArgumentList 'server.py' -WorkingDirectory $ProjectDir
    Write-Host "[启动] 数据服务器 server.py (3000)" -ForegroundColor Green
}

# ---------- 8001 OTA 固件服务器 ----------
if (Test-Port 8001) {
    Write-Host "[跳过] 8001 已经在监听，静态服务器不用重复启动" -ForegroundColor DarkGray
} else {
    if (-not (Test-Path (Join-Path $BuildDir 'sample_project.bin'))) {
        Write-Host "[警告] build 里还没有 sample_project.bin，先编译再跑这个脚本" -ForegroundColor Yellow
    }
    Start-Process -FilePath $PythonW -ArgumentList '-m', 'http.server', '8001' -WorkingDirectory $BuildDir
    Write-Host "[启动] 固件服务器 (8001) → $BuildDir" -ForegroundColor Green
}

Start-Sleep -Seconds 3

# ---------- 打印本机地址 ----------
$ip = (Get-NetIPAddress -AddressFamily IPv4 |
       Where-Object { $_.IPAddress -notlike '127.*' -and $_.IPAddress -notlike '169.254.*' } |
       Where-Object { $_.InterfaceAlias -notlike '*VMware*' } |
       Select-Object -First 1).IPAddress

Write-Host ''
Write-Host '================ 访问地址 ================' -ForegroundColor Cyan
Write-Host "  本机网页   http://127.0.0.1:3000"
if ($ip) {
    Write-Host "  手机网页   http://${ip}:3000        （要和电脑同一个 WiFi）"
    Write-Host "  OTA 地址   http://${ip}:8001/sample_project.bin"
    Write-Host ''
    Write-Host "  把上面这个 OTA 地址填进 menuconfig → OTA → 固件下载地址，" -ForegroundColor DarkGray
    Write-Host "  或者在电脑上跑：ota_push.py <OTA地址>" -ForegroundColor DarkGray
} else {
    Write-Host '  没找到局域网 IP，检查 WiFi 是否连上' -ForegroundColor Yellow
}
Write-Host '==========================================' -ForegroundColor Cyan