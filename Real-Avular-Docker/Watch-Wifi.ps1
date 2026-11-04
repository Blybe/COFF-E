# Read-only Windows WLAN monitor for the dashboard. No credentials are queried.
param(
    [string]$Adapter = 'WiFi 2',
    [switch]$Once
)

$ErrorActionPreference = 'Stop'
$folder = Join-Path $PSScriptRoot 'status'
[System.IO.Directory]::CreateDirectory($folder) | Out-Null
$path = Join-Path $folder 'wifi.json'
$mutex = New-Object System.Threading.Mutex($false, 'Local\COFFE-Real-Avular-Wifi')
if (-not $mutex.WaitOne(0)) {
    Write-Error 'A COFF-E Wi-Fi monitor is already running.'
    exit 1
}
try {
    do {
        $interfaces = @()
        $current = $null
        foreach ($line in (netsh wlan show interfaces)) {
            if ($line -match '^\s*Name\s*:\s*(.+)$') {
                $current = @{adapter = $Matches[1].Trim()}
                $interfaces += $current
            } elseif ($null -ne $current -and $line -match '^\s*(State|SSID|Signal)\s*:\s*(.*)$') {
                $key, $value = $Matches[1].ToLowerInvariant(), $Matches[2].Trim()
                if ($key -eq 'signal') {
                    $current[$key] = if ($value -match '^(\d+)%$') { [int]$Matches[1] } else { $null }
                } else {
                    $current[$key] = $value
                }
            }
        }
        $wifi = $interfaces | Where-Object { $_.adapter -eq $Adapter } | Select-Object -First 1
        $snapshot = @{updated = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() / 1000.0
                      adapter = $Adapter; state = 'unavailable'; ssid = $null; signal = $null}
        if ($wifi) {
            foreach ($key in @('state', 'ssid', 'signal')) { $snapshot[$key] = $wifi[$key] }
        }
        [System.IO.File]::WriteAllText("$path.tmp", ($snapshot | ConvertTo-Json -Compress),
                                      (New-Object System.Text.UTF8Encoding($false)))
        Move-Item -LiteralPath "$path.tmp" -Destination $path -Force
        if (-not $Once) { Start-Sleep -Seconds 3 }
    } while (-not $Once)
} finally {
    $mutex.ReleaseMutex()
    $mutex.Dispose()
}
