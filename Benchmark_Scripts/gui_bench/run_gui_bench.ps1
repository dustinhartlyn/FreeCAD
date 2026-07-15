<#
    run_gui_bench.ps1 — A/B GUI Sketcher responsiveness harness.

    Launches gui_bench.py inside each FreeCAD build's GUI, one after another,
    collects the JSON each writes, and prints a side-by-side comparison table.

      dev    = your pixi build. Launched via `pixi run <exe> gui_bench.py` so the
               conda/pixi environment (PATH to its DLLs) is activated. The freshly
               built Sketcher / SketcherGui pyds are copied into the env first.
      stock  = C:\Program Files\FreeCAD 1.1\bin\freecad.exe  (self-contained).

    Each child writes its results JSON and hard-exits; we poll for that file
    rather than the launcher's process handle (the pixi FreeCAD.exe is a thin
    launcher that can exit before the real GUI finishes).

    Usage (from the repo root):
      pwsh Benchmark_Scripts/gui_bench/run_gui_bench.ps1
      pwsh Benchmark_Scripts/gui_bench/run_gui_bench.ps1 -Sizes 50,100,200,300
      pwsh Benchmark_Scripts/gui_bench/run_gui_bench.ps1 -Builds dev
      pwsh Benchmark_Scripts/gui_bench/run_gui_bench.ps1 -Scenarios add_dimension,drag

    IMPORTANT
      * Close any FreeCAD GUI you have open first — a running GUI locks the pyds
        so the dev-pyd copy fails silently (you'd bench stale code), and stray
        processes are force-killed between runs.
      * The drag scenario warps the real mouse cursor (QTest grab-check). Don't
        touch the mouse while it runs.
#>
[CmdletBinding()]
param(
    [int[]]    $Sizes     = @(50, 100, 200),
    [string[]] $Scenarios = @('enter_edit', 'add_dimension', 'datum_edit', 'recompute', 'drag'),
    [ValidateSet('dev', 'stock')]
    [string[]] $Builds    = @('dev', 'stock'),
    [string]   $StockExe  = 'C:\Program Files\FreeCAD 1.1\bin\freecad.exe',
    [ValidateSet('debug', 'release')]
    [string]   $DevConfig  = 'debug',   # 'release' for a fair absolute A/B vs release-stock
    [int]      $TimeoutSec = 600,
    [int]      $WatchdogSec = 300
)

$ErrorActionPreference = 'Stop'
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot  = (Resolve-Path (Join-Path $scriptDir '..\..')).Path
$benchPy   = Join-Path $scriptDir 'gui_bench.py'
$outDir    = Join-Path $scriptDir 'results'
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

$sizesCsv     = ($Sizes -join ',')
$scenariosCsv = ($Scenarios -join ',')

function Stop-FreeCAD {
    Get-Process -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -match 'freecad' } |
        Stop-Process -Force -ErrorAction SilentlyContinue
    Start-Sleep -Milliseconds 500
}

function Sync-DevPyd {
    # install-debug refreshes Library/lib but NOT Library/Mod/Sketcher — copy the
    # freshly-built Sketcher + SketcherGui pyds so the GUI loads current code.
    $src = Join-Path $repoRoot "build\$DevConfig\Mod\Sketcher"
    $dst = Join-Path $repoRoot '.pixi\envs\default\Library\Mod\Sketcher'
    if (-not (Test-Path $src)) { throw "dev $DevConfig build not found at $src (run: pixi run build-$DevConfig)" }
    Write-Host "  dev config: $DevConfig" -ForegroundColor DarkGray
    foreach ($pyd in @('Sketcher.pyd', 'SketcherGui.pyd')) {
        $s = Join-Path $src $pyd
        if (Test-Path $s) {
            Copy-Item $s (Join-Path $dst $pyd) -Force
            Write-Host "  synced $pyd" -ForegroundColor DarkGray
        }
    }
}

function Wait-Result {
    param([System.Management.Automation.Job]$Job, [string]$OutFile, [string]$Name)
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while (-not (Test-Path $OutFile) -and (Get-Date) -lt $deadline) {
        if ($Job.State -eq 'Completed' -and -not (Test-Path $OutFile)) {
            Start-Sleep -Milliseconds 1500   # grace for the file flush
            break
        }
        Start-Sleep -Milliseconds 500
    }
    Stop-Job $Job -ErrorAction SilentlyContinue
    $log = Receive-Job $Job -ErrorAction SilentlyContinue | Out-String
    Remove-Job $Job -Force -ErrorAction SilentlyContinue
    if (-not (Test-Path $OutFile)) {
        Write-Host "  [$Name] NO RESULTS (crash on startup?). Launcher log tail:" -ForegroundColor Red
        Write-Host ($log -split "`n" | Select-Object -Last 12 | Out-String) -ForegroundColor DarkGray
        return $null
    }
    # Retry parse in case we caught a partial write.
    for ($k = 0; $k -lt 6; $k++) {
        try { return Get-Content $OutFile -Raw | ConvertFrom-Json }
        catch { Start-Sleep -Milliseconds 400 }
    }
    Write-Host "  [$Name] results file did not parse as JSON" -ForegroundColor Red
    return $null
}

function Invoke-DevBuild {
    Sync-DevPyd
    $exe = Join-Path $repoRoot '.pixi\envs\default\Library\bin\FreeCAD.exe'
    if (-not (Test-Path $exe)) { throw "dev FreeCAD.exe not found at $exe" }
    $out = Join-Path $outDir 'dev.json'
    foreach ($f in @($out, "$out.progress")) { if (Test-Path $f) { Remove-Item $f -Force } }
    Write-Host "`n=== [dev] pixi run $exe $benchPy" -ForegroundColor Cyan
    $job = Start-Job -ScriptBlock {
        param($repo, $exe, $py, $out, $sizes, $scen, $wd)
        Set-Location $repo
        $env:GUI_BENCH_OUT = $out; $env:GUI_BENCH_LABEL = 'dev'
        $env:GUI_BENCH_SIZES = $sizes; $env:GUI_BENCH_SCENARIOS = $scen
        $env:GUI_BENCH_WATCHDOG = $wd
        & pixi run $exe $py 2>&1 | Out-String
    } -ArgumentList $repoRoot, $exe, $benchPy, $out, $sizesCsv, $scenariosCsv, $WatchdogSec
    return (Wait-Result -Job $job -OutFile $out -Name 'dev')
}

function Invoke-StockBuild {
    if (-not (Test-Path $StockExe)) { throw "stock FreeCAD not found at $StockExe" }
    $out = Join-Path $outDir 'stock.json'
    foreach ($f in @($out, "$out.progress")) { if (Test-Path $f) { Remove-Item $f -Force } }
    Write-Host "`n=== [stock-1.1] $StockExe $benchPy" -ForegroundColor Cyan
    $job = Start-Job -ScriptBlock {
        param($exe, $py, $out, $sizes, $scen, $wd)
        $env:GUI_BENCH_OUT = $out; $env:GUI_BENCH_LABEL = 'stock-1.1'
        $env:GUI_BENCH_SIZES = $sizes; $env:GUI_BENCH_SCENARIOS = $scen
        $env:GUI_BENCH_WATCHDOG = $wd
        & $exe $py 2>&1 | Out-String
    } -ArgumentList $StockExe, $benchPy, $out, $sizesCsv, $scenariosCsv, $WatchdogSec
    return (Wait-Result -Job $job -OutFile $out -Name 'stock-1.1')
}

# --------------------------------------------------------------------------
# Run each requested build
# --------------------------------------------------------------------------
$data = @{}
foreach ($b in $Builds) {
    Stop-FreeCAD
    if ($b -eq 'dev')   { $data['dev']   = Invoke-DevBuild }
    else                { $data['stock'] = Invoke-StockBuild }
    Stop-FreeCAD
}

# --------------------------------------------------------------------------
# Provenance
# --------------------------------------------------------------------------
Write-Host "`n================= PROVENANCE =================" -ForegroundColor Yellow
foreach ($k in @('dev', 'stock')) {
    if ($data.ContainsKey($k) -and $data[$k]) {
        $d = $data[$k]
        Write-Host ("  {0,-6} FreeCAD {1}  Qt={2}" -f $k, $d.freecad_version, $d.qt)
        Write-Host ("         Sketcher.pyd: {0}" -f $d.sketcher_pyd) -ForegroundColor DarkGray
    }
}

# --------------------------------------------------------------------------
# size -> scenario -> {ms,note} per build
# --------------------------------------------------------------------------
function Get-Table($d) {
    $t = @{}
    if (-not $d) { return $t }
    foreach ($r in $d.results) {
        if ($r.PSObject.Properties.Name -contains 'error') { continue }
        $sz = [int]$r.size
        $t[$sz] = @{}
        foreach ($sc in $r.scenarios.PSObject.Properties) {
            $val = $sc.Value
            $ms = if ($val.PSObject.Properties.Name -contains 'ms') { [double]$val.ms } else { $null }
            $note = ''
            if ($val.PSObject.Properties.Name -contains 'error') { $note = ' (ERROR)' }
            if ($sc.Name -eq 'drag') {
                if (($val.PSObject.Properties.Name -contains 'measurement_ok') -and (-not $val.measurement_ok)) {
                    $note += ' (INVALID — tip did not move)'
                }
                if (($val.PSObject.Properties.Name -contains 'grabbable') -and (-not $val.grabbable)) {
                    $note += ' (grab-check n/a)'
                }
            }
            $t[$sz][$sc.Name] = @{ ms = $ms; note = $note }
        }
    }
    return $t
}
$devT   = Get-Table $data['dev']
$stockT = Get-Table $data['stock']

# --------------------------------------------------------------------------
# A/B table
# --------------------------------------------------------------------------
Write-Host "`n============ A/B RESULTS (ms; lower is better) ============" -ForegroundColor Green
$fmt = "{0,-14}{1,7}{2,12}{3,12}{4,10}"
Write-Host ($fmt -f 'scenario', 'size', 'stock-1.1', 'dev', 'speedup')
Write-Host ('-' * 55)
$allSizes = @($Sizes | Sort-Object)
foreach ($sc in $Scenarios) {
    foreach ($sz in $allSizes) {
        $sv = if ($stockT.ContainsKey($sz) -and $stockT[$sz].ContainsKey($sc)) { $stockT[$sz][$sc].ms } else { $null }
        $dv = if ($devT.ContainsKey($sz)   -and $devT[$sz].ContainsKey($sc))   { $devT[$sz][$sc].ms }   else { $null }
        $note = ''
        if ($devT.ContainsKey($sz)   -and $devT[$sz].ContainsKey($sc))   { $note += $devT[$sz][$sc].note }
        if ($stockT.ContainsKey($sz) -and $stockT[$sz].ContainsKey($sc)) { $note += $stockT[$sz][$sc].note }
        $sStr = if ($null -ne $sv) { '{0:N1}' -f $sv } else { '-' }
        $dStr = if ($null -ne $dv) { '{0:N1}' -f $dv } else { '-' }
        $spd  = if ($sv -and $dv -and $dv -gt 0) { '{0:N2}x' -f ($sv / $dv) } else { '-' }
        $line = $fmt -f $sc, $sz, $sStr, $dStr, $spd
        if ($note) { $line += $note }
        $color = if ($note -match 'INVALID|ERROR') { 'Red' } else { 'White' }
        Write-Host $line -ForegroundColor $color
    }
}

# --------------------------------------------------------------------------
# CSV
# --------------------------------------------------------------------------
$csv = Join-Path $outDir 'ab_summary.csv'
$rows = foreach ($sc in $Scenarios) {
    foreach ($sz in $allSizes) {
        $sv = if ($stockT.ContainsKey($sz) -and $stockT[$sz].ContainsKey($sc)) { $stockT[$sz][$sc].ms } else { $null }
        $dv = if ($devT.ContainsKey($sz)   -and $devT[$sz].ContainsKey($sc))   { $devT[$sz][$sc].ms }   else { $null }
        [pscustomobject]@{
            scenario = $sc; size = $sz; stock_ms = $sv; dev_ms = $dv
            speedup  = if ($sv -and $dv -and $dv -gt 0) { [math]::Round($sv / $dv, 3) } else { $null }
        }
    }
}
$rows | Export-Csv -Path $csv -NoTypeInformation -Encoding utf8
Write-Host "`nWrote $csv" -ForegroundColor DarkGray
Write-Host "Raw JSON: $outDir\dev.json , $outDir\stock.json" -ForegroundColor DarkGray
