<#
.SYNOPSIS
  Deterministic build dispatcher for The Wrangler mode.
  Encodes .roomodes "Conditional Compilation Architecture" rule (rule 3) as code
  instead of relying on the model to remember it correctly every time.

.PARAMETER ChangedFiles
  List of file paths changed in this task (relative or absolute). If any .h/.hpp
  file is present, a full workspace rebuild is forced. Otherwise the build is
  targeted to the smallest sensible CMake target.

.PARAMETER Target
  Optional explicit CMake target to use for a targeted build (e.g. "Sketcher").
  Ignored if a header file triggers a full rebuild.

.NOTES
  Always wraps with vcvarsall.bat x64 per .wrangler_runbook.md:
  "Do not call pixi.exe run build directly. Raw execution drops standard
  library paths like <set> and <memory>, breaking compilation."
#>
param(
    [string[]]$ChangedFiles = @(),
    [string]$Target = "Sketcher"
)

$vcvarsall = "D:\PROGRA~2\MICROS~1\18\COMMUN~1\VC\Auxiliary\Build\vcvarsall.bat"

$headerChanged = $ChangedFiles | Where-Object { $_ -match '\.(h|hpp)$' }

if ($headerChanged) {
    Write-Output "HEADER CHANGE DETECTED ($($headerChanged -join ', ')) -> full workspace rebuild (pixi run build)"
    cmd /c "call `"$vcvarsall`" x64 && pixi run build"
}
else {
    Write-Output "No header changes -> targeted incremental build (target: $Target)"
    cmd /c "call `"$vcvarsall`" x64 && cmake --build build\debug --target $Target"
}

exit $LASTEXITCODE
