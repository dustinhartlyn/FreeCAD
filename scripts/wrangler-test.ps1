<#
.SYNOPSIS
  Deterministic test runner for The Wrangler mode. Wraps "pixi run test" (ctest)
  and classifies failures against the known pre-existing baseline, so the model
  doesn't have to re-derive "is this a regression or the same 4 Qt GUI failures
  we've had since Phase 5" every single run.

.NOTES
  Per .wrangler_runbook.md: direct "ctest -R Sketcher" without the pixi
  environment fails with 0xc0000135 (DLL not found). Always go through
  "pixi run test" for PATH resolution.
#>
param(
    [string[]]$KnownPreExistingFailures = @("19", "20", "21", "22")  # Qt GUI headless 0xc0000409, pre-existing
)

$output = pixi run test 2>&1 | Tee-Object -Variable testOutput
$exitCode = $LASTEXITCODE

$failedTests = @()
foreach ($line in $testOutput) {
    if ($line -match '^\s*(\d+)/\d+ Test\s+#(\d+).*\*\*\*Failed') {
        $failedTests += $Matches[2]
    }
}

$unexpectedFailures = $failedTests | Where-Object { $KnownPreExistingFailures -notcontains $_ }

Write-Output "----"
Write-Output "ctest exit code: $exitCode"
Write-Output "Failed test numbers: $($failedTests -join ', ')"
Write-Output "Known pre-existing (Qt GUI headless 0xc0000409): $($KnownPreExistingFailures -join ', ')"
if ($unexpectedFailures) {
    Write-Output "UNEXPECTED FAILURES (NOT pre-existing): $($unexpectedFailures -join ', ')"
    Write-Output "VERDICT: REGRESSION -- do not report this as a pass."
}
elseif ($failedTests) {
    Write-Output "VERDICT: PASS (only known pre-existing Qt GUI failures present)"
}
else {
    Write-Output "VERDICT: PASS (all tests green)"
}
Write-Output "----"

exit $exitCode
