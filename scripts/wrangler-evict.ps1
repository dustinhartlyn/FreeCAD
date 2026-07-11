<#
.SYNOPSIS
  OS handle eviction for The Wrangler mode. Logs the destructive action to
  .wrangler_runbook.md BEFORE executing it, per .roomodes rule 8 -- so the
  action is recorded even if the eviction itself fails or has side effects
  the model didn't anticipate (e.g. killing a FreeCAD session with unsaved
  user work).

.PARAMETER ProcessName
  Defaults to FreeCAD. Only known build/runtime lock-holders should be passed
  here -- this script intentionally does not accept arbitrary process names
  without the caller stating why.

.PARAMETER Reason
  Required. Free-text reason this eviction is being run (e.g. the exact
  compiler error that triggered it). Logged verbatim.
#>
param(
    [string]$ProcessName = "FreeCAD",
    [Parameter(Mandatory = $true)][string]$Reason
)

$timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
$logLine = "- [$timestamp] OS HANDLE EVICTION: Stop-Process -Name $ProcessName -Force. Reason: $Reason"

Add-Content -Path "Agent_Notes/.wrangler_runbook.md" -Value $logLine -Encoding utf8

Write-Output "Logged eviction to Agent_Notes/.wrangler_runbook.md. Executing Stop-Process -Name $ProcessName -Force ..."
Stop-Process -Name $ProcessName -Force -ErrorAction SilentlyContinue
Write-Output "Done."
