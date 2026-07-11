<#
.SYNOPSIS
  Appends a structured entry to the "## Test & Benchmark Results Log" section
  of .gcs_status.md, per .roomodes Wrangler rule 6a / Critic rule 13.
  Guarantees the required fields are present instead of relying on the model
  to hand-write correct markdown every time.

.PARAMETER Title
  Short title, e.g. "Stage 4: Cluster Decomposition Differential Test (shared-point case)"

.PARAMETER Command
  The exact command that was run.

.PARAMETER Hypothesis
  The SPECIFIC, narrow claim being tested. If the test only covers a subset of
  the original hypothesis (e.g. decoupled clusters but not shared-point
  clusters), that narrowing must be stated here explicitly -- this field exists
  specifically so a narrowed-scope result cannot be mistaken for a general one.

.PARAMETER Expected
  What the plan predicted BEFORE the actual result was seen.

.PARAMETER Actual
  The actual output, verbatim -- pass/fail counts, numbers, assertion failures.

.PARAMETER Verdict
  Must be one of: CONFIRMED, FALSIFIED, INCONCLUSIVE. No other value accepted --
  this is intentionally not free text, to stop "PASSED (mostly)" from sliding in.
#>
param(
    [Parameter(Mandatory = $true)][string]$Title,
    [Parameter(Mandatory = $true)][string]$Command,
    [Parameter(Mandatory = $true)][string]$Hypothesis,
    [Parameter(Mandatory = $true)][string]$Expected,
    [Parameter(Mandatory = $true)][string]$Actual,
    [Parameter(Mandatory = $true)][ValidateSet("CONFIRMED", "FALSIFIED", "INCONCLUSIVE")][string]$Verdict
)

$date = Get-Date -Format "yyyy-MM-dd HH:mm"

$entry = @"

### $Title ($date)

**Command**: ``$Command``

**Hypothesis** (state the exact, narrow claim -- not a generalization of it): $Hypothesis

**Expected** (stated before the result was seen): $Expected

**Actual** (verbatim): $Actual

**Verdict**: $Verdict
"@

if (-not (Select-String -Path "Agent_Notes/.gcs_status.md" -Pattern "## .*Test & Benchmark Results Log" -Quiet)) {
    Add-Content -Path "Agent_Notes/.gcs_status.md" -Value "`n## Test & Benchmark Results Log`n" -Encoding utf8
}

Add-Content -Path "Agent_Notes/.gcs_status.md" -Value $entry -Encoding utf8
Write-Output "Logged to Agent_Notes/.gcs_status.md: $Title -> $Verdict"
