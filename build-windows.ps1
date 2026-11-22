param(
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$Configuration = "Release",
    [string]$EnginePath = "",
    [switch]$Tests,
    [switch]$RenderTests,
    [switch]$Clean,
    [int]$Jobs = 0
)
$ErrorActionPreference = 'Stop'
$argsList = @("build", $Configuration)
if ($EnginePath) { $argsList += @("--engine", $EnginePath) }
if ($Tests) { $argsList += "--tests" }
if ($RenderTests) { $argsList += "--render-tests" }
if ($Clean) { $argsList += "--clean" }
if ($Jobs) { $argsList += @("--jobs", "$Jobs") }
$Driver = Join-Path $PSScriptRoot "scripts/windows-build.py"
if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 $Driver @argsList
} else {
    & python $Driver @argsList
}
exit $LASTEXITCODE
