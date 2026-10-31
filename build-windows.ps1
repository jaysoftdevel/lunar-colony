param(
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$Configuration = "Release",
    [switch]$Tests,
    [switch]$Clean
)
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$argsList = @($Configuration)
if ($Tests) { $argsList += "--tests" }
if ($Clean) { $argsList += "--clean" }
& (Join-Path $Root "build.bat") @argsList
exit $LASTEXITCODE
