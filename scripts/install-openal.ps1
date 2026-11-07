$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$bin = Join-Path $root "build\windows\bin"
$installed = Join-Path $bin "OpenAL32.dll"
if (Test-Path -LiteralPath $installed -PathType Leaf) { exit 0 }

$version = "1.25.2"
$temp = Join-Path ([System.IO.Path]::GetTempPath()) ("lunar-openal-" + [Guid]::NewGuid())
New-Item -Path $temp -ItemType Directory | Out-Null
try {
    $archive = Join-Path $temp "openal-soft.zip"
    $url = "https://openal-soft.org/openal-binaries/openal-soft-${version}-bin.zip"
    Invoke-WebRequest -UseBasicParsing -Uri $url -OutFile $archive
    $expanded = Join-Path $temp "openal"
    Expand-Archive -LiteralPath $archive -DestinationPath $expanded
    $libraries = @(Get-ChildItem -LiteralPath $expanded -Recurse -File -Filter "soft_oal.dll" |
        Where-Object { $_.Directory.Name -eq "Win64" })
    if ($libraries.Count -ne 1) { throw "Expected one OpenAL Soft Win64 runtime in the release archive." }
    New-Item -Path $bin -ItemType Directory -Force | Out-Null
    Copy-Item -LiteralPath $libraries[0].FullName -Destination $installed
    Write-Host "OpenAL Soft $version installed beside both Windows executables."
}
finally {
    Remove-Item -LiteralPath $temp -Recurse -Force
}
