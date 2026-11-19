param(
    [string]$OutputDirectory = (Join-Path $PSScriptRoot 'bin')
)

$ErrorActionPreference = 'Stop'
$repositoryRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$vendorRuntime = Join-Path $repositoryRoot 'vendor\orbbec-viewer'
$requiredFiles = @{
    'OpenNI2.dll' = '8F76A691C3163B9088790A88CBA50F6BE95CA607DD2E28F5B83D7896F121DA3E'
    'OpenNI.ini' = '3E177E1852CC861CC005C5E6E48037F04D85C0AF6BD4988594F6CBB69775403C'
    'OpenNI2\Drivers\OniFile.dll' = '3680CA5BB42F1172404B61E14181E3EE79F3FE41CDB6DFC653C08AAAEE40246F'
    'OpenNI2\Drivers\OniFile.ini' = '768EC608FC73A058E146761283EED863B520D88DF120039A7375B77090D646B3'
    'OpenNI2\Drivers\orbbec.dll' = '11965195FAE5C07D03919BAF70A9B896B6311F5295F345209FBD19F388E0AE1F'
    'OpenNI2\Drivers\orbbec.ini' = 'C2345FC609C01CC6AEF3EE63A1D609490387F554576DBE421599C73B5A58F295'
}

if (-not (Test-Path -LiteralPath $vendorRuntime)) {
    throw "Project vendor runtime is missing: $vendorRuntime. Run tools\sync-orbbec-vendor.ps1."
}

foreach ($relativePath in $requiredFiles.Keys) {
    $source = Join-Path $vendorRuntime $relativePath
    if (-not (Test-Path -LiteralPath $source)) {
        throw "Project vendor runtime file is missing: $relativePath"
    }
    $actualHash = (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash
    if ($actualHash -ne $requiredFiles[$relativePath]) {
        throw "Project vendor runtime hash mismatch: $relativePath; expected $($requiredFiles[$relativePath]); actual $actualHash"
    }
}

New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
foreach ($relativePath in $requiredFiles.Keys) {
    $source = Join-Path $vendorRuntime $relativePath
    $destination = Join-Path $OutputDirectory $relativePath
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $destination) | Out-Null
    if (Test-Path -LiteralPath $destination) {
        $destinationHash = (Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash
        if ($destinationHash -eq $requiredFiles[$relativePath]) { continue }
    }
    Copy-Item -LiteralPath $source -Destination $destination -Force
}

Write-Host "Verified and deployed Orbbec runtime from project vendor copy."
