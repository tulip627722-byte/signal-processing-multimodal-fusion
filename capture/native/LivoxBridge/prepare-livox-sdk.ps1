param(
    [switch]$Build
)

$ErrorActionPreference = 'Stop'
$bridgeDir = $PSScriptRoot
$repositoryRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $bridgeDir))
$dependencyRoot = Join-Path $repositoryRoot '.deps'
$sdkRoot = Join-Path $dependencyRoot 'Livox-SDK-2.3.1'
$archive = Join-Path $dependencyRoot 'Livox-SDK-v2.3.1.zip'
$expectedCommit = '14c533dd7175bd90a6b568c0aa1733f35d36cb89'
$downloadUrl = 'https://github.com/Livox-SDK/Livox-SDK/archive/refs/tags/v2.3.1.zip'

New-Item -ItemType Directory -Force -Path $dependencyRoot | Out-Null
if (-not (Test-Path -LiteralPath $sdkRoot)) {
    if (-not (Test-Path -LiteralPath $archive)) {
        $temporaryArchive = $archive + '.part'
        if (Test-Path -LiteralPath $temporaryArchive) { Remove-Item -LiteralPath $temporaryArchive -Force }
        & curl.exe -L --retry 4 --retry-all-errors --connect-timeout 20 --max-time 300 -o $temporaryArchive $downloadUrl
        if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $temporaryArchive)) {
            throw "Livox-SDK v2.3.1 下载失败：$downloadUrl"
        }
        Move-Item -LiteralPath $temporaryArchive -Destination $archive
    }
    Expand-Archive -LiteralPath $archive -DestinationPath $dependencyRoot -Force
    $expanded = Join-Path $dependencyRoot 'Livox-SDK-2.3.1'
    if (-not (Test-Path -LiteralPath $expanded)) { throw "Livox-SDK v2.3.1 解压失败。" }
}

Set-Content -LiteralPath (Join-Path $dependencyRoot 'Livox-SDK-version.txt') -Encoding ASCII -Value @(
    'tag=v2.3.1'
    "commit=$expectedCommit"
    "source=$downloadUrl"
)

Write-Host "Livox-SDK v2.3.1 prepared at $sdkRoot"
if ($Build) {
    $cmake = 'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
    if (-not (Test-Path -LiteralPath $cmake)) { throw "CMake not found: $cmake" }
    $buildDir = Join-Path $bridgeDir 'build'
    & $cmake -S $bridgeDir -B $buildDir -A x64 "-DLIVOX_SDK_ROOT=$sdkRoot"
    if ($LASTEXITCODE -ne 0) { throw 'LivoxBridge configure failed.' }
    & $cmake --build $buildDir --config Release
    if ($LASTEXITCODE -ne 0) { throw 'LivoxBridge build failed.' }
    $dll = Join-Path $buildDir 'Release\LivoxBridge.dll'
    if (-not (Test-Path -LiteralPath $dll)) { throw "LivoxBridge.dll not found: $dll" }
    Copy-Item -LiteralPath $dll -Destination (Join-Path $repositoryRoot 'capture\app\bin\LivoxBridge.dll') -Force
    Write-Host 'LivoxBridge.dll deployed to capture\app\bin.'
}
