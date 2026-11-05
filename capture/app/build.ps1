param(
    [string]$OutputName = 'Acquire.exe'
)

$ErrorActionPreference = 'Stop'

$projectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$outputDir = Join-Path $projectDir 'bin'
$compiler = 'C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe'

if (-not (Test-Path -LiteralPath $compiler)) {
    throw "C# compiler not found: $compiler"
}

New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

$sources = Get-ChildItem -LiteralPath (Join-Path $projectDir 'src') -Recurse -Filter '*.cs' |
    Sort-Object FullName |
    ForEach-Object { $_.FullName }

$outputFile = Join-Path $outputDir $OutputName
$compilerArgs = @(
    '/nologo'
    '/target:winexe'
    '/optimize+'
    '/platform:x64'
    "/out:$outputFile"
    '/reference:System.dll'
    '/reference:System.Core.dll'
    '/reference:System.Drawing.dll'
    '/reference:System.Windows.Forms.dll'
    '/reference:System.Management.dll'
) + $sources

& $compiler $compilerArgs

if ($LASTEXITCODE -ne 0) {
    throw "Build failed with exit code $LASTEXITCODE"
}

Write-Host "Built: $outputFile"
