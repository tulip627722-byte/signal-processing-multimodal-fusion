param(
    [string]$Source = 'E:\奥比中光Astra Pro\奥比中光Astra Pro (1)\奥比中光Astra Pro\上位机软件',
    [switch]$VerifyOnly
)

$ErrorActionPreference = 'Stop'
$repositoryRoot = Split-Path -Parent $PSScriptRoot
$destination = Join-Path $repositoryRoot 'vendor\orbbec-viewer'
$manifestPath = Join-Path $repositoryRoot 'capture\docs\orbbec-vendor-manifest.csv'
$expected = Import-Csv -LiteralPath $manifestPath

function Get-VendorInventory([string]$Root) {
    Get-ChildItem -LiteralPath $Root -Recurse -File | Sort-Object FullName | ForEach-Object {
        [PSCustomObject]@{
            RelativePath = $_.FullName.Substring($Root.Length).TrimStart('\')
            Length = [string]$_.Length
            SHA256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
        }
    }
}

if (-not $VerifyOnly) {
    if (-not (Test-Path -LiteralPath $Source)) { throw "厂家源目录不存在：$Source" }
    if (Test-Path -LiteralPath $destination) { throw "目标已存在，为避免覆盖请先人工确认：$destination" }
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $destination) | Out-Null
    Copy-Item -LiteralPath $Source -Destination $destination -Recurse
    $nestedViewer = Join-Path $destination '上位机软件'
    if (Test-Path -LiteralPath $nestedViewer) {
        Get-ChildItem -LiteralPath $nestedViewer -Force | ForEach-Object {
            $flattenedPath = Join-Path $destination $_.Name
            if (Test-Path -LiteralPath $flattenedPath) {
                throw "厂家副本展平失败，目标已存在：$flattenedPath"
            }
            Move-Item -LiteralPath $_.FullName -Destination $flattenedPath
        }
        Remove-Item -LiteralPath $nestedViewer
    }
}

if (-not (Test-Path -LiteralPath $destination)) { throw "厂家副本不存在：$destination" }
$actual = Get-VendorInventory $destination
$difference = Compare-Object $expected $actual -Property RelativePath, Length, SHA256
if ($difference) {
    $difference | Format-Table -AutoSize
    throw '厂家副本与锁定清单不一致。'
}

Write-Host "厂家副本校验通过：$($actual.Count) 个文件，$((($actual | Measure-Object Length -Sum).Sum)) 字节。"
