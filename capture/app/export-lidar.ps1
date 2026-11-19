param(
    [Parameter(Mandatory = $true)]
    [string]$TakeDirectory,
    [string]$OutputDirectory,
    [switch]$Merged
)

$ErrorActionPreference = 'Stop'
$projectDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$executable = Join-Path $projectDirectory 'bin\Acquire.exe'
$take = (Resolve-Path -LiteralPath $TakeDirectory).Path
if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) {
    throw "Acquire.exe 不存在，请先运行 .\build.ps1：$executable"
}
if (-not (Test-Path -LiteralPath (Join-Path $take 'lidar') -PathType Container)) {
    throw "take 缺少 lidar 目录：$take"
}

if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $output = Join-Path $take 'lidar_cloud'
} else {
    $output = [System.IO.Path]::GetFullPath($OutputDirectory)
}

$arguments = @('--export-lidar', $take, '--out', $output)
if ($Merged) { $arguments += '--merged' }
$process = Start-Process -FilePath $executable -ArgumentList $arguments -Wait -PassThru -WindowStyle Hidden
if ($process.ExitCode -ne 0) {
    throw "LiDAR 点云导出失败，退出码 $($process.ExitCode)"
}

$sourceCount = @(Get-ChildItem -LiteralPath (Join-Path $take 'lidar') -Filter '*.bin' -File).Count
$plyCount = @(Get-ChildItem -LiteralPath $output -Filter '*.ply' -File).Count
$expected = $sourceCount + [int]$Merged.IsPresent
if ($plyCount -ne $expected -or -not (Test-Path -LiteralPath (Join-Path $output 'index.csv'))) {
    throw "导出产物不完整：应有 $expected 个 PLY，实际 $plyCount 个"
}

Write-Host "LiDAR 点云导出完成：$sourceCount 批，输出 $output"
