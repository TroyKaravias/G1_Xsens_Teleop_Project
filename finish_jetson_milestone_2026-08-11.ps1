$ErrorActionPreference = 'Stop'

$Jetson = 'unitree@192.168.123.164'
$CollectorName = 'collect_jetson_milestone_2026-08-11.sh'
$ArchiveName = 'G1_Jetson_State_Milestone_2026-08-11.tar.gz'
$ChecksumName = "$ArchiveName.sha256"
$CollectorPath = Join-Path $PSScriptRoot $CollectorName
$ArchivePath = Join-Path $PSScriptRoot $ArchiveName
$ChecksumPath = Join-Path $PSScriptRoot $ChecksumName

if (-not (Test-Path -LiteralPath $CollectorPath)) {
    throw "Collector script not found: $CollectorPath"
}
if ((Test-Path -LiteralPath $ArchivePath) -or (Test-Path -LiteralPath $ChecksumPath)) {
    throw "A local Jetson milestone archive already exists. Nothing was overwritten."
}

Write-Host 'Step 1/4: Uploading the read-only collector (SSH may request your password)...'
& scp $CollectorPath "${Jetson}:/home/unitree/$CollectorName"
if ($LASTEXITCODE -ne 0) { throw 'Collector upload failed.' }

Write-Host 'Step 2/4: Creating the Jetson archive.'
Write-Host 'Review the displayed sizes, then type BACKUP when prompted.'
& ssh -t $Jetson "bash /home/unitree/$CollectorName"
if ($LASTEXITCODE -ne 0) { throw 'Jetson archive creation failed or was cancelled.' }

Write-Host 'Step 3/4: Downloading the archive and its checksum...'
& scp "${Jetson}:/home/unitree/$ArchiveName" $PSScriptRoot
if ($LASTEXITCODE -ne 0) { throw 'Archive download failed.' }
& scp "${Jetson}:/home/unitree/$ChecksumName" $PSScriptRoot
if ($LASTEXITCODE -ne 0) { throw 'Checksum download failed.' }

Write-Host 'Step 4/4: Verifying SHA-256 integrity...'
$Expected = ((Get-Content -LiteralPath $ChecksumPath -Raw).Trim() -split '\s+')[0].ToLowerInvariant()
$Actual = (Get-FileHash -LiteralPath $ArchivePath -Algorithm SHA256).Hash.ToLowerInvariant()
if ($Expected -ne $Actual) {
    throw "Checksum mismatch. Expected $Expected but received $Actual"
}

Write-Host ''
Write-Host 'PASS: complete Jetson software-state milestone downloaded and verified.' -ForegroundColor Green
Get-Item -LiteralPath $ArchivePath, $ChecksumPath | Select-Object FullName, Length, LastWriteTime
Write-Host ''
Write-Host 'The remote archive was intentionally left in /home/unitree as a second copy.'
