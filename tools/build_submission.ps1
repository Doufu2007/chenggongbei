$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
& python (Join-Path $Root "tools/build_submission.py")
if ($LASTEXITCODE -ne 0) { throw "submission build failed" }
