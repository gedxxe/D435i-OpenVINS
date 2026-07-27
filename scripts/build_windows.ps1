param([ValidateSet("Debug", "Release")][string]$Configuration = "Debug")
$ErrorActionPreference = "Stop"
$RepoDir = Split-Path -Parent $PSScriptRoot
Push-Location $RepoDir
try {
    cmake --preset windows-msvc
    cmake --build (Join-Path $RepoDir "build/windows-msvc") --config $Configuration
    ctest --test-dir (Join-Path $RepoDir "build/windows-msvc") `
        -C $Configuration --output-on-failure --no-tests=error
}
finally {
    Pop-Location
}
