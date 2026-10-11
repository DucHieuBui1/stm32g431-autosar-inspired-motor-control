$ErrorActionPreference = 'Stop'

$scriptRoot = (Resolve-Path -LiteralPath $PSScriptRoot).Path
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $scriptRoot '..\..\..')).Path
$compiler = 'C:\MinGW\bin\gcc.exe'

if (-not (Test-Path -LiteralPath $compiler -PathType Leaf)) {
    throw "Pinned host compiler is missing: $compiler"
}

$compilerVersion = & $compiler --version
if (($LASTEXITCODE -ne 0) -or
    ($compilerVersion[0] -ne 'gcc.exe (MinGW.org GCC-6.3.0-1) 6.3.0')) {
    throw "Unexpected pinned host compiler identity: $($compilerVersion[0])"
}

$tempRoot = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath())
$buildLeaf = 'maev-os-host-' + [System.Guid]::NewGuid().ToString('N')
$buildRoot = [System.IO.Path]::GetFullPath((Join-Path $tempRoot $buildLeaf))
$tempPrefix = $tempRoot.TrimEnd('\') + '\'

if ((-not $buildRoot.StartsWith($tempPrefix,
                                [System.StringComparison]::OrdinalIgnoreCase)) -or
    ((Split-Path -Leaf $buildRoot) -ne $buildLeaf)) {
    throw "Refusing unvalidated host-test build path: $buildRoot"
}

$executable = Join-Path $buildRoot 'test_os_step7a.exe'
$configExecutable = Join-Path $buildRoot 'test_os_cfg_step7a.exe'
$arguments = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O0',
    '-I', (Join-Path $scriptRoot 'fake'),
    '-I', (Join-Path $projectRoot 'Software\Bsw\Common\Inc'),
    '-I', (Join-Path $projectRoot 'Software\Bsw\Services\Os\Inc'),
    (Join-Path $scriptRoot 'test_os_step7a.c'),
    '-o', $executable
)
$configArguments = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O0',
    '-I', (Join-Path $projectRoot 'Software\Bsw\Common\Inc'),
    '-I', (Join-Path $projectRoot 'Software\Bsw\Services\Os\Inc'),
    '-I', (Join-Path $projectRoot 'Software\Bsw\Services\EcuM\Inc'),
    '-I', (Join-Path $projectRoot 'Software\Bsw\Services\SafetySupervisor\Inc'),
    '-I', (Join-Path $projectRoot 'Software\Bsw\Services\StateManager\Inc'),
    (Join-Path $scriptRoot 'test_os_cfg_step7a.c'),
    '-o', $configExecutable
)

try {
    New-Item -ItemType Directory -Path $buildRoot | Out-Null
    & $compiler @arguments
    if ($LASTEXITCODE -ne 0) {
        throw 'OS host-test compilation failed.'
    }

    & $executable
    if ($LASTEXITCODE -ne 0) {
        throw 'OS host-test execution failed.'
    }

    & $compiler @configArguments
    if ($LASTEXITCODE -ne 0) {
        throw 'OS configuration host-test compilation failed.'
    }

    & $configExecutable
    if ($LASTEXITCODE -ne 0) {
        throw 'OS configuration host-test execution failed.'
    }

    Write-Output 'PASS: pinned Step-7A OS host matrix'
}
finally {
    if (Test-Path -LiteralPath $buildRoot -PathType Container) {
        $resolvedCleanup = (Resolve-Path -LiteralPath $buildRoot).Path
        if ($resolvedCleanup.StartsWith($tempPrefix,
                                        [System.StringComparison]::OrdinalIgnoreCase) -and
            ((Split-Path -Leaf $resolvedCleanup) -eq $buildLeaf)) {
            Remove-Item -LiteralPath $resolvedCleanup -Recurse -Force
        }
        else {
            throw "Refusing unvalidated host-test cleanup path: $resolvedCleanup"
        }
    }
}
