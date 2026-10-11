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
$buildLeaf = 'maev-motor-fast-loop-host-' +
    [System.Guid]::NewGuid().ToString('N')
$buildRoot = [System.IO.Path]::GetFullPath((Join-Path $tempRoot $buildLeaf))
$tempPrefix = $tempRoot.TrimEnd('\') + '\'

if ((-not $buildRoot.StartsWith(
        $tempPrefix, [System.StringComparison]::OrdinalIgnoreCase)) -or
    ((Split-Path -Leaf $buildRoot) -ne $buildLeaf)) {
    throw "Refusing unvalidated host-test build path: $buildRoot"
}

$integrationExecutable = Join-Path $buildRoot 'test_motor_control_fast_loop.exe'
$integrationArguments = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O2',
    '-I', (Join-Path $projectRoot 'Software\Bsw\Common\Inc'),
    '-I', (Join-Path $projectRoot 'Software\Bsw\ComplexDrivers\MotorControl\Inc'),
    (Join-Path $scriptRoot 'test_motor_control_fast_loop.c'),
    '-lm', '-o', $integrationExecutable
)
$injectionExecutable = Join-Path $buildRoot 'test_motor_control_fast_loop_injection.exe'
$injectionArguments = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O2',
    '-I', (Join-Path $projectRoot 'Software\Bsw\Common\Inc'),
    '-I', (Join-Path $projectRoot 'Software\Bsw\ComplexDrivers\MotorControl\Inc'),
    (Join-Path $scriptRoot 'test_motor_control_fast_loop_injection.c'),
    '-o', $injectionExecutable
)

try {
    New-Item -ItemType Directory -Path $buildRoot | Out-Null
    & $compiler @integrationArguments
    if ($LASTEXITCODE -ne 0) {
        throw 'MotorControl FastLoop integration host-test compilation failed.'
    }

    & $integrationExecutable
    if ($LASTEXITCODE -ne 0) {
        throw 'MotorControl FastLoop integration host-test execution failed.'
    }

    & $compiler @injectionArguments
    if ($LASTEXITCODE -ne 0) {
        throw 'MotorControl FastLoop injection host-test compilation failed.'
    }

    & $injectionExecutable
    if ($LASTEXITCODE -ne 0) {
        throw 'MotorControl FastLoop injection host-test execution failed.'
    }

    Write-Output 'PASS: pinned MotorControl FastLoop host matrix'
}
finally {
    if (Test-Path -LiteralPath $buildRoot -PathType Container) {
        $resolvedCleanup = (Resolve-Path -LiteralPath $buildRoot).Path
        if ($resolvedCleanup.StartsWith(
                $tempPrefix,
                [System.StringComparison]::OrdinalIgnoreCase) -and
            ((Split-Path -Leaf $resolvedCleanup) -eq $buildLeaf)) {
            Remove-Item -LiteralPath $resolvedCleanup -Recurse -Force
        }
        else {
            throw "Refusing unvalidated host-test cleanup path: $resolvedCleanup"
        }
    }
}
