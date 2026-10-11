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
$buildLeaf = 'maev-serial-host-' + [System.Guid]::NewGuid().ToString('N')
$buildRoot = [System.IO.Path]::GetFullPath((Join-Path $tempRoot $buildLeaf))
$tempPrefix = $tempRoot.TrimEnd('\') + '\'

if ((-not $buildRoot.StartsWith($tempPrefix,
                                [System.StringComparison]::OrdinalIgnoreCase)) -or
    ((Split-Path -Leaf $buildRoot) -ne $buildLeaf)) {
    throw "Refusing unvalidated host-test build path: $buildRoot"
}

$executable = Join-Path $buildRoot 'test_serial_communication_cdd.exe'
$mcuExecutable = Join-Path $buildRoot 'test_mcu_communication.exe'
$arguments = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O0',
    '-I', (Join-Path $scriptRoot 'fake'),
    '-I', (Join-Path $projectRoot 'Software\Bsw\Common\Inc'),
    '-I', (Join-Path $projectRoot 'Software\Bsw\ComplexDrivers\SerialCommunication\Inc'),
    (Join-Path $scriptRoot 'test_serial_communication_cdd.c'),
    '-o', $executable
)

try {
    New-Item -ItemType Directory -Path $buildRoot | Out-Null
    & $compiler @arguments
    if ($LASTEXITCODE -ne 0) {
        throw 'SerialCommunication CDD host-test compilation failed.'
    }

    & $executable
    if ($LASTEXITCODE -ne 0) {
        throw 'SerialCommunication CDD host-test execution failed.'
    }

    $mcuArguments = @(
        '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O0',
        '-I', (Join-Path $scriptRoot 'fake_mcu'),
        '-I', (Join-Path $projectRoot 'Software\Bsw\Common\Inc'),
        '-I', (Join-Path $projectRoot 'Software\Bsw\Mcal\Mcu\Inc'),
        (Join-Path $scriptRoot 'test_mcu_communication.c'),
        '-o', $mcuExecutable
    )
    & $compiler @mcuArguments
    if ($LASTEXITCODE -ne 0) {
        throw 'Mcu communication host-test compilation failed.'
    }

    & $mcuExecutable
    if ($LASTEXITCODE -ne 0) {
        throw 'Mcu communication host-test execution failed.'
    }

    $productionSource = Join-Path $projectRoot 'Software\Bsw\ComplexDrivers\SerialCommunication\Src\SerialCommunicationCdd.c'
    $prohibited = Select-String -CaseSensitive -LiteralPath $productionSource -Pattern 'MotorControlCdd_|StateManager_|\bOs_|\bStartOS\b|\bWaitEvent\b|\bSetEvent\b'
    if ($null -ne $prohibited) {
        throw 'SerialCommunication CDD contains a prohibited direct service/actuator reference.'
    }

    Write-Output 'PASS: pinned SerialCommunication/Mcu communication host matrix'
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
