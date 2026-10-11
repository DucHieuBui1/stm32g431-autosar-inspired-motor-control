$ErrorActionPreference = 'Stop'

$scriptRoot = (Resolve-Path -LiteralPath $PSScriptRoot).Path
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $scriptRoot '..\..\..')).Path
$compiler = 'C:\MinGW\bin\gcc.exe'

if (-not (Test-Path -LiteralPath $compiler -PathType Leaf)) {
    throw "Pinned host compiler is missing: $compiler"
}

$compilerVersion = & $compiler --version
if ($LASTEXITCODE -ne 0) {
    throw 'Unable to query the pinned host compiler.'
}

$expectedVersion = 'gcc.exe (MinGW.org GCC-6.3.0-1) 6.3.0'
if (($compilerVersion.Count -eq 0) -or
    ($compilerVersion[0] -ne $expectedVersion)) {
    throw "Unexpected host compiler identity: $($compilerVersion[0])"
}

$tempRoot = [System.IO.Path]::GetFullPath([System.IO.Path]::GetTempPath())
$buildLeaf = 'maev-adc-host-' + [System.Guid]::NewGuid().ToString('N')
$buildRoot = [System.IO.Path]::GetFullPath((Join-Path $tempRoot $buildLeaf))

$tempPrefix = $tempRoot.TrimEnd('\') + '\'
if ((-not $buildRoot.StartsWith($tempPrefix,
                                [System.StringComparison]::OrdinalIgnoreCase)) -or
    ((Split-Path -Leaf $buildRoot) -ne $buildLeaf)) {
    throw "Refusing unvalidated host-test build path: $buildRoot"
}

$commonFlags = @(
    '-std=c99',
    '-Wall',
    '-Wextra',
    '-Werror',
    '-pedantic',
    '-O0'
)

$commonInclude = Join-Path $projectRoot 'Software\Bsw\Common\Inc'
$adcInclude = Join-Path $projectRoot 'Software\Bsw\Mcal\Adc\Inc'
$mcuInclude = Join-Path $projectRoot 'Software\Bsw\Mcal\Mcu\Inc'
$wdgInclude = Join-Path $projectRoot 'Software\Bsw\Mcal\Wdg\Inc'
$ecumInclude = Join-Path $projectRoot 'Software\Bsw\Services\EcuM\Inc'
$osInclude = Join-Path $projectRoot 'Software\Bsw\Services\Os\Inc'
$safetySupervisorInclude = Join-Path $projectRoot 'Software\Bsw\Services\SafetySupervisor\Inc'
$stateManagerInclude = Join-Path $projectRoot 'Software\Bsw\Services\StateManager\Inc'
$wdgMInclude = Join-Path $projectRoot 'Software\Bsw\Services\WdgM\Inc'
$motorInclude = Join-Path $projectRoot 'Software\Bsw\ComplexDrivers\MotorControl\Inc'
$serialInclude = Join-Path $projectRoot 'Software\Bsw\ComplexDrivers\SerialCommunication\Inc'
$fakeInclude = Join-Path $scriptRoot 'fake'

$adcExecutable = Join-Path $buildRoot 'test_adc.exe'
$ecumExecutable = Join-Path $buildRoot 'test_ecum_adc_integration.exe'

try {
    New-Item -ItemType Directory -Path $buildRoot | Out-Null

    $adcArguments = $commonFlags + @(
        '-I', $fakeInclude,
        '-I', $adcInclude,
        '-I', $mcuInclude,
        '-I', $commonInclude,
        (Join-Path $scriptRoot 'test_adc.c'),
        '-o', $adcExecutable
    )
    & $compiler @adcArguments
    if ($LASTEXITCODE -ne 0) {
        throw 'ADC/Mcu host-test compilation failed.'
    }

    & $adcExecutable
    if ($LASTEXITCODE -ne 0) {
        throw 'ADC/Mcu host-test execution failed.'
    }

    $ecumArguments = $commonFlags + @(
        '-I', $adcInclude,
        '-I', $mcuInclude,
        '-I', $wdgInclude,
        '-I', $ecumInclude,
        '-I', $osInclude,
        '-I', $safetySupervisorInclude,
        '-I', $stateManagerInclude,
        '-I', $wdgMInclude,
        '-I', $motorInclude,
        '-I', $serialInclude,
        '-I', $commonInclude,
        (Join-Path $scriptRoot 'test_ecum_adc_integration.c'),
        '-o', $ecumExecutable
    )
    & $compiler @ecumArguments
    if ($LASTEXITCODE -ne 0) {
        throw 'EcuM/ADC integration host-test compilation failed.'
    }

    & $ecumExecutable
    if ($LASTEXITCODE -ne 0) {
        throw 'EcuM/ADC integration host-test execution failed.'
    }

    Write-Output "PASS: pinned host compiler $expectedVersion"
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
