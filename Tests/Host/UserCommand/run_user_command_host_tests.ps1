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
$buildLeaf = 'maev-user-command-host-' +
    [System.Guid]::NewGuid().ToString('N')
$buildRoot = [System.IO.Path]::GetFullPath((Join-Path $tempRoot $buildLeaf))
$tempPrefix = $tempRoot.TrimEnd('\') + '\'

if ((-not $buildRoot.StartsWith(
        $tempPrefix, [System.StringComparison]::OrdinalIgnoreCase)) -or
    ((Split-Path -Leaf $buildRoot) -ne $buildLeaf)) {
    throw "Refusing unvalidated host-test build path: $buildRoot"
}

$commonInclude = Join-Path $projectRoot 'Software\Bsw\Common\Inc'
$moduleInclude = Join-Path $projectRoot 'Software\Asw\UserCommand\Inc'
$productionSource = Join-Path $projectRoot 'Software\Asw\UserCommand\Src\User_Command.c'
$executable = Join-Path $buildRoot 'test_user_command.exe'
$corruptionExecutable = Join-Path $buildRoot 'test_user_command_corruption.exe'
$compileArguments = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O0',
    '-I', $commonInclude,
    '-I', $moduleInclude,
    (Join-Path $scriptRoot 'test_user_command.c'),
    '-o', $executable
)
$corruptionCompileArguments = @(
    '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O0',
    '-I', $commonInclude,
    '-I', $moduleInclude,
    (Join-Path $scriptRoot 'test_user_command_corruption.c'),
    '-o', $corruptionExecutable
)

function Write-ProbeSource {
    param(
        [Parameter(Mandatory)][string]$Path,
        [Parameter(Mandatory)][string]$Text)

    $utf8NoBom = [System.Text.UTF8Encoding]::new($false)
    [System.IO.File]::WriteAllText($Path, $Text, $utf8NoBom)
}

function Invoke-CompileProbe {
    param(
        [Parameter(Mandatory)][string]$Name,
        [Parameter(Mandatory)][string]$SourceText,
        [Parameter(Mandatory)][bool]$ExpectSuccess)

    $probeSource = Join-Path $buildRoot ($Name + '.c')
    $probeObject = Join-Path $buildRoot ($Name + '.o')
    Write-ProbeSource -Path $probeSource -Text $SourceText
    & $compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-pedantic' '-O0' `
        '-I' $commonInclude '-I' $moduleInclude '-c' $probeSource `
        '-o' $probeObject 2>$null
    $succeeded = ($LASTEXITCODE -eq 0)
    if ($succeeded -ne $ExpectSuccess) {
        throw "Unexpected compile result for UserCommand probe: $Name"
    }
}

function New-ProfileSource {
    param(
        [Parameter(Mandatory)][string]$SourceText,
        [Parameter(Mandatory)][string]$Enabled,
        [Parameter(Mandatory)][string]$Current,
        [Parameter(Mandatory)][string]$Duration,
        [Parameter(Mandatory)][string]$Lease)

    return $SourceText.Replace(
        '#define USER_COMMAND_ENABLED          0U',
        "#define USER_COMMAND_ENABLED          $Enabled").Replace(
        '#define USER_COMMAND_CURRENT_MA       0',
        "#define USER_COMMAND_CURRENT_MA       $Current").Replace(
        '#define USER_COMMAND_RUN_DURATION_MS  0U',
        "#define USER_COMMAND_RUN_DURATION_MS  $Duration").Replace(
        '#define USER_COMMAND_LEASE_MS         0U',
        "#define USER_COMMAND_LEASE_MS         $Lease")
}

try {
    New-Item -ItemType Directory -Path $buildRoot | Out-Null
    & $compiler @compileArguments
    if ($LASTEXITCODE -ne 0) {
        throw 'UserCommand host-test compilation failed.'
    }

    & $executable
    if ($LASTEXITCODE -ne 0) {
        throw 'UserCommand host-test execution failed.'
    }

    & $compiler @corruptionCompileArguments
    if ($LASTEXITCODE -ne 0) {
        throw 'UserCommand corrupt-profile host-test compilation failed.'
    }

    & $corruptionExecutable
    if ($LASTEXITCODE -ne 0) {
        throw 'UserCommand corrupt-profile host-test execution failed.'
    }

    $sourceText = Get-Content -Raw -LiteralPath $productionSource
    $enabledText = New-ProfileSource -SourceText $sourceText `
        -Enabled '1U' -Current '300' -Duration '5000U' -Lease '100U'
    Invoke-CompileProbe -Name 'valid_enabled_profile' `
        -SourceText $enabledText -ExpectSuccess $true

    $invalidProfiles = [ordered]@{
        invalid_enabled = @('2U', '0', '0U', '0U')
        disabled_latent_current = @('0U', '1', '0U', '0U')
        invalid_current_low = @('1U', '-301', '5000U', '100U')
        invalid_current_high = @('1U', '301', '5000U', '100U')
        invalid_duration_low = @('1U', '300', '0U', '100U')
        invalid_duration_high = @('1U', '300', '5001U', '100U')
        invalid_lease_low = @('1U', '300', '5000U', '0U')
        invalid_lease_high = @('1U', '300', '5000U', '101U')
    }
    foreach ($probe in $invalidProfiles.GetEnumerator()) {
        $probeText = New-ProfileSource -SourceText $sourceText `
            -Enabled $probe.Value[0] -Current $probe.Value[1] `
            -Duration $probe.Value[2] -Lease $probe.Value[3]
        Invoke-CompileProbe -Name $probe.Key -SourceText $probeText `
            -ExpectSuccess $false
    }

    Write-Output 'PASS: pinned UserCommand SC-0 host matrix'
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
