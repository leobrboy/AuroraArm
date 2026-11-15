$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$gccOnPath = Get-Command 'arm-none-eabi-gcc.exe' -ErrorAction SilentlyContinue
$qemuOnPath = Get-Command 'qemu-system-arm.exe' -ErrorAction SilentlyContinue
$gcc = if ($env:ARM_NONE_EABI_GCC) { $env:ARM_NONE_EABI_GCC } elseif ($gccOnPath) { $gccOnPath.Source } else { 'C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/14.2 rel1/bin/arm-none-eabi-gcc.exe' }
$qemu = if ($env:QEMU_SYSTEM_ARM) { $env:QEMU_SYSTEM_ARM } elseif ($qemuOnPath) { $qemuOnPath.Source } else { 'C:/Program Files/qemu/qemu-system-arm.exe' }
$elf = Join-Path $PSScriptRoot 'm4_qemu/auroraarm_m4.elf'
$map = Join-Path $PSScriptRoot 'm4_qemu/auroraarm_m4.map'
$sourceStartup = Join-Path $PSScriptRoot 'm4_qemu/startup.c'
$sourceFirmware = Join-Path $PSScriptRoot 'm4_qemu/firmware.c'
$linker = Join-Path $PSScriptRoot 'm4_qemu/mps2_an386.ld'
$controller = Join-Path $root 'generated/auroraarm_controller.c'

foreach ($path in @($gcc, $qemu, $sourceStartup, $sourceFirmware, $linker, $controller, (Join-Path $PSScriptRoot 'm4_vectors.h'))) {
    if (-not (Test-Path -LiteralPath $path)) { throw "Required file/tool not found: $path" }
}

Push-Location $root
try {
    & $gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft `
        -O2 -ffreestanding -ffunction-sections -fdata-sections -nostartfiles `
        "-Wl,-T,$linker" '-Wl,--gc-sections' "-Wl,-Map,$map" `
        $sourceStartup $sourceFirmware $controller -lm -o $elf
    if ($LASTEXITCODE -ne 0) { throw "ARM cross-compilation failed with exit code $LASTEXITCODE" }

    $stdoutPath = Join-Path $PSScriptRoot 'm4_qemu_stdout.txt'
    $stderrPath = Join-Path $PSScriptRoot 'm4_qemu_stderr.txt'
    $qemuArgs = "-M mps2-an386 -cpu cortex-m4 -nographic -semihosting-config enable=on,target=native -kernel `"$elf`""
    $qemuProcess = Start-Process -FilePath $qemu -ArgumentList $qemuArgs -WindowStyle Hidden `
        -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath -PassThru
    if (-not $qemuProcess.WaitForExit(60000)) {
        Stop-Process -Id $qemuProcess.Id -Force
        throw 'Cortex-M4 QEMU test exceeded the 60 s timeout'
    }
    $qemuOutput = @()
    if (Test-Path -LiteralPath $stdoutPath) { $qemuOutput += Get-Content -LiteralPath $stdoutPath }
    if (Test-Path -LiteralPath $stderrPath) { $qemuOutput += Get-Content -LiteralPath $stderrPath }
    $qemuOutput = $qemuOutput | Select-Object -Unique
    $qemuOutput | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'm4_qemu_results.txt')
    $qemuText = $qemuOutput -join "`n"
    $qemuProcess.Refresh()
    $qemuCode = $qemuProcess.ExitCode
    if ($qemuText -notmatch 'M4_QEMU_PASS' -or ($null -ne $qemuCode -and $qemuCode -ne 0)) {
        throw "Cortex-M4 QEMU test failed with exit code $qemuCode"
    }
} finally {
    Pop-Location
}
