<#
  sim.ps1 - compile + run an xsim testbench for the Repo layout (rtl/ + tb/).
  Run from anywhere; it builds ../rtl/<Files> + ./<Tb> and runs from tb/ so the
  $readmemh vectors (blk_*.hex) resolve.

  Examples:
    .\sim.ps1 -Top tb_lzo -Files "lzo_comp.sv,lzo_decomp.sv" -Tb "tb_lzo.sv"
    .\sim.ps1 -Top tb_top -Files "lzo_comp.sv,lzo_decomp.sv,lzo_top.sv" -Tb "tb_top.sv"
#>
[CmdletBinding()]
param(
  [Parameter(Mandatory=$true)][string]$Top,
  [string]$Files = "",
  [string]$Tb = ""
)
$ErrorActionPreference = 'Stop'
$BIN  = "C:\Xilinx\2025.1\Vivado\bin"
$ROOT = Split-Path $PSScriptRoot -Parent   # Repo/
$RTL  = Join-Path $ROOT "rtl"
$TBDIR = $PSScriptRoot                       # Repo/tb

$src = @()
foreach ($f in ($Files -split ',')) { if ($f.Trim()) { $src += (Join-Path $RTL   $f.Trim()) } }
foreach ($f in ($Tb    -split ',')) { if ($f.Trim()) { $src += (Join-Path $TBDIR $f.Trim()) } }

Push-Location $TBDIR
try {
  Write-Host "==== xvlog ====" -ForegroundColor Cyan
  & "$BIN\xvlog.bat" --sv $src
  if ($LASTEXITCODE -ne 0) { throw "xvlog failed" }
  Write-Host "==== xelab $Top ====" -ForegroundColor Cyan
  & "$BIN\xelab.bat" $Top -s "${Top}_snap" --timescale 1ns/1ps
  if ($LASTEXITCODE -ne 0) { throw "xelab failed" }
  Write-Host "==== xsim $Top ====" -ForegroundColor Cyan
  & "$BIN\xsim.bat" "${Top}_snap" -runall
  if ($LASTEXITCODE -ne 0) { throw "xsim failed" }
} finally { Pop-Location }
