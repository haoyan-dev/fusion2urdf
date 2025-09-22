#Requires -Version 5.1

<#
.SYNOPSIS
    Install URDF Exporter for Fusion 360
.DESCRIPTION
    This script installs the URDF Exporter add-in to Fusion 360's Scripts directory.
    It supports both copying files and creating symbolic links.
.PARAMETER UseSymbolicLink
    Create a symbolic link instead of copying files. Requires Administrator privileges.
.PARAMETER Force
    Overwrite existing installation without prompting.
.EXAMPLE
    .\install.ps1
    Install by copying files
.EXAMPLE
    .\install.ps1 -UseSymbolicLink
    Install using symbolic link (requires Administrator privileges)
.EXAMPLE
    .\install.ps1 -Force
    Install and overwrite existing files without prompting
#>

param(
    [switch]$UseSymbolicLink,
    [switch]$Force
)

# Function to check if running as Administrator
function Test-Administrator {
    $currentUser = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($currentUser)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

# Function to install URDF Exporter
function Install-URDFExporter {
    param(
        [bool]$UseSymLink = $false,
        [bool]$ForceInstall = $false
    )
    
    try {
        # Define paths
        $scriptDir = if ($PSScriptRoot) { 
            $PSScriptRoot 
        } elseif ($MyInvocation.MyCommand.Path) { 
            Split-Path -Parent $MyInvocation.MyCommand.Path 
        } else { 
            Get-Location 
        }
        $sourceDir = Join-Path $scriptDir "URDF_Exporter"
        $targetDir = Join-Path $env:APPDATA "Autodesk\Autodesk Fusion 360\API\Scripts\URDF_Exporter"
        
        # Validate source directory exists
        if (-not (Test-Path $sourceDir)) {
            Write-Error "Source directory not found: $sourceDir"
            Write-Error "Make sure you're running this script from the fusion2urdf directory."
            return $false
        }
        
        # Check if target already exists
        if (Test-Path $targetDir) {
            if (-not $ForceInstall) {
                $response = Read-Host "URDF_Exporter already exists at $targetDir. Overwrite? (y/N)"
                if ($response -notmatch "^[Yy]") {
                    Write-Host "Installation cancelled."
                    return $false
                }
            }
            
            # Remove existing installation
            Write-Host "Removing existing installation..."
            Remove-Item $targetDir -Recurse -Force
        }
        
        # Create parent directory if it doesn't exist
        $parentDir = Split-Path $targetDir -Parent
        if (-not (Test-Path $parentDir)) {
            Write-Host "Creating Fusion 360 Scripts directory..."
            New-Item -ItemType Directory -Path $parentDir -Force | Out-Null
        }
        
        if ($UseSymLink) {
            # Check for Administrator privileges
            if (-not (Test-Administrator)) {
                Write-Error "Symbolic link creation requires Administrator privileges."
                Write-Error "Please run PowerShell as Administrator or use regular copy mode."
                return $false
            }
            
            # Create symbolic link
            Write-Host "Creating symbolic link..."
            New-Item -ItemType SymbolicLink -Path $targetDir -Target $sourceDir | Out-Null
            Write-Host "Symbolic link created successfully!"
        } else {
            # Copy files
            Write-Host "Copying URDF_Exporter files..."
            Copy-Item $sourceDir -Destination $targetDir -Recurse -Force
            Write-Host "Files copied successfully!"
        }
        
        Write-Host ""
        Write-Host "URDF Exporter installed successfully!" -ForegroundColor Green
        Write-Host "Installation location: $targetDir" -ForegroundColor Cyan
        Write-Host ""
        Write-Host "To use the add-in:"
        Write-Host "1. Open Fusion 360"
        Write-Host "2. Go to DESIGN -> ADD-INS"
        Write-Host "3. Select 'URDF_Exporter' from the Scripts tab"
        Write-Host "4. Click 'Run'"
        Write-Host ""
        
        return $true
        
    } catch {
        Write-Error "Installation failed: $($_.Exception.Message)"
        return $false
    }
}

# Main execution
Write-Host "URDF Exporter for Fusion 360 - Installer" -ForegroundColor Yellow
Write-Host "=========================================" -ForegroundColor Yellow
Write-Host ""

if ($UseSymbolicLink) {
    Write-Host "Installing using symbolic link..." -ForegroundColor Cyan
} else {
    Write-Host "Installing by copying files..." -ForegroundColor Cyan
}

$success = Install-URDFExporter -UseSymLink $UseSymbolicLink -ForceInstall $Force

if (-not $success) {
    Write-Host ""
    Write-Host "Installation failed!" -ForegroundColor Red
    exit 1
}

Write-Host "Press any key to continue..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")