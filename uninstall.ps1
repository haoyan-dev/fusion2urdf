#Requires -Version 5.1

<#
.SYNOPSIS
    Uninstall URDF Exporter for Fusion 360
.DESCRIPTION
    This script removes the URDF Exporter add-in from Fusion 360's Scripts directory.
    It can handle both copied files and symbolic links.
.PARAMETER Force
    Remove without prompting for confirmation.
.EXAMPLE
    .\uninstall.ps1
    Uninstall with confirmation prompt
.EXAMPLE
    .\uninstall.ps1 -Force
    Uninstall without confirmation prompt
#>

param(
    [switch]$Force
)

# Function to uninstall URDF Exporter
function Uninstall-URDFExporter {
    param(
        [bool]$ForceUninstall = $false
    )
    
    try {
        # Define target directory
        $targetDir = Join-Path $env:APPDATA "Autodesk\Autodesk Fusion 360\API\Scripts\URDF_Exporter"
        
        # Check if installation exists
        if (-not (Test-Path $targetDir)) {
            Write-Host "❌ URDF_Exporter is not installed." -ForegroundColor Red
            Write-Host "Installation not found at: $targetDir"
            return $true
        }
        
        # Determine installation type
        $item = Get-Item $targetDir
        $installationType = if ($item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) {
            "symbolic link"
        } else {
            "copied files"
        }
        
        Write-Host "Found URDF_Exporter installation ($installationType) at:" -ForegroundColor Cyan
        Write-Host $targetDir
        Write-Host ""
        
        # Confirm uninstallation
        if (-not $ForceUninstall) {
            $response = Read-Host "Are you sure you want to remove URDF_Exporter? (y/N)"
            if ($response -notmatch "^[Yy]") {
                Write-Host "Uninstallation cancelled."
                return $false
            }
        }
        
        # Remove installation
        Write-Host "Removing URDF_Exporter..."
        Remove-Item $targetDir -Recurse -Force
        
        # Verify removal
        if (Test-Path $targetDir) {
            Write-Error "Failed to remove URDF_Exporter completely."
            return $false
        }
        
        Write-Host ""
        Write-Host "URDF Exporter uninstalled successfully!" -ForegroundColor Green
        Write-Host ""
        Write-Host "The add-in has been removed from Fusion 360."
        Write-Host "You may need to restart Fusion 360 to see the changes."
        Write-Host ""
        
        return $true
        
    } catch {
        Write-Error "Uninstallation failed: $($_.Exception.Message)"
        return $false
    }
}

# Function to check for other Fusion 360 add-ins in Scripts directory
function Show-ScriptsDirectoryStatus {
    try {
        $scriptsDir = Join-Path $env:APPDATA "Autodesk\Autodesk Fusion 360\API\Scripts"
        
        if (Test-Path $scriptsDir) {
            $items = Get-ChildItem $scriptsDir -Directory | Where-Object { $_.Name -ne "URDF_Exporter" }
            
            if ($items.Count -gt 0) {
                Write-Host "Other add-ins in Scripts directory:" -ForegroundColor Yellow
                foreach ($item in $items) {
                    $type = if ($item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) { " (symlink)" } else { "" }
                    Write-Host "  • $($item.Name)$type"
                }
            } else {
                Write-Host "No other add-ins found in Scripts directory." -ForegroundColor Gray
            }
        }
    } catch {
        # Silently ignore errors when checking other add-ins
    }
}

# Main execution
Write-Host "URDF Exporter for Fusion 360 - Uninstaller" -ForegroundColor Yellow
Write-Host "===========================================" -ForegroundColor Yellow
Write-Host ""

$success = Uninstall-URDFExporter -ForceUninstall $Force

if ($success) {
    Show-ScriptsDirectoryStatus
} else {
    Write-Host ""
    Write-Host "❌ Uninstallation failed!" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "Press any key to continue..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")