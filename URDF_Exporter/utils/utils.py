# -*- coding: utf-8 -*-
"""Utility functions for URDF generation from Fusion 360.

This module provides various utility functions for:
- File and directory operations
- STL mesh export from Fusion 360 components
- XML processing and formatting
- URDF package structure creation
- Physical property calculations and transformations
- Name conversion and validation

Created on Sun May 12 19:15:34 2019
@author: syuntoku
"""

import fileinput
import os.path
import re
import shutil
import sys
from typing import TYPE_CHECKING, TypedDict
from xml.dom import minidom
from xml.etree import ElementTree

# pyright: reportMissingImports=false
import adsk
import adsk.core
import adsk.fusion

# Import for type hints - avoid circular imports by using TYPE_CHECKING
if TYPE_CHECKING:
    from ..core.Joint import Joint
    from ..core.Link import Link


class UrdfInfo(TypedDict):
    """Type definition for URDF generation information dictionary.

    Contains all necessary information for generating URDF files and
    associated ROS package structure from Fusion 360 designs.

    Attributes:
        robot_name: Name of the robot (used in URDF and file names)
        package_name: Name of the ROS package
        package_dir: Full path to the package directory
        package_template_dir: Path to the package template directory
        urdf_dir: Directory path for URDF files
        meshes_dir: Directory path for mesh files (STL)
        launch_dir: Directory path for launch files
        repo: Repository reference string for mesh file paths in URDF
        joints: Dictionary of Joint objects keyed by joint name
        links: Dictionary of Link objects keyed by link name
    """

    robot_name: str
    package_name: str
    package_dir: str
    package_template_dir: str
    urdf_dir: str
    meshes_dir: str
    launch_dir: str
    repo: str
    joints: "dict[str, Joint]"
    links: "dict[str, Link]"


def convert_occ_name(occ_name: str) -> str:
    """Convert Fusion 360 occurrence name to valid URDF link/joint name.

    Removes version suffixes (e.g., ":1", ":2") from Fusion 360 occurrence names
    to create clean, consistent names for URDF links and joints.

    Args:
        occ_name: Original occurrence name from Fusion 360

    Returns:
        str: Cleaned name suitable for URDF (version suffix removed)

    Example:
        >>> convert_occ_name("wheel:1")
        "wheel"
        >>> convert_occ_name("base_link:2")
        "base_link"
    """
    # The pattern is xxx:1, find the ":1" part and remove it
    if re.search(":[0-9]+$", occ_name):
        occ_name = re.sub(":[0-9]+$", "", occ_name)
    return occ_name


def make_package_structure(urdf_infos: UrdfInfo) -> None:
    """Create the directory structure for a ROS package.

    Creates all necessary directories for a complete ROS package including:
    - Main package directory
    - meshes/ subdirectory for STL files
    - urdf/ subdirectory for URDF files
    - launch/ subdirectory for launch files

    Args:
        urdf_infos: Dictionary containing directory paths

    Note:
        Creates directories recursively if they don't exist. Safe to call
        multiple times - will not overwrite existing directories.
    """
    package_dir = urdf_infos["package_dir"]
    meshes_dir = urdf_infos["meshes_dir"]
    urdf_dir = urdf_infos["urdf_dir"]
    launch_dir = urdf_infos["launch_dir"]

    if not os.path.exists(package_dir):
        os.makedirs(package_dir)
    if not os.path.exists(meshes_dir):
        os.makedirs(meshes_dir)
    if not os.path.exists(urdf_dir):
        os.makedirs(urdf_dir)
    if not os.path.exists(launch_dir):
        os.makedirs(launch_dir)


def export_stl(
    design: adsk.fusion.Design,
    urdf_infos: UrdfInfo,
) -> None:
    """Export STL mesh files for all occurrences in a Fusion 360 design.

    Processes all occurrences in the design's root component and exports each
    as an STL file to the meshes directory. Uses low mesh refinement for faster
    export and smaller file sizes suitable for robotics applications.

    Args:
        design: Fusion 360 design object containing the robot model
        urdf_infos: Dictionary containing mesh directory path and other info

    Note:
        - Exports in ASCII STL format (not binary)
        - Uses MeshRefinementLow for performance
        - Prints progress and error messages to console
        - Skips components that cannot be exported due to errors
    """

    # create a single exportManager instance
    exportMgr = design.exportManager
    meshes_dir = urdf_infos["meshes_dir"]

    # export the occurrence one by one in the component to a specified file
    occurrences = design.rootComponent.allOccurrences

    for occ in occurrences:
        try:
            print(occ.component.name)
            fileName = meshes_dir + "/" + occ.component.name
            # create stl exportOptions
            stlExportOptions = exportMgr.createSTLExportOptions(occ.component, fileName)
            stlExportOptions.sendToPrintUtility = False
            stlExportOptions.isBinaryFormat = False
            # options are .MeshRefinementLow .MeshRefinementMedium .MeshRefinementHigh
            stlExportOptions.meshRefinement = (
                adsk.fusion.MeshRefinementSettings.MeshRefinementLow  # type: ignore
            )
            print("Exporting " + occ.component.name + " to " + fileName)
            print(f"Unit: {stlExportOptions.unitType}")

            exportMgr.execute(stlExportOptions)
        except Exception:
            print("Component " + occ.component.name + " has something wrong.")


def file_dialog(ui: adsk.core.UserInterface) -> str | bool:
    """Display folder selection dialog for choosing output directory.

    Opens a folder selection dialog allowing the user to choose where
    to save the generated URDF package files.

    Args:
        ui: Fusion 360 user interface object

    Returns:
        str: Selected folder path if user clicked OK
        bool: False if user cancelled the dialog
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = "Fusion Folder Dialog"

    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(
    inertia: list[float], center_of_mass: list[float], mass: float
) -> list[float]:
    """Transform inertia tensor from world origin to center of mass frame.

    Applies the parallel axis theorem to convert inertia tensor values from
    the world coordinate frame to the center of mass frame, as required by URDF.

    Args:
        inertia: Inertia tensor about world origin [ixx, iyy, izz, ixy, iyz, ixz]
        center_of_mass: Center of mass coordinates [x, y, z] in meters
        mass: Mass of the body in kilograms

    Returns:
        list[float]: Inertia tensor about center of mass [ixx, iyy, izz, ixy, iyz, ixz]
                    in kg⋅m²

    Note:
        Uses the parallel axis theorem: I_cm = I_origin - m * d²
        where d is the distance from origin to center of mass.
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2 + z**2, x**2 + z**2, x**2 + y**2, -x * y, -y * z, -x * z]
    return [round(i - mass * t, 6) for i, t in zip(inertia, translation_matrix)]


def prettify(elem: ElementTree.Element) -> str:
    """Format XML element as pretty-printed string with proper indentation.

    Converts an XML ElementTree element to a nicely formatted string with
    consistent indentation for better readability in generated URDF files.

    Args:
        elem: XML ElementTree element to format

    Returns:
        str: Pretty-printed XML string with 2-space indentation

    Note:
        Uses UTF-8 encoding and 2-space indentation for consistency.
    """
    rough_string = ElementTree.tostring(elem, "utf-8")
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def copy_package(urdf_infos: UrdfInfo) -> None:
    """Copy ROS package template files to the target package directory.

    Copies all files from the package template directory to create the basic
    ROS package structure with CMakeLists.txt, package.xml, and other necessary files.

    Args:
        urdf_infos: Dictionary containing package_dir and package_template_dir paths

    Note:
        Uses dirs_exist_ok=True to allow overwriting existing directories.
        Template files will be customized later by update functions.
    """
    package_dir = urdf_infos["package_dir"]
    package_template_dir = urdf_infos["package_template_dir"]

    shutil.copytree(
        package_template_dir, package_dir, dirs_exist_ok=True
    )  # dirs_exist_ok=True allows overwriting


def update_cmakelists(urdf_infos: UrdfInfo) -> None:
    """Update CMakeLists.txt with the correct package name.

    Modifies the CMakeLists.txt file in the package directory to replace
    the template project name with the actual package name.

    Args:
        urdf_infos: Dictionary containing package_name and package_dir

    Note:
        Performs in-place file editing to replace "project(fusion2urdf)"
        with "project({package_name})".
    """
    file_name = urdf_infos["package_dir"] + "/CMakeLists.txt"

    for line in fileinput.input(file_name, inplace=True):
        if "project(fusion2urdf)" in line:
            sys.stdout.write("project(" + urdf_infos["package_name"] + ")\n")
        else:
            sys.stdout.write(line)


def update_package_xml(urdf_infos: UrdfInfo) -> None:
    """Update package.xml with the correct package name and description.

    Modifies the package.xml file to replace template values with the actual
    package name and generates an appropriate description.

    Args:
        urdf_infos: Dictionary containing package_name and package_dir

    Note:
        Updates both the <name> tag and <description> tag with package-specific values.
        Performs in-place file editing.
    """
    file_name = urdf_infos["package_dir"] + "/package.xml"

    for line in fileinput.input(file_name, inplace=True):
        if "<name>" in line:
            sys.stdout.write("  <name>" + urdf_infos["package_name"] + "</name>\n")
        elif "<description>" in line:
            sys.stdout.write(
                "<description>The "
                + urdf_infos["package_name"]
                + " package</description>\n"
            )
        else:
            sys.stdout.write(line)
