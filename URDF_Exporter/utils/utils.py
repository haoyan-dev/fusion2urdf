# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk
import adsk.core
import adsk.fusion
import os.path
import re
from xml.etree import ElementTree
from xml.dom import minidom
import shutil  # Replaced distutils with shutil
import fileinput
import sys


def convert_occ_name(occ_name: str) -> str:
    """
    Convert the occurrence name to a valid URDF link/joint name by replacing
    spaces and special characters with underscores.

    Parameters
    ----------
    occ_name : str
        The original occurrence name.

    Returns
    -------
    str
        The converted name suitable for URDF.
    """
    # The pattern is xxx:1, find the ":1" part and remove it
    if re.search(":[0-9]+$", occ_name):
        occ_name = re.sub(":[0-9]+$", "", occ_name)
    return occ_name


def export_stl(
    design: adsk.fusion.Design,
    package_dir: str,
    occurrences: adsk.fusion.OccurrenceList,
):
    """
    export stl files into "save_dir/"

    Parameters
    ----------
    design: adsk.fusion.Design.cast(product)
    save_dir: str
        directory path to save
    components: design.allComponents
    """

    # create a single exportManager instance
    exportMgr = design.exportManager
    # get the script location
    try:
        os.mkdir(package_dir + "/meshes")
    except FileExistsError:
        pass

    scriptDir = package_dir + "/meshes"
    # export the occurrence one by one in the component to a specified file
    for occ in occurrences:
        try:
            print(occ.component.name)
            fileName = scriptDir + "/" + occ.component.name
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


def file_dialog(ui):
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = "Fusion Folder Dialog"

    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into
    that about center of mass coordinate

    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]

    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2 + z**2, x**2 + z**2, x**2 + y**2, -x * y, -y * z, -x * z]
    return [round(i - mass * t, 6) for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.

    Parameters
    ----------
    elem : xml.etree.ElementTree.Element

    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, "utf-8")
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def copy_package(save_dir, package_dir):
    try:
        # Check if the target directory exists, if not, create it
        if not os.path.exists(save_dir + "/launch"):
            os.mkdir(save_dir + "/launch")
        if not os.path.exists(save_dir + "/urdf"):
            os.mkdir(save_dir + "/urdf")

        # Check if the package directory exists and copy it
        if os.path.exists(package_dir):
            shutil.copytree(
                package_dir, save_dir, dirs_exist_ok=True
            )  # dirs_exist_ok=True allows overwriting
        else:
            print(f"Package directory '{package_dir}' does not exist.")

    except Exception as e:
        print(f"Error copying package: {e}")


def update_cmakelists(save_dir, package_name):
    file_name = save_dir + "/CMakeLists.txt"

    for line in fileinput.input(file_name, inplace=True):
        if "project(fusion2urdf)" in line:
            sys.stdout.write("project(" + package_name + ")\n")
        else:
            sys.stdout.write(line)


def update_package_xml(save_dir, package_name):
    file_name = save_dir + "/package.xml"

    for line in fileinput.input(file_name, inplace=True):
        if "<name>" in line:
            sys.stdout.write("  <name>" + package_name + "</name>\n")
        elif "<description>" in line:
            sys.stdout.write(
                "<description>The " + package_name + " package</description>\n"
            )
        else:
            sys.stdout.write(line)
