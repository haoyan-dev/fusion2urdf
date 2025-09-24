# Author-syuntoku14
# Description-Generate URDF file from Fusion 360

from typing import Any
import adsk
import adsk.core
import adsk.fusion
import traceback
import os
import sys

from .utils import utils
from .core import (
    Joint,
    Link,
    make_joints,
    make_links,
    write_urdf,
    write_materials_xacro,
    write_transmissions_xacro,
    write_gazebo_xacro,
    write_display_launch,
    write_gazebo_launch,
    write_control_launch,
    write_yaml,
)

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model


def run(context):
    ui = None
    success_msg = "Successfully create URDF file"
    msg = success_msg

    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = "Fusion2URDF"
        if not design:
            ui.messageBox("No active Fusion design", title)
            return

        root = design.rootComponent  # root component
        components = design.allComponents

        # set the names
        # robot_name = root.name.split()[0]
        # package_name = robot_name + "_description"
        # save_dir = utils.file_dialog(ui)
        # if save_dir == False:
        #     ui.messageBox("Fusion2URDF was canceled", title)
        #     return 0

        # save_dir = save_dir + "/" + package_name
        # try:
        #     os.mkdir(save_dir)
        # except FileExistsError:
        #     ui.messageBox("Folder already exists. Continuing...", title)
        #     pass
        # except Exception as e:
        #     ui.messageBox("Failed to create directory: {}".format(save_dir), title)
        #     return 0

        # package_dir = os.path.abspath(os.path.dirname(__file__)) + "/package/"

        # get user's download folder by default
        home = os.path.expanduser("~")
        download_dir = os.path.join(home, "Downloads")
        save_dir = os.path.join(download_dir, "assets")
        robot_name = root.name.split()[0]
        package_name = robot_name + "_description"
        package_dir = os.path.join(save_dir, package_name)
        urdf_dir = os.path.join(package_dir, "urdf")
        meshes_dir = os.path.join(package_dir, "meshes")
        launch_dir = os.path.join(package_dir, "launch")
        package_template_dir = os.path.abspath(
            os.path.dirname(__file__) + "/package/"
        )

        urdf_infos: dict[str, Any] = {
            "robot_name": robot_name,
            "package_name": package_name,
            "package_dir": package_dir,
            "package_template_dir": package_template_dir,
            "urdf_dir": urdf_dir,
            "meshes_dir": meshes_dir,
            "launch_dir": launch_dir,
            "repo": f"package://{package_name}/",
        }

        # create package directory
        if not os.path.exists(package_dir):
            os.makedirs(package_dir)

        # create urdf directory
        if not os.path.exists(os.path.join(package_dir, "urdf")):
            os.makedirs(os.path.join(package_dir, "urdf"))
    
        # create meshes directory
        if not os.path.exists(os.path.join(package_dir, "meshes")):
            os.makedirs(os.path.join(package_dir, "meshes"))
        # create launch directory
        if not os.path.exists(os.path.join(package_dir, "launch")):
            os.makedirs(os.path.join(package_dir, "launch"))

        # Check all occurrences
        all_occurrences = root.allOccurrences
        # for occ in all_occurrences:
        #     ui.messageBox("Component: " + occ.component.name, title)
        #     ui.messageBox(
        #         "Transform: "
        #         + str(occ.transform2.asArray())
        #         + "\n"
        #         + "Is Grounded: "
        #         + str(occ.isGrounded),
        #         title,
        #     )

        for occ in all_occurrences:
            if occ != root:
                print("Component: " + occ.component.name)
                pass
            else:
                print("Root Component: " + occ.component.name)
                pass

        # Check joints
        joints, links, success, msg = make_joints(
            root, urdf_infos
        )

        urdf_infos["joints"] = joints
        urdf_infos["links"] = links

        # save_dir = utils.file_dialog(ui)
        # if not save_dir:
        #     ui.messageBox("Fusion2URDF was canceled", title)
        #     return 0
        write_urdf(urdf_infos)
        write_materials_xacro(urdf_infos)
        write_transmissions_xacro(urdf_infos)
        write_gazebo_xacro(urdf_infos)
        write_display_launch(urdf_infos)
        write_gazebo_launch(urdf_infos)
        write_control_launch(urdf_infos)
        write_yaml(urdf_infos)

        # copy over package files
        utils.copy_package(urdf_infos)
        utils.update_cmakelists(urdf_infos)
        utils.update_package_xml(urdf_infos)
        utils.export_stl(design, urdf_infos)

        print("Completed STL export to: " + urdf_infos["package_dir"])

        return 0
        # --------------------

        # Generate inertial_dict

        ui.messageBox(msg, title)
        if not success:
            return 0

        ui.messageBox(msg, title)

        # Generate joints_dict. All joints are related to root.
        joints, success, msg = make_joints(root)
        ui.messageBox(msg, title)

        if not success:
            return 0

        # --------------------
        # Generate URDF
        write_urdf(joints, links, package_name, robot_name, save_dir)
        write_materials_xacro(robot_name, save_dir)
        write_transmissions_xacro(joints, robot_name, save_dir)
        write_gazebo_xacro(joints, robot_name, save_dir)
        write_display_launch(package_name, robot_name, save_dir)
        write_gazebo_launch(package_name, robot_name, save_dir)
        write_control_launch(package_name, robot_name, save_dir, joints)
        write_yaml(joints, robot_name, save_dir)

        # copy over package files
        utils.copy_package(save_dir, package_dir)
        utils.update_cmakelists(save_dir, package_name)
        utils.update_package_xml(save_dir, package_name)

        # Generate STl files
        utils.export_stl(design, save_dir, components)

        ui.messageBox(msg, title)

    except Exception:
        if ui:
            ui.messageBox("Failed:\n{}".format(traceback.format_exc()))
