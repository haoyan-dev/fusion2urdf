# Author-syuntoku14
# Description-Generate URDF file from Fusion 360

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
        package_dir = os.path.join(save_dir, "robot_v1_description")

        # create package directory
        try:
            os.makedirs(package_dir, exist_ok=True)
        except Exception as e:
            print(f"Failed to create directory: {package_dir}. Error: {e}")

        # create urdf directory
        try:
            os.makedirs(os.path.join(package_dir, "urdf"), exist_ok=True)
        except Exception as e:
            print(
                f"Failed to create directory: {os.path.join(package_dir, 'urdf')}. Error: {e}"
            )

        # create meshes directory
        try:
            os.makedirs(os.path.join(package_dir, "meshes"), exist_ok=True)
        except Exception as e:
            print(
                f"Failed to create directory: {os.path.join(package_dir, 'meshes')}. Error: {e}"
            )

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
        all_joints = root.joints
        links, success, msg = make_links(
            all_occurrences, repo="package://robot_v1_description/"
        )
        joints, success, msg = make_joints(
            all_joints, "package://robot_v1_description/", all_occurrences
        )

        # save_dir = utils.file_dialog(ui)
        # if not save_dir:
        #     ui.messageBox("Fusion2URDF was canceled", title)
        #     return 0
        write_urdf(joints, links, "robot_v1_description", "robot_v1", save_dir)

        utils.export_stl(design, package_dir, all_occurrences)

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
