# Author-syuntoku14
# Description-Generate URDF file from Fusion 360

from pathlib import Path
import traceback

# pyright: reportMissingImports=false
import adsk
import adsk.core
import adsk.fusion

from .core import (
    make_joints,
    make_links,
    write_control_launch,
    write_gazebo_launch,
    write_gazebo_xacro,
    write_materials_xacro,
    write_transmissions_xacro,
    write_urdf_xacro,
    write_yaml,
)
from .utils import (
    UrdfInfo,
    create_package,
    export_stl,
    file_dialog,
    make_package_structure,
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
        title = "Fusion2URDFNG"
        if not design:
            ui.messageBox("No active Fusion design", title)
            return

        root = design.rootComponent  # root component

        # Popup a dialog to let user specify ros/ros2 package
        package_type, cancelled = ui.inputBox(
            "Enter package type (ros or ros2):", "Package Type", "ros2"
        )
        if package_type not in ["ros", "ros2"]:
            ui.messageBox("Invalid package type. Please enter 'ros' or 'ros2'.", title)
            return

        # --------------------
        # get user's download folder by default
        # home = os.path.expanduser("~")
        # download_dir = os.path.join(home, "Downloads")
        # save_dir = os.path.join(download_dir, "assets")
        save_dir = file_dialog(ui)

        if not save_dir:
            ui.messageBox("Fusion2URDF was canceled", title)
            return 0

        assert isinstance(save_dir, str)
        save_dir = Path(save_dir)

        robot_name = root.name.split()[0]
        package_name = robot_name + "_description"
        package_dir = save_dir / package_name
        urdf_dir = package_dir / "urdf"
        meshes_dir = package_dir / "meshes"
        launch_dir = package_dir / "launch"
        package_template_dir = Path(__file__).resolve().parent / "package"

        # check folder existence
        if package_dir.exists():
            result = ui.messageBox(
                f"Folder '{package_dir}' already exists. Remove it and continue?",
                title,
                adsk.core.MessageBoxButtonTypes.YesNoButtonType,  # type: ignore
                adsk.core.MessageBoxIconTypes.WarningIconType,  # type: ignore
            )
            if result == adsk.core.DialogResults.DialogNo:  # type: ignore
                ui.messageBox("Fusion2URDF was canceled", title)
                return 0
            elif result == adsk.core.DialogResults.DialogYes:  # type: ignore
                import shutil

                shutil.rmtree(package_dir)
                ui.messageBox(
                    f"Removed '{package_dir}' folder, continue with export.", title
                )

        urdf_infos: UrdfInfo = {
            "robot_name": robot_name,
            "package_name": package_name,
            "package_dir": str(package_dir),
            "package_template_dir": str(package_template_dir),
            "urdf_dir": str(urdf_dir),
            "meshes_dir": str(meshes_dir),
            "launch_dir": str(launch_dir),
            "repo": f"package://{package_name}/",
            "joints": {},
            "links": {},
        }

        # create package directory
        make_package_structure(urdf_infos)

        # Generate links
        urdf_infos["links"] = make_links(root, urdf_infos)
        # Generate joints_dict. All joints are related to root.
        urdf_infos["joints"] = make_joints(root)

        # write files
        write_urdf_xacro(urdf_infos)
        write_materials_xacro(urdf_infos)
        write_transmissions_xacro(urdf_infos)
        write_gazebo_xacro(urdf_infos)
        # write_display_launch(urdf_infos)
        write_gazebo_launch(urdf_infos)
        write_control_launch(urdf_infos)
        write_yaml(urdf_infos)

        # copy over package files
        create_package(urdf_infos, package_type)
        export_stl(design, urdf_infos)

        ui.messageBox(msg, title)

    except Exception:
        if ui:
            ui.messageBox("Failed:\n{}".format(traceback.format_exc()))

    return 0
