# -*- coding: utf-8 -*-
"""
Export robot description format files from Fusion360 design
"""

import adsk, adsk.core, adsk.fusion, traceback
import os, sys
from ...core.link import Link
from ...core.joint import Joint
from . import constants
from ...core import write
from ...core import utils

def run():
    # Initialization
    app = adsk.core.Application.get()
    ui = app.userInterface
    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)

    msg_box_title = "Fusion2Robot"
    
    # open a text palette for debuging
    textPalette = ui.palettes.itemById("TextCommands")
    if not textPalette.isVisible:
        textPalette.isVisible = True
    constants.set_text_palette(textPalette)

    # # Set styles of progress dialog
    # progressDialog = ui.createProgressDialog()
    # progressDialog.cancelButtonText = 'Cancel'
    # progressDialog.isBackgroundTranslucent = False
    # progressDialog.isCancelButtonShown = True

    try:
        # Set design type into do not capture design history
        design.designType = adsk.fusion.DesignTypes.DirectDesignType

        # Check the length unit of Fusion360
        if design.unitsManager.defaultLengthUnits != "m":
            ui.messageBox("Please set length unit to 'm'!", msg_box_title)
            return 0 # exit run() function
        
        root = design.rootComponent # get root component
        allComp = design.allComponents
        robot_name = root.name.split()[0]
        constants.set_robot_name(robot_name)
        
        # Set the folder to store exported files
        folder_dialog = ui.createFolderDialog()
        folder_dialog.title = "Chose your folder to export"
        dialog_result = folder_dialog.showDialog() # show folder dialog
        save_folder = ""
        if dialog_result == adsk.core.DialogResults.DialogOK:
            save_folder = folder_dialog.folder
        else:
            ui.messageBox("Fusion2Robot was canceled", msg_box_title)
            return 0 # exit run() function
        
        save_folder = save_folder + "/" + robot_name
        try: os.mkdir(save_folder)
        except: pass

        ui.messageBox("Start Fusion2Robot Add-IN", msg_box_title)

        # TODO: add a progress dialog, which has example from API documents
        # progressDialog.show("Fusion2RobotSim", "Exporting Robot Model")

        link_list = [] # a list to store link objects
        joint_list = [] # a list to store joint objects

        # try to solve the nested components problem
        # but still not fully tested
        for occ in root.allOccurrences:
            # TODO: it seems use occ.joints.count will make it usable with occurrences? Test it
            if occ.component.joints.count > 0:
                # textPalette.writeText(str(occ.fullPathName))
                continue
            else:
                # Only occurrence contains zero joint and has zero childOccurrences 
                # can be seen as a link
                if occ.childOccurrences.count > 0:
                    # textPalette.writeText(str(occ.fullPathName))
                    # textPalette.writeText(str(occ.childOccurrences.count))
                    continue
                else:
                    # textPalette.writeText(str(occ.fullPathName))
                    # textPalette.writeText(str(occ.childOccurrences.count))
                    link_list.append(Link(occ)) # add link objects into link_list

        for joint in root.allJoints:
            joint_list.append(Joint(joint)) # add joint objects into joint_list

        rdf = constants.get_rdf()
        simulator = constants.get_sim_env()
        if rdf == None:
            ui.messageBox("Robot description format is None. \
                          Please choose a robot description format", msg_box_title)
        elif rdf == "URDF":
            if simulator  == "None":
                ui.messageBox("Simulation environment is None. Please select a simulation environment.", msg_box_title)
            elif simulator == "Gazebo":
                # write to .urdf file
                write.write_urdf(link_list, joint_list, save_folder, robot_name)
                # export mesh files
                utils.export_stl(design, save_folder)
                ui.messageBox("Finish exporting URDF for Gazebo.", msg_box_title)
                
            elif simulator == "PyBullet":
                # write to .urdf file
                write.write_urdf(link_list, joint_list, save_folder, robot_name)
                # export mesh files
                utils.export_stl(design, save_folder)
                # generate pybullet script
                write.write_hello_pybullet(rdf, robot_name, save_folder)
                ui.messageBox("Finish exporting URDF for PyBullet.", msg_box_title)

        elif rdf == "SDFormat":
            if simulator == "None":
                ui.messageBox("Simulation environment is None. \
                              Please select a simulation environment.", msg_box_title)
            elif simulator == "Gazebo":
                # write to .sdf file
                write.write_sdf(link_list, joint_list, save_folder, robot_name)
                # write a model cofig file
                author = constants.get_author_name()
                des = constants.get_model_description()
                write.write_sdf_config(save_folder, robot_name, author, des)
                # export stl files
                utils.export_stl(design, save_folder)
                ui.messageBox("Finish exporting SDFormat for Gazebo.", msg_box_title)

            elif simulator == "PyBullet":
                # write to .sdf file
                write.write_sdf(link_list, joint_list, save_folder, robot_name)
                # export stl files
                utils.export_stl(design, save_folder)
                # generate pybullet script
                write.write_hello_pybullet(rdf,robot_name, save_folder)
                ui.messageBox("Finish exporting SDFormat for PyBullet.", msg_box_title)

        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
