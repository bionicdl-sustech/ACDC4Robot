# -*- coding: utf-8 -*-
"""
Get informations about links which are components from Fusion360 API

"""
import adsk, adsk.fusion, adsk.core
from xml.etree.ElementTree import ElementTree, Element, SubElement
from ..commands.Fusion2Robot import constants
from . import utils
from .joint import Joint

class Link():
    """
    Link class
    """
    dir = "" # location of the exported file
    def __init__(self, occurrence: adsk.fusion.Occurrence) -> None:
        """
        Parameters:
        ---------
        occurrence: adsk.fusion.Occurrence
            occurrence in Fusion360 refers to linkage
        dir: str
            location of the exported file
        pose: Matrix3D
            adsk.fusion.Occurrence.transform2
            Gets the 3d matrix data that defines this occurrences orientation and position in its assembly context
            the assembly context here is the root component's frame
        
        """
        # TODO: maybe this is a better and accurate way for this, not test
        # self.link = occurrence.createForAssemblyContext(occurrence)
        
        self.link = occurrence
        self.name = None
        # pose can have two meanings:
        # 1. the homogeneous matrix of link-frame L w.r.t world-frame w
        # 2. the coordinates of link-frame L
        self.pose: adsk.core.Matrix3D = occurrence.transform2 
        self.phyPro = occurrence.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)

    def get_parent_joint(self) -> adsk.fusion.Joint:
        """
        Get the parent joint of a robot link
        Every robot link can only have one parent joint,
        but can have multiple child joints to connect child links

        Return:
        j: parent_joints
        
        """
        # text_palette = constants.get_text_palette()
        joint_list: adsk.fusion.JointList = self.link.joints
        # text_palette.writeText("Link: {}".format(self.get_name()))
        for j in joint_list:
            # text_palette.writeText("Joint list item: {}".format(j.name))
            # text_palette.writeText("Joint's child link: {}".format(j.occurrenceOne.fullPathName))
            if j.occurrenceOne == self.link:
                # text_palette.writeText("Return j type: {}".format(j.objectType))
                return j
            else:
                continue

        # text_palette.writeText("Return a None")
        return None

    def get_name(self):
        if self.link.component.name == "base_link":
            name = "base_link"
        else:
            name = utils.get_valid_filename(self.link.fullPathName)

        return name

    def get_pose_sdf(self):
        """
        Return a pose(translation, rotation) expressed in the frame named by @relative_to.
        The first three components (x, y, z) represent the position of the element's origin(in the @relative_to frame)
        The rotation component represents the orientation of the element as either a sequence of Euler rotations(r, p, y)

        """
        pose = utils.matrix3d_2_pose(self.pose)
        return pose

    def get_inertia_sdf(self):
        """
        Return link's moments of inertia about Co(the link's center of mass)
        for the unit vectors Cx, Cy, Cz fixed in the center-of-mass-frame C.
        Note: the orientation of Cx, Cy, Cz relative to Lx, Ly, Lz which is specified
        by the `pose` tag.
        We set center-of-mass frame's orientation to be the same as link-frame L
        """
        (_, w_ixx, w_iyy, w_izz, w_ixy, w_iyz, w_ixz) = self.phyPro.getXYZMomentsOfInertia() # unit: kg*cm^2
        # Use parallel axis theorem to change inertia from w.r.t world-frame's origin to w.r.t CoM
        x = self.phyPro.centerOfMass.x * 0.01 # get the x coordinate w.r.t world-frame, unit: m
        y = self.phyPro.centerOfMass.y * 0.01
        z = self.phyPro.centerOfMass.z * 0.01
        mass = self.phyPro.mass # unit: kg
        com_ixx = w_ixx*0.0001 - mass*(y**2+z**2) # kg*cm^2 -> kg*m^2
        com_iyy = w_iyy*0.0001 - mass*(x**2+z**2)
        com_izz = w_izz*0.0001 - mass*(x**2+y**2)
        com_ixy = w_ixy*0.0001 + mass*(x*y)
        com_iyz = w_iyz*0.0001 + mass*(y*z)
        com_ixz = w_ixz*0.0001 + mass*(x*z)

        # Change inertia tensor to the orientation of link-frame L
        # reference: https://robot.sia.cn/CN/abstract/abstract374.shtml
        R = utils.getRotationMatrix(self.pose)
        R_T = utils.matrixTranspose(R)
        inertia_tensor = [[com_ixx, com_ixy, com_ixz],
                          [com_ixy, com_iyy, com_iyz],
                          [com_ixz, com_iyz, com_izz]]
        
        # moments of inertia has the same orientation of L-frame
        I = utils.matrixMul(utils.matrixMul(R_T, inertia_tensor), R)
        # I = utils.matrixMul(utils.matrixMul(R, inertia_tensor), R_T)

        ixx = I[0][0]
        iyy = I[1][1]
        izz = I[2][2]
        ixy = I[0][1]
        iyz = I[1][2]
        ixz = I[0][2]

        return [ixx, iyy, izz, ixy, iyz, ixz] # unit: kg*m^2
    
    def get_initia_urdf(self):
        """
        Return [ixx, iyy, izz, ixy, iyz, ixz]
        the inertia tensor elements of the link w.r.t the CoM frame C
        CoM frame C has the same orientation with the parent frame
        
        """
        # First, get the inertia tensor and CoM w.r.t the world frame
        (_, w_ixx, w_iyy, w_izz, w_ixy, w_iyz, w_ixz) = self.phyPro.getXYZMomentsOfInertia() # unit: kg*cm^2
        x = self.phyPro.centerOfMass.x * 0.01 # get the x coordinate w.r.t world-frame, unit: m
        y = self.phyPro.centerOfMass.y * 0.01
        z = self.phyPro.centerOfMass.z * 0.01

        # Use parallel axis theorem to change inertia from w.r.t world-frame's origin to w.r.t CoM
        mass = self.phyPro.mass # unit: kg
        com_ixx = w_ixx*0.0001 - mass*(y**2+z**2) # kg*cm^2 -> kg*m^2
        com_iyy = w_iyy*0.0001 - mass*(x**2+z**2)
        com_izz = w_izz*0.0001 - mass*(x**2+y**2)
        com_ixy = w_ixy*0.0001 + mass*(x*y)
        com_iyz = w_iyz*0.0001 + mass*(y*z)
        com_ixz = w_ixz*0.0001 + mass*(x*z)
        inertia_tensor = [[com_ixx, com_ixy, com_ixz],
                          [com_ixy, com_iyy, com_iyz],
                          [com_ixz, com_iyz, com_izz]]
        
        # get the parent frame
        parent_joint = self.get_parent_joint()
        # parent frame for a link has parent joint, is the parent joint frame
        # else, is the link frame itselt
        if parent_joint is None:
            parent_frame: adsk.core.Matrix3D = self.pose
        else:
            joint = Joint(parent_joint)
            parent_frame: adsk.core.Matrix3D = joint.get_joint_frame()

        # Change inertia tensor to the orientation of parent frame


        # # Get eigenvectors, which are the principal axes of the inertia tensor
        # _, eigenvectors = utils.eigenvalues_and_eigenvectors(inertia_tensor)

        # # Get the rotation matrix represents the orientation of the principal axes
        # R = utils.matrixTranspose(eigenvectors)
        # R_T = eigenvectors

        R = utils.getRotationMatrix(parent_frame)
        R_T = utils.matrixTranspose(R)

        # Express inertia tersor w.r.t CoM frame C with axes Cx, Cy, Cz aligned with principal inertia directions
        I = utils.matrixMul(utils.matrixMul(R_T, inertia_tensor), R)
        ixx = I[0][0]
        iyy = I[1][1]
        izz = I[2][2]
        ixy = I[0][1]
        iyz = I[1][2]
        ixz = I[0][2]

        return [ixx, iyy, izz, ixy, iyz, ixz] # unit: kg*m^2

    def get_mass_sdf(self):
        """
        Get the mass value
        """
        mass = self.phyPro.mass # kg
        return mass
    
    def get_mass_urdf(self):
        """
        Get the mass value
        """
        mass = self.phyPro.mass # kg
        return mass
    
    def get_CoM_frame(self) -> adsk.core.Matrix3D:
        """
        Get the CoM frame whose axes Cx, Cy, Cz align with principal inertia direction
        Return:
        ---------
        CoM_frame: adsk.core.Matrix3D
            translation unit: cm
        """
        # TODO: the algorithm comes from ChatGPT, does not seems w
        # NOT use this yet
        (_, w_ixx, w_iyy, w_izz, w_ixy, w_iyz, w_ixz) = self.phyPro.getXYZMomentsOfInertia() # unit: kg*cm^2
        x = self.phyPro.centerOfMass.x  # get the x coordinate w.r.t world-frame
        y = self.phyPro.centerOfMass.y
        z = self.phyPro.centerOfMass.z

        # Use parallel axis theorem to change inertia from w.r.t world-frame's origin to w.r.t CoM
        mass = self.phyPro.mass # unit: kg
        com_ixx = w_ixx*0.0001 - mass*(y**2+z**2) # kg*cm^2 -> kg*m^2
        com_iyy = w_iyy*0.0001 - mass*(x**2+z**2)
        com_izz = w_izz*0.0001 - mass*(x**2+y**2)
        com_ixy = w_ixy*0.0001 + mass*(x*y)
        com_iyz = w_iyz*0.0001 + mass*(y*z)
        com_ixz = w_ixz*0.0001 + mass*(x*z)
        inertia_tensor = [[com_ixx, com_ixy, com_ixz],
                          [com_ixy, com_iyy, com_iyz],
                          [com_ixz, com_iyz, com_izz]]
        
        # Get eigenvectors, which are the principal axes of the inertia tensor
        _, eigenvectors = utils.eigenvalues_and_eigenvectors(inertia_tensor)

        # Get the rotation matrix represents the orientation of the principal axes
        R = utils.matrixTranspose(eigenvectors)

        origin = adsk.core.Point3D.create(x, y, z)
        xAxis = adsk.core.Vector3D.create(eigenvectors[0][0], eigenvectors[0][1], eigenvectors[0][2])
        yAxis = adsk.core.Vector3D.create(eigenvectors[1][0], eigenvectors[1][1], eigenvectors[1][2])
        zAxis = adsk.core.Vector3D.create(eigenvectors[2][0], eigenvectors[2][1], eigenvectors[2][2])

        CoM_frame = adsk.core.Matrix3D.create()
        CoM_frame.setWithCoordinateSystem(origin, xAxis, yAxis, zAxis)

        return CoM_frame

    def get_CoM_sdf(self):
        """
        Return the pose of CoM: [x, y, z, roll, pitch, yaw]
        Center-of-mass frame C relative to the link-frame L
        """
        # TODO: to avoid compatibility issues associated with the negative sign convention for product
        # of inertia, align CoM frame C's axes to the principal inertia directions, so that all the 
        # products of inertia are zero
        
        # Let the orientation of center-of-mass frame C is same as link-frame L
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        w_CoM_x = self.phyPro.centerOfMass.x # point CoM's x coordinate w.r.t world-frame
        w_CoM_y = self.phyPro.centerOfMass.y
        w_CoM_z = self.phyPro.centerOfMass.z 
        w_Lo_x = self.pose.translation.x    # link-frame's origin point's x coordinat w.r.t world-frame
        w_Lo_y = self.pose.translation.y
        w_Lo_z = self.pose.translation.z
        # represent vector Lo-CoM which start from link-fram's origin point to CoM point w.r.t world-frame
        w_Lo_CoM = [[(w_CoM_x-w_Lo_x)*0.01], [(w_CoM_y-w_Lo_y)*0.01], [(w_CoM_z-w_Lo_z)*0.01]] # cm -> m
        # represent world-frame's orientation w.r.t link-frame
        L_R_w = [[self.pose.getCell(0, 0), self.pose.getCell(1, 0), self.pose.getCell(2, 0)],
                 [self.pose.getCell(0, 1), self.pose.getCell(1, 1), self.pose.getCell(2, 1)],
                 [self.pose.getCell(0, 2), self.pose.getCell(1, 2), self.pose.getCell(2, 2)]]

        L_Lo_CoM = utils.changeOrientation(L_R_w, w_Lo_CoM)
        pose_CoM = [L_Lo_CoM[0][0], L_Lo_CoM[1][0], L_Lo_CoM[2][0], roll, pitch, yaw]
        
        return pose_CoM
    
    def get_CoM_urdf(self):
        """
        Return the pose of CoM: [x, y, z, roll, pitch, yaw]
        which is the origin element inside the inertial element
        Center-of-mass frame C relative to the link-frame L
        Link-frame L is coincide with link's parent joint frame Jp
        """
        if self.get_parent_joint() is None:
            # for the first link which does not have parent joint
            # do not know the CoM is w.r.t world frame or the link frame
            # here let it be w.r.t the link frame
            CoM_pose = self.get_CoM_sdf()
        else:
            textPalette: adsk.core.Palette = constants.get_text_palette()
            joint = Joint(self.get_parent_joint())
            textPalette.writeText("Parent joint: {}".format(joint.get_name()))
            parent_joint_frame: adsk.core.Matrix3D = joint.get_joint_frame()
            from_origin, from_xAxis, from_yAxis, from_zAxis = parent_joint_frame.getAsCoordinateSystem()
            textPalette.writeText("Parent joint frame: \n"+utils.matrix3D_to_str(parent_joint_frame))
            
            w_CoM_x = self.phyPro.centerOfMass.x # point CoM's x coordinate w.r.t world-frame
            w_CoM_y = self.phyPro.centerOfMass.y
            w_CoM_z = self.phyPro.centerOfMass.z 

            # Let the CoM-frame C has the same orientation with parent link frame
            CoM_frame_O: adsk.core.Point3D = adsk.core.Point3D.create()
            CoM_frame_O.set(w_CoM_x, w_CoM_y, w_CoM_z)
            CoM_frame: adsk.core.Matrix3D = adsk.core.Matrix3D.create()
            CoM_frame_x: adsk.core.Vector3D = from_xAxis
            CoM_frame_y: adsk.core.Vector3D = from_yAxis
            CoM_frame_z: adsk.core.Vector3D = from_zAxis
            CoM_frame.setWithCoordinateSystem(CoM_frame_O, CoM_frame_x, CoM_frame_y, CoM_frame_z)
            to_origin, to_xAsix, to_yAxis, to_zAxis = CoM_frame.getAsCoordinateSystem()
            textPalette.writeText("CoM frame: \n" + utils.matrix3D_to_str(CoM_frame))

            transform = adsk.core.Matrix3D.create()
            # transform.setToAlignCoordinateSystems(from_origin, from_xAxis, from_yAxis, from_zAxis, 
            #                                     to_origin, to_xAsix, to_yAxis, to_zAxis)
            transform = utils.coordinateTransform(parent_joint_frame, CoM_frame)
            textPalette.writeText("Transform matrix: \n" + utils.matrix3D_to_str(transform))

            CoM_pose = utils.matrix3d_2_pose(transform)

        return CoM_pose
    
    def get_mesh_origin(self):
        """
        Fusion360 returns the mesh file with the same coordinate frame as the reference occurrence
        So the mesh origin transforms the parent-joint frame J to the link-frame L
        this is used for the mesh element of visual and collision in URDF
        Return:
        ---------
        mesh_orign: list
            [x, y, z, roll, pitch, yaw]
        """
        if self.get_parent_joint() is None:
            # for the first link which does not have parent joint
            # haven't found the reference about how to define the mesh origin
            # it might be w.r.t world frame or link frame
            # here let it be w.r.t link frame, which coincide with the link frame
            mesh_origin = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # transform = self.pose
            # mesh_origin = utils.matrix3d_2_pose(transform)
        else:
            joint = Joint(self.get_parent_joint())
            parent_joint_frame: adsk.core.Matrix3D = joint.get_joint_frame()
            textPalette: adsk.core.Palette = constants.get_text_palette()
            textPalette.writeText("Parent joint frame: \n"+utils.matrix3D_to_str(parent_joint_frame))
            # link frame coincides with the mesh's frame
            link_frame: adsk.core.Matrix3D = self.pose
            textPalette.writeText("Link frame: \n" + utils.matrix3D_to_str(link_frame))
            from_origin, from_xAxis, from_yAxis, from_zAxis = parent_joint_frame.getAsCoordinateSystem()
            to_origin, to_xAsix, to_yAxis, to_zAxis = link_frame.getAsCoordinateSystem()

            transform = adsk.core.Matrix3D.create()
            # transform.setToAlignCoordinateSystems(from_origin, from_xAxis, from_yAxis, from_zAxis, 
            #                                     to_origin, to_xAsix, to_yAxis, to_zAxis)
            transform = utils.coordinateTransform(parent_joint_frame, link_frame)
            textPalette.writeText("Transform matrix: \n" + utils.matrix3D_to_str(transform))
            mesh_origin = utils.matrix3d_2_pose(transform)

        return mesh_origin

    def get_collision_sdf(self):
        """
        Return:
            col_name: a string for the <collision> element's name attribute
            geometry_uri: a uri to locate the collisioin's mesh geometry
        """
        col_name = str(self.get_name()) + "_collision"
        robot_name = constants.get_robot_name()
        geometry_uri = "model://" + robot_name + "/meshes/" + str(self.get_name()) + ".stl"
        return col_name, geometry_uri
    
    def get_collision_urdf(self):
        """
        Return:
            col_name: a string for the <collision> element's name attribute
            mesh_loc: the path to the mesh file
        """
        col_name = self.get_name() + "_collision"
        robot_name = constants.get_robot_name()
        # mesh_loc = "package://" + robot_name + "/meshes/" + self.get_name() + ".stl"
        mesh_loc = "meshes/" + self.get_name() + ".stl"
        return col_name, mesh_loc

    def get_visual_sdf(self):
        """
        Return:
            visual_name: a string for the <visual> element's name attribute
            geometry_uri: a uri to locate the visual's mesh geometry
        """
        visual_name = self.get_name() + "_visual"
        robot_name = constants.get_robot_name()
        geometry_uri = "model://" + robot_name + "/meshes/" + str(self.get_name()) + ".stl"
        return visual_name, geometry_uri
    
    def get_visual_urdf(self):
        """
        Return:
            visual_name: a string for the <visual> element's name attribute
            mesh_loc: str
                the path to the mesh file
        """
        visual_name = self.get_name() + "_visual"
        robot_name = constants.get_robot_name()
        # mesh_loc = "package://" + robot_name + "/meshes/" + self.get_name() + ".stl"
        mesh_loc = "meshes/" + self.get_name() + ".stl"
        return visual_name, mesh_loc

    def get_link_sdf_element(self):
        """
        return a Element object for writing link tag in sdf
        """
        link_ele = Element("link")
        link_ele.attrib = {"name": str(self.get_name())}

        # pose
        pose = SubElement(link_ele, "pose")
        pose.text = "{} {} {} {} {} {}".format(self.get_pose_sdf()[0],
                                               self.get_pose_sdf()[1],
                                               self.get_pose_sdf()[2],
                                               self.get_pose_sdf()[3],
                                               self.get_pose_sdf()[4],
                                               self.get_pose_sdf()[5])
        
        # inertial
        inertial = SubElement(link_ele, "inertial")
        mass = SubElement(inertial, "mass")
        mass.text = str(self.get_mass_sdf())
        CoM = SubElement(inertial, "pose")
        CoM.text = "{} {} {} {} {} {}".format(self.get_CoM_sdf()[0],
                                              self.get_CoM_sdf()[1],
                                              self.get_CoM_sdf()[2],
                                              self.get_CoM_sdf()[3],
                                              self.get_CoM_sdf()[4],
                                              self.get_CoM_sdf()[5])
        inertia = SubElement(inertial, "inertia")
        ixx = SubElement(inertia, "ixx")
        iyy = SubElement(inertia, "iyy")
        izz = SubElement(inertia, "izz")
        ixy = SubElement(inertia, "ixy")
        iyz = SubElement(inertia, "iyz")
        ixz = SubElement(inertia, "ixz")
        ixx.text = "{}".format(self.get_inertia_sdf()[0])
        iyy.text = "{}".format(self.get_inertia_sdf()[1])
        izz.text = "{}".format(self.get_inertia_sdf()[2])
        ixy.text = "{}".format(self.get_inertia_sdf()[3])
        iyz.text = "{}".format(self.get_inertia_sdf()[4])
        ixz.text = "{}".format(self.get_inertia_sdf()[5])

        # visual
        visual = SubElement(link_ele, "visual")
        visual_name, visual_uri = self.get_visual_sdf()
        visual.attrib = {"name": visual_name}
        vis_geo = SubElement(visual, "geometry")
        vis_geo_mesh = SubElement(vis_geo, "mesh")
        vis_geo_mesh_uri = SubElement(vis_geo_mesh, "uri")
        vis_geo_mesh_uri.text = visual_uri

        # collision
        collision = SubElement(link_ele, "collision")
        collision_name, collision_uri = self.get_collision_sdf()
        collision.attrib = {"name": collision_name}
        col_geo = SubElement(collision, "geometry")
        col_geo_mesh = SubElement(col_geo, "mesh")
        col_geo_mesh_uri = SubElement(col_geo_mesh, "uri")
        col_geo_mesh_uri.text = collision_uri

        return link_ele
    
    def get_link_urdf_element(self):
        """
        return a Element object for writing link tag in urdf
        """
        # create a link element
        link_ele = Element("link")
        link_ele.attrib = {"name": str(self.get_name())}

        # add inertia sub-element
        inertial = SubElement(link_ele, "inertial")
        origin = SubElement(inertial, "origin")
        origin.attrib = {"xyz": "{} {} {}".format(self.get_CoM_urdf()[0], self.get_CoM_urdf()[1], self.get_CoM_urdf()[2]), 
                         "rpy": "{} {} {}".format(self.get_CoM_urdf()[3], self.get_CoM_urdf()[4], self.get_CoM_urdf()[5])}
        mass = SubElement(inertial, "mass")
        mass.attrib = {"value": "{}".format(self.get_mass_urdf())}
        inertia = SubElement(inertial, "inertia")
        inertia.attrib = {"ixx": "{}".format(self.get_initia_urdf()[0]),
                          "iyy": "{}".format(self.get_initia_urdf()[1]),
                          "izz": "{}".format(self.get_initia_urdf()[2]),
                          "ixy": "{}".format(self.get_initia_urdf()[3]),
                          "iyz": "{}".format(self.get_initia_urdf()[4]),
                          "ixz": "{}".format(self.get_initia_urdf()[5])
                          }

        # add visual sub-element
        visual = SubElement(link_ele, "visual")
        vis_name, visual_mesh_loc = self.get_visual_urdf()
        visual.attrib = {"name": vis_name}
        origin_v = SubElement(visual, "origin")
        origin_v.attrib = {"xyz": "{} {} {}".format(self.get_mesh_origin()[0], self.get_mesh_origin()[1], self.get_mesh_origin()[2]),
                           "rpy": "{} {} {}".format(self.get_mesh_origin()[3], self.get_mesh_origin()[4], self.get_mesh_origin()[5])}
        geometry_v = SubElement(visual, "geometry")
        mesh_v = SubElement(geometry_v, "mesh")
        mesh_v.attrib = {"filename": visual_mesh_loc}

        # add collision sub-element
        collision = SubElement(link_ele, "collision")
        col_name, col_mesh_loc = self.get_collision_urdf()
        collision.attrib = {"name": col_name}
        origin_c = SubElement(collision, "origin")
        origin_c.attrib = {"xyz": "{} {} {}".format(self.get_mesh_origin()[0], self.get_mesh_origin()[1], self.get_mesh_origin()[2]),
                           "rpy": "{} {} {}".format(self.get_mesh_origin()[3], self.get_mesh_origin()[4], self.get_mesh_origin()[5])}
        geometry_c = SubElement(collision, "geometry")
        mesh_c = SubElement(geometry_c, "mesh")
        mesh_c.attrib = {"filename": col_mesh_loc}

        return link_ele