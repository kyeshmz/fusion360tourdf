# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019

@author: syuntoku
"""

import adsk
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils

class Joint:
    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit):
        """
        Attributes
        ----------
        name: str
            name of the joint
        type: str
            type of the joint(ex: rev)
        xyz: [x, y, z]
            coordinate of the joint
        axis: [x, y, z]
            coordinate of axis of the joint
        parent: str
            parent link
        child: str
            child link
        joint_xml: str
            generated xml describing about the joint
        tran_xml: str
            generated xml describing about the transmission
        """
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.axis = axis  # for 'revolute' and 'continuous'
        self.upper_limit = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit = lower_limit  # for 'revolute' and 'prismatic'
        
    def make_joint_xml(self):
        """
        Generate the joint_xml and hold it by self.joint_xml
        """
        joint = Element('joint')
        joint.attrib = {'name':self.name, 'type':self.type}
        
        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link':self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link':self.child}
        if self.type == 'revolute' or self.type == 'continuous' or self.type == 'prismatic':        
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz':' '.join([str(_) for _ in self.axis])}
        if self.type == 'revolute' or self.type == 'prismatic':
            limit = SubElement(joint, 'limit')
            limit.attrib = {'upper': str(self.upper_limit), 'lower': str(self.lower_limit),
                            'effort': '100', 'velocity': '100'}
            
        self.joint_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

    def make_transmission_xml(self):
        """
        Generate the tran_xml and hold it by self.tran_xml
        
        
        Notes
        -----------
        mechanicalTransmission: 1
        type: transmission interface/SimpleTransmission
        hardwareInterface: PositionJointInterface        
        """        
        
        tran = Element('transmission')
        tran.attrib = {'name':self.name + '_tran'}
        
        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'
        
        joint = SubElement(tran, 'joint')
        joint.attrib = {'name':self.name}
        hardwareInterface_joint = SubElement(joint, 'hardwareInterface')
        hardwareInterface_joint.text = 'hardware_interface/EffortJointInterface'
        
        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name':self.name + '_actr'}
        hardwareInterface_actr = SubElement(actuator, 'hardwareInterface')
        hardwareInterface_actr.text = 'hardware_interface/EffortJointInterface'
        mechanicalReduction = SubElement(actuator, 'mechanicalReduction')
        mechanicalReduction.text = '1'
        
        self.tran_xml = "\n".join(utils.prettify(tran).split("\n")[1:])


def make_joints_dict(root, msg):
    """
    joints_dict holds parent, axis and xyz informatino of the joints


    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status

    Returns
    ----------
    joints_dict:
        {name: {type, axis, upper_limit, lower_limit, parent, child, xyz}}
    msg: str
        Tell the status
    """

    joints_dict = {}

    def iter_joints():
        try:
            for joint in root.joints:
                yield joint, False
        except Exception:
            return
        try:
            as_built = getattr(root, 'asBuiltJoints', None)
            if as_built is not None:
                for joint in as_built:
                    yield joint, True
        except Exception:
            return

    for joint, is_as_built in iter_joints():
        joint_dict = {}
        try:
            type_value = _joint_type_value(joint.jointMotion)
        except Exception:
            msg = getattr(joint, 'name', 'joint') + ' has an unreadable joint type.'
            break
        joint_type = _urdf_type_for(type_value)
        if joint_type is None:
            fusion_name = _FUSION_JOINT_NAMES.get(type_value, str(type_value))
            msg = (getattr(joint, 'name', 'joint') +
                   ' uses ' + fusion_name + ' joints, which URDF does not support.'
                   ' Please use Rigid, Revolute, or Slider joints.')
            break
        joint_dict['type'] = joint_type

        # swhich by the type of the joint
        joint_dict['axis'] = [0, 0, 0]
        joint_dict['upper_limit'] = 0.0
        joint_dict['lower_limit'] = 0.0

        # support  "Revolute", "Rigid" and "Slider"
        if joint_type == 'revolute':
            try:
                joint_dict['axis'] = [round(i, 6) for i in
                    joint.jointMotion.rotationAxisVector.asArray()]  # In Fusion, exported axis is normalized.
            except Exception:
                msg = joint.name + ' has an unreadable rotation axis.'
                break
            max_enabled = joint.jointMotion.rotationLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.rotationLimits.isMinimumValueEnabled
            if max_enabled and min_enabled:
                joint_dict['upper_limit'] = round(joint.jointMotion.rotationLimits.maximumValue, 6)
                joint_dict['lower_limit'] = round(joint.jointMotion.rotationLimits.minimumValue, 6)
            elif max_enabled and not min_enabled:
                msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                break
            else:  # if there is no angle limit
                joint_dict['type'] = 'continuous'

        elif joint_type == 'prismatic':
            try:
                joint_dict['axis'] = [round(i, 6) for i in
                    joint.jointMotion.slideDirectionVector.asArray()]  # Also normalized
            except Exception:
                msg = joint.name + ' has an unreadable slide direction.'
                break
            max_enabled = joint.jointMotion.slideLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.slideLimits.isMinimumValueEnabled
            if max_enabled and min_enabled:
                joint_dict['upper_limit'] = round(joint.jointMotion.slideLimits.maximumValue / 100, 6)
                joint_dict['lower_limit'] = round(joint.jointMotion.slideLimits.minimumValue / 100, 6)
            elif max_enabled and not min_enabled:
                msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                break
        elif joint_type == 'fixed':
            pass

        try:
            occ_two = joint.occurrenceTwo
            occ_one = joint.occurrenceOne
        except Exception:
            msg = joint.name + " doesn't reference two occurrences. Please set it and run again."
            break
        try:
            if occ_two.component.name == 'base_link':
                joint_dict['parent'] = 'base_link'
            else:
                joint_dict['parent'] = utils.sanitize_name(occ_two.name)
        except Exception:
            msg = joint.name + ' has an unreadable parent occurrence.'
            break
        try:
            joint_dict['child'] = utils.sanitize_name(occ_one.name)
        except Exception:
            msg = joint.name + ' has an unreadable child occurrence.'
            break


        #There seem to be a problem with geometryOrOriginTwo. To calcualte the correct orogin of the generated stl files following approach was used.
        #https://forums.autodesk.com/t5/fusion-360-api-and-scripts/difference-of-geometryororiginone-and-geometryororiginonetwo/m-p/9837767
        #Thanks to Masaki Yamamoto!
        # Update: Occurrence.transform was retired in July 2025; use transform2.
        # geometryOneTransform/geometryTwoTransform are used as a fallback when
        # joint geometry is no longer available.

        try:
            xyz_of_joint_cm = _resolve_joint_xyz_cm(joint, is_as_built)
            if xyz_of_joint_cm is None:
                raise ValueError('no origin')
            joint_dict['xyz'] = [round(i / 100.0, 6) for i in xyz_of_joint_cm]  # converted to meter

        except Exception:
            msg = joint.name + " doesn't have joint origin. Please set it and run again."
            break

        name = utils.sanitize_name(joint.name)
        if name in joints_dict:
            suffix = '_asbuilt' if is_as_built else '_2'
            candidate = name + suffix
            counter = 2
            while candidate in joints_dict:
                counter += 1
                candidate = name + '_' + str(counter)
            name = candidate
        joints_dict[name] = joint_dict
    return joints_dict, msg


# Fusion JointTypes enum values, see
# https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_JointTypes.htm
# 0 Rigid, 1 Revolute, 2 Slider, 3 Cylindrical, 4 PinSlot, 5 Planar, 6 Ball,
# 7 Inferred.
_FUSION_JOINT_NAMES = {
    0: 'Rigid',
    1: 'Revolute',
    2: 'Slider',
    3: 'Cylindrical',
    4: 'PinSlot',
    5: 'Planar',
    6: 'Ball',
    7: 'Inferred',
}

_FUSION_TO_URDF = {
    0: 'fixed',
    1: 'revolute',
    2: 'prismatic',
}


def _joint_type_value(joint_motion):
    """Return the integer JointTypes value for a joint motion."""
    joint_type = joint_motion.jointType
    for attr in ('value', 'value__'):
        try:
            value = getattr(joint_type, attr, None)
            if isinstance(value, int):
                return value
        except Exception:
            continue
    try:
        return int(joint_type)
    except Exception:
        pass
    class_name = type(joint_motion).__name__
    by_class = {
        'RigidJointMotion': 0,
        'RevoluteJointMotion': 1,
        'SliderJointMotion': 2,
        'CylindricalJointMotion': 3,
        'PinSlotJointMotion': 4,
        'PlanarJointMotion': 5,
        'BallJointMotion': 6,
    }
    if class_name in by_class:
        return by_class[class_name]
    raise ValueError('Unknown joint type: %r' % (joint_type,))


def _urdf_type_for(type_value):
    """Map a Fusion JointTypes value to a URDF joint type or None."""
    return _FUSION_TO_URDF.get(type_value)


def _origin_array(origin_like):
    """Return a 3-element array from a JointGeometry/JointOrigin/Point."""
    if origin_like is None:
        return None
    for attr in ('origin', 'geometry', 'point'):
        try:
            candidate = getattr(origin_like, attr, None)
            if candidate is None:
                continue
            if callable(candidate):
                candidate = candidate()
            if hasattr(candidate, 'asArray'):
                return list(candidate.asArray())
            if hasattr(candidate, 'origin') and hasattr(candidate.origin, 'asArray'):
                return list(candidate.origin.asArray())
        except Exception:
            continue
    try:
        if hasattr(origin_like, 'asArray'):
            return list(origin_like.asArray())
    except Exception:
        pass
    return None


def _matrix_translation_cm(matrix):
    if matrix is None:
        return None
    try:
        if callable(matrix):
            matrix = matrix()
        translation = getattr(matrix, 'translation', None)
        if translation is None:
            return None
        if callable(translation):
            translation = translation()
        return list(translation.asArray())
    except Exception:
        return None


def _resolve_joint_xyz_cm(joint, is_as_built):
    """Return the joint position in cm in the root frame, or None."""
    if is_as_built:
        return _resolve_asbuilt_xyz_cm(joint)
    return _resolve_standard_xyz_cm(joint)


def _resolve_standard_xyz_cm(joint):
    # Primary path: joint origins relative to each occurrence, composed
    # with the occurrence transform (transform2, retired transform fallback).
    try:
        one = getattr(joint, 'geometryOrOriginOne', None)
        two = getattr(joint, 'geometryOrOriginTwo', None)
        xyz_from_one = _origin_array(one)
        xyz_from_two = _origin_array(two)
        xyz_of_one = utils.occurrence_translation(joint.occurrenceOne)
        matrix_two = utils.occurrence_matrix_array(joint.occurrenceTwo)
        if xyz_from_one is not None and xyz_from_two is not None:
            if xyz_of_one is not None and matrix_two is not None:
                if utils.arrays_close(xyz_from_two, xyz_from_one) or \
                        utils.arrays_close(xyz_from_two, xyz_of_one):
                    return list(xyz_from_two)
                return utils.transform_point(matrix_two, xyz_from_two)
            return list(xyz_from_two)
    except Exception:
        pass

    # Fallback 1: explicit joint-geometry transforms (survive model edits
    # that invalidate the original joint geometry).
    for attr in ('geometryTwoTransform', 'geometryOneTransform'):
        try:
            matrix = getattr(joint, attr, None)
            translation = _matrix_translation_cm(matrix)
            if translation is not None:
                return list(translation)
        except Exception:
            continue

    # Fallback 2: JointOrigin wrapper around geometry.
    try:
        two = getattr(joint, 'geometryOrOriginTwo', None)
        if two is not None and 'JointOrigin' in type(two).__name__:
            data = _origin_array(getattr(two, 'geometry', None))
            if data is not None:
                return list(data)
        data = _origin_array(two)
        if data is not None:
            return list(data)
    except Exception:
        pass
    return None


def _resolve_asbuilt_xyz_cm(joint):
    for attr in ('geometry', 'transform'):
        try:
            candidate = getattr(joint, attr, None)
            if candidate is None:
                continue
            if attr == 'transform':
                translation = _matrix_translation_cm(candidate)
                if translation is not None:
                    return list(translation)
                continue
            data = _origin_array(candidate)
            if data is not None:
                return list(data)
        except Exception:
            continue
    return None
