# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:11:28 2019

@author: syuntoku
"""

import adsk
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils
from ..utils import transforms as tf
from ..utils import materials as matlib

class Link:

    def __init__(self, name, xyz, center_of_mass, repo, mass, inertia_tensor,
                 rpy=None, color_rgba=None):
        """
        Parameters
        ----------
        name: str
            name of the link
        xyz: [x, y, z]
            coordinate for the visual and collision
        center_of_mass: [x, y, z]
            coordinate for the center of mass
        link_xml: str
            generated xml describing about the link
        repo: str
            the name of the repository to save the xml file
        mass: float
            mass of the link
        inertia_tensor: [ixx, iyy, izz, ixy, iyz, ixz]
            tensor of the inertia
        """
        self.name = name
        # xyz for visual
        self.xyz = [-_ for _ in xyz]  # reverse the sign of xyz
        # xyz for center of mass
        self.center_of_mass = center_of_mass
        self.rpy = list(rpy) if rpy is not None else [0, 0, 0]
        self.color_rgba = color_rgba
        self.link_xml = None
        self.repo = repo
        self.mass = mass
        self.inertia_tensor = inertia_tensor
        
    def make_link_xml(self, with_collision_mesh=True):
        """
        Generate the link_xml and hold it by self.link_xml.
        """
        
        link = Element('link')
        link.attrib = {'name':self.name}
        
        #inertial
        inertial = SubElement(link, 'inertial')
        origin_i = SubElement(inertial, 'origin')
        origin_i.attrib = {'xyz':' '.join([str(_) for _ in self.center_of_mass]), 'rpy':'0 0 0'}       
        mass = SubElement(inertial, 'mass')
        mass.attrib = {'value': tf.format_float(self.mass)}
        inertia = SubElement(inertial, 'inertia')
        inertia.attrib = \
            {'ixx': tf.format_float(self.inertia_tensor[0]), 'iyy': tf.format_float(self.inertia_tensor[1]),
             'izz': tf.format_float(self.inertia_tensor[2]), 'ixy': tf.format_float(self.inertia_tensor[3]),
             'iyz': tf.format_float(self.inertia_tensor[4]), 'ixz': tf.format_float(self.inertia_tensor[5])}        
        
        # visual
        visual = SubElement(link, 'visual')
        origin_v = SubElement(visual, 'origin')
        origin_v.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]),
                           'rpy': tf.format_vector(self.rpy or [0, 0, 0])}
        geometry_v = SubElement(visual, 'geometry')
        mesh_v = SubElement(geometry_v, 'mesh')
        mesh_v.attrib = {'filename':'package://' + self.repo + self.name + '.stl','scale':'0.001 0.001 0.001'}
        if self.color_rgba is not None:
            material = SubElement(visual, 'material')
            material.attrib = {'name': matlib.material_name_for(self.name)}
            color = SubElement(material, 'color')
            color.attrib = {'rgba': matlib.rgba_to_urdf_string(self.color_rgba)}
        else:
            material = SubElement(visual, 'material')
            material.attrib = {'name':'silver'}
        
        # collision (mesh default; Write.py replaces with a primitive box
        # when collision_method=primitive_box and a bounding box exists).
        if with_collision_mesh:
            collision = SubElement(link, 'collision')
            origin_c = SubElement(collision, 'origin')
            origin_c.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]),
                               'rpy': tf.format_vector(self.rpy or [0, 0, 0])}
            geometry_c = SubElement(collision, 'geometry')
            mesh_c = SubElement(geometry_c, 'mesh')
            mesh_c.attrib = {'filename':'package://' + self.repo + self.name + '.stl','scale':'0.001 0.001 0.001'}

        # print("\n".join(utils.prettify(link).split("\n")[1:]))
        self.link_xml = "\n".join(utils.prettify(link).split("\n")[1:])


def make_inertial_dict(root, msg):
    """
    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status

    Returns
    ----------
    inertial_dict: {name:{mass, inertia, center_of_mass}}

    msg: str
        Tell the status
    """
    # Walk all occurrences (global context) so nested assemblies flatten to
    # leaf links instead of being rejected (upstream "nested components are
    # not supported" limit). Hidden occurrences (light bulb off) are skipped
    # like newtonjeri's exporter, letting users select the export set.
    inertial_dict = {}

    for occs in utils.iter_link_occurrences(root):
        # Skip the root component.
        occs_dict = {}
        try:
            prop = occs.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
        except Exception:
            continue

        occs_dict['name'] = utils.sanitize_name(occs.name)

        mass = prop.mass  # kg
        occs_dict['mass'] = mass
        try:
            center_of_mass = [_/100.0 for _ in prop.centerOfMass.asArray()] ## cm to m
        except Exception:
            center_of_mass = [0.0, 0.0, 0.0]
        occs_dict['center_of_mass'] = center_of_mass

        # https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-ce341ee6-4490-11e5-b25b-f8b156d7cd97
        try:
            (_, xx, yy, zz, xy, yz, xz) = prop.getXYZMomentsOfInertia()
            moment_inertia_world = [_ / 10000.0 for _ in [xx, yy, zz, xy, yz, xz] ] ## kg / cm^2 -> kg/m^2
            occs_dict['inertia'] = utils.origin2center_of_mass(moment_inertia_world, center_of_mass, mass)
        except Exception:
            occs_dict['inertia'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Per-link appearance color (None -> global silver fallback).
        try:
            occs_dict['color_rgba'] = matlib.occurrence_rgba(occs)
        except Exception:
            occs_dict['color_rgba'] = None
        # Bounding box for optional primitive-box collision.
        try:
            occs_dict['bbox_center_m'], occs_dict['bbox_size_m'] = _bbox_m(occs)
        except Exception:
            occs_dict['bbox_center_m'] = None
            occs_dict['bbox_size_m'] = None

        try:
            component_name = occs.component.name
        except Exception:
            component_name = ''
        if component_name == 'base_link':
            inertial_dict['base_link'] = occs_dict
        else:
            inertial_dict[utils.sanitize_name(occs.name)] = occs_dict

    ensure_base_link(root, inertial_dict)
    return inertial_dict, msg


def ensure_base_link(root, inertial_dict):
    """Synthesize base_link from design-root bodies when missing.

    Imported CAD often keeps the chassis as loose bodies under the design
    root with joints referencing the root (Adriaeik's synthetic-root case).
    Upstream errored out ("There is no base_link"); instead build it from
    the root component's own bodies when present. Returns True when
    base_link exists afterwards.
    """
    if 'base_link' in inertial_dict:
        return True
    try:
        bodies = getattr(root, 'bRepBodies', None)
        count = bodies.count if bodies is not None else 0
    except Exception:
        count = 0
    if not count:
        return False
    try:
        prop = root.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
    except Exception:
        return False
    try:
        center_of_mass = [_ / 100.0 for _ in prop.centerOfMass.asArray()]
        (_, xx, yy, zz, xy, yz, xz) = prop.getXYZMomentsOfInertia()
        moment_world = [_ / 10000.0 for _ in [xx, yy, zz, xy, yz, xz]]
        inertia = utils.origin2center_of_mass(moment_world, center_of_mass, prop.mass)
    except Exception:
        return False
    inertial_dict['base_link'] = {
        'name': 'base_link',
        'mass': prop.mass,
        'center_of_mass': center_of_mass,
        'inertia': inertia,
        'color_rgba': None,
        'bbox_center_m': None,
        'bbox_size_m': None,
    }
    return True


def _bbox_m(occ):
    """Return (center_m, size_m) from an occurrence bounding box, or (None, None)."""
    box = None
    for source in (occ, getattr(occ, 'component', None)):
        try:
            candidate = getattr(source, 'boundingBox', None)
            if candidate is None:
                continue
            if callable(candidate):
                candidate = candidate()
            box = candidate
            break
        except Exception:
            continue
    if box is None:
        return None, None
    try:
        lo = box.minPoint
        hi = box.maxPoint
        lo_xyz = [lo.x / 100.0, lo.y / 100.0, lo.z / 100.0]
        hi_xyz = [hi.x / 100.0, hi.y / 100.0, hi.z / 100.0]
        center = [(a + b) / 2.0 for a, b in zip(lo_xyz, hi_xyz)]
        size = [max(0.0, b - a) for a, b in zip(lo_xyz, hi_xyz)]
        return center, size
    except Exception:
        pass
    # Point3D.asArray fallback (values in cm).
    try:
        lo_arr = [v / 100.0 for v in box.minPoint.asArray()]
        hi_arr = [v / 100.0 for v in box.maxPoint.asArray()]
        center = [(a + b) / 2.0 for a, b in zip(lo_arr, hi_arr)]
        size = [max(0.0, b - a) for a, b in zip(lo_arr, hi_arr)]
        return center, size
    except Exception:
        return None, None