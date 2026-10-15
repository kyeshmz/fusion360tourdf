# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk, adsk.core, adsk.fusion
import os.path, re
from xml.etree import ElementTree
from xml.dom import minidom
import shutil  # Replaced distutils with shutil
import fileinput
import sys


def sanitize_name(name):
    """Return a ROS/URDF-safe link or joint name.

    Fusion allows spaces, colons, parens, dashes, etc. URDF/ROS
    names should stay in [A-Za-z0-9_]. Collapse runs of invalid
    characters to a single underscore so `base_link (1):1` and
    `base_link__1` converge instead of diverging.
    """
    cleaned = re.sub(r'[^A-Za-z0-9_]+', '_', str(name)).strip('_')
    return cleaned if cleaned else 'link'


def transform_point(matrix_array_16, point_3):
    """Apply a 4x4 homogeneous transform (16-element array) to a 3D point."""
    M = matrix_array_16
    a = point_3
    ex = [M[0], M[4], M[8]]
    ey = [M[1], M[5], M[9]]
    ez = [M[2], M[6], M[10]]
    oo = [M[3], M[7], M[11]]
    return [a[0] * ex[i] + a[1] * ey[i] + a[2] * ez[i] + oo[i] for i in range(3)]


def arrays_close(v1, v2, tol=1e-6):
    """Element-wise closeness check for 3D vectors."""
    try:
        return max(abs(a - b) for a, b in zip(v1, v2)) < tol
    except Exception:
        return False


def occurrence_matrix_array(occ):
    """Return the 16-element transform array for an occurrence.

    Uses ``transform2`` (the current API, ``transform`` was retired in
    July 2025) with a fallback to ``transform`` for very old clients.
    Returns None when neither is available.
    """
    for attr in ('transform2', 'transform'):
        try:
            matrix = getattr(occ, attr, None)
            if matrix is None:
                continue
            if callable(matrix):
                matrix = matrix()
            if hasattr(matrix, 'asArray'):
                return list(matrix.asArray())
            if isinstance(matrix, (list, tuple)) and len(matrix) == 16:
                return list(matrix)
        except Exception:
            continue
    return None


def occurrence_translation(occ):
    """Return the occurrence translation in Fusion internal units (cm)."""
    for attr in ('transform2', 'transform'):
        try:
            matrix = getattr(occ, attr, None)
            if matrix is None:
                continue
            if callable(matrix):
                matrix = matrix()
            translation = getattr(matrix, 'translation', None)
            if translation is None:
                continue
            if callable(translation):
                translation = translation()
            if hasattr(translation, 'asArray'):
                return list(translation.asArray())
        except Exception:
            continue
    matrix_array = occurrence_matrix_array(occ)
    if matrix_array:
        return [matrix_array[3], matrix_array[7], matrix_array[11]]
    return None


def _occurrence_bodies(occ):
    """Return the BRep bodies for an occurrence, via proxy or component."""
    for source in (occ, getattr(occ, 'component', None)):
        if source is None:
            continue
        try:
            bodies = getattr(source, 'bRepBodies', None)
            if bodies is None:
                continue
            if callable(bodies):
                bodies = bodies()
            return bodies
        except Exception:
            continue
    return None


def _force_millimeter_stl(stl_options):
    """Best-effort: force STL export units to millimeters.

    URDF meshes use ``scale 0.001`` (mm -> m). Newer Fusion exposes
    ``STLExportOptions.unitType``; older builds do not. Never fail the
    export because of this hint.
    """
    try:
        if not hasattr(stl_options, 'unitType'):
            return
        distance_units = getattr(adsk.fusion, 'DistanceUnits', None)
        millimeter = None
        if distance_units is not None:
            for attr in ('MillimeterDistanceUnits', 'MillimeterUnits', 'mm'):
                millimeter = getattr(distance_units, attr, None)
                if millimeter is not None:
                    break
        if millimeter is not None:
            stl_options.unitType = millimeter
    except Exception:
        pass

def copy_occs(root):    
    """    
    duplicate all the components
    """    
    def copy_body(allOccs, occs):
        """    
        copy the old occs to new component
        """
        
        bodies = _occurrence_bodies(occs)
        try:
            body_count = bodies.count if bodies is not None else 0
        except Exception:
            body_count = 0
        if not body_count:
            return None
        transform = adsk.core.Matrix3D.create()
        
        # Create new components from occs
        # This support even when a component has some occses. 

        new_occs = allOccs.addNewComponent(transform)  # this create new occs
        if occs.component.name == 'base_link':
            occs.component.name = 'old_component'
            new_occs.component.name = 'base_link'
        else:
            new_occs.component.name = sanitize_name(occs.name)
        new_occs = allOccs.item((allOccs.count-1))
        for i in range(bodies.count):
            body = bodies.item(i)
            body.copyToComponent(new_occs)
        return new_occs
    
    allOccs = root.occurrences
    oldOccs = []
    coppy_list = [occs for occs in allOccs]
    for occs in coppy_list:
        bodies = _occurrence_bodies(occs)
        try:
            count = bodies.count if bodies is not None else 0
        except Exception:
            count = 0
        if count > 0:
            copy_body(allOccs, occs)
            oldOccs.append(occs)

    for occs in oldOccs:
        try:
            occs.component.name = 'old_component'
        except Exception:
            pass


def export_stl(design, save_dir, components):  
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
    import os
    os.makedirs(os.path.join(save_dir, 'meshes'), exist_ok=True)
    scriptDir = os.path.join(save_dir, 'meshes')
    exported = set()
    # export the occurrence one by one in the component to a specified file
    for component in components:
        try:
            allOccus = component.allOccurrences
        except Exception:
            continue
        for occ in allOccus:
            try:
                comp_name = occ.component.name
            except Exception:
                continue
            if 'old_component' in comp_name:
                continue
            safe_name = sanitize_name(comp_name)
            if safe_name in exported:
                continue
            try:
                print(comp_name)
                fileName = os.path.join(scriptDir, safe_name)
                # create stl exportOptions
                stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
                stlExportOptions.sendToPrintUtility = False
                stlExportOptions.isBinaryFormat = True
                # options are .MeshRefinementLow .MeshRefinementMedium .MeshRefinementHigh
                stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                _force_millimeter_stl(stlExportOptions)
                exportMgr.execute(stlExportOptions)
                exported.add(safe_name)
            except Exception:
                print('Component ' + comp_name + ' has something wrong.')


def file_dialog(ui):     
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog' 
    
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
    translation_matrix = [y**2 + z**2, x**2 + z**2, x**2 + y**2,
                         -x*y, -y*z, -x*z]
    return [round(i - mass*t, 6) for i, t in zip(inertia, translation_matrix)]


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
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def copy_package(save_dir, package_dir):
    try:
        # Check if the target directory exists, if not, create it
        import os
        os.makedirs(os.path.join(save_dir, 'launch'), exist_ok=True)
        os.makedirs(os.path.join(save_dir, 'urdf'), exist_ok=True)
        
        # Check if the package directory exists and copy it
        if os.path.exists(package_dir):
            shutil.copytree(package_dir, save_dir, dirs_exist_ok=True)  # dirs_exist_ok=True allows overwriting
        else:
            print(f"Package directory '{package_dir}' does not exist.")
        
    except Exception as e:
        print(f"Error copying package: {e}")


def update_cmakelists(save_dir, package_name):
    import os
    file_name = os.path.join(save_dir, 'CMakeLists.txt')
    if not os.path.exists(file_name):
        return

    for line in fileinput.input(file_name, inplace=True):
        if 'project(fusion2urdf)' in line or 'project(fusion360tourdf)' in line:
            sys.stdout.write("project(" + package_name + ")\n")
        else:
            sys.stdout.write(line)


def update_package_xml(save_dir, package_name):
    import os
    file_name = os.path.join(save_dir, 'package.xml')
    if not os.path.exists(file_name):
        return

    for line in fileinput.input(file_name, inplace=True):
        if '<name>' in line:
            sys.stdout.write("  <name>" + package_name + "</name>\n")
        elif '<description>' in line:
            sys.stdout.write("<description>The " + package_name + " package</description>\n")
        else:
            sys.stdout.write(line)
