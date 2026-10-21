"""Final public combinations with source-literal geometry and motion expectations."""
import importlib
import itertools
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from tests.test_mesh_inputs import MeshBody, mesh_root, supplied, READS, snapshot_design
from tests.test_mesh_geometry import cell_surface, transformed, stl_bytes
from tests.test_fusion_inputs import design, fixed, hinge, occurrence, root
from tests.test_public_motion import native_link, assembly
from tests.test_combined_frames import source_snapshot
from tests.test_public_export_policies import (Y_UP, IDENTITY, mv, mm, rotation, xml_world,
    xml_pose_world, vector, stl_points, scene, source_world, expected_link_pose, BOUNDS, SAMPLES)
from URDF_Exporter.core.Write import write_ros2_package
from URDF_Exporter.utils.mesh_geometry import CancelledError
from tools.verify_loaders import check_geometry, geometry_contract, mesh_path

ENTRY = importlib.import_module('URDF_Exporter.URDF_Exporter')
POLICIES = {'policy_rotation': {'output_rotation_preset': 'y_up_to_z_up'},
            'policy_boxes_dh': {'output_rotation_preset': 'y_up_to_z_up', 'frame_convention': 'dh_common_normal',
                                'collision_method': 'primitive_box'},
            'policy_floor': {'minimum_principal_inertia': .0035}}
MESH_LOWS = {'base_link': (-.04, -.05, -.06), 'caf_1': (.01, .02, .03), 'follower_1': (.05, -.03, .01)}


def pose(r, t=(0, 0, 0)):
    return [[r[3*i+j] for j in range(3)]+[t[i]] for i in range(3)]+[[0, 0, 0, 1]]


def bounds(points):
    return [[f(p[i] for p in points) for i in range(3)] for f in (min, max)]


def mesh_assembly():
    triangles = transformed(cell_surface([(0, 0, 0)]), scale=(2, 3, 5))
    component, _ = mesh_root(transformed(triangles, translation=(-4, -5, -6)))
    native = MeshBody('scan', triangles)
    owners = []
    for path, xyz in (('café:1', (1, 2, 3)), ('follower:1', (5, -3, 1))):
        matrix = [1, 0, 0, xyz[0], 0, 1, 0, xyz[1], 0, 0, 1, xyz[2], 0, 0, 0, 1]
        owner = occurrence(path, matrix=matrix)
        owner.meshBodies = [MeshBody('scan', (), owner=owner, native=native)]
        owners.append(owner)
    reference = occurrence('reference:1')
    drive, follow = [hinge(n, o, None) for n, o in zip(('drive', 'follow'), owners)]
    for joint, extent in ((drive, .8), (follow, .5)):
        joint.jointMotion.rotationLimits.isMinimumValueEnabled = True
        joint.jointMotion.rotationLimits.isMaximumValueEnabled = True
        joint.jointMotion.rotationLimits.minimumValue = -extent
        joint.jointMotion.rotationLimits.maximumValue = extent
    component.allOccurrences = owners+[reference]
    component.allJoints = [drive, follow, fixed('reference_mount', reference, owners[0])]
    component.motionLinks = [native_link(drive, follow, ids=(10, 10), reversed=True)]
    return component


def integrated_oracle(variant):
    from tests.fixtures import oracle, MESH_BOUNDS_MM, material_oracle
    if variant in POLICIES:
        options = POLICIES[variant]
        g = Y_UP if 'output_rotation_preset' in options else IDENTITY
        names = ['base_link', 'arm', 'carriage'] + (['export_output_reference'] if g != IDENTITY else [])
        cases = {}
        for label, sample in zip(('zero', 'moved', 'negative'), SAMPLES):
            visual, collision, visual_poses = {}, {}, {}
            for link, (lo, hi) in BOUNDS.items():
                vertices = [lo, (hi[0], lo[1], lo[2]), (lo[0], hi[1], lo[2]), (lo[0], lo[1], hi[2])]
                transform = lambda p: mv(g, source_world([v*.001 for v in p], link, sample))
                visual[link] = bounds([transform(p) for p in vertices])
                collision[link] = bounds([transform(p) for p in itertools.product(*zip(lo, hi))]) if variant == 'policy_boxes_dh' else visual[link]
                t = mv(g, source_world([0, 0, 0], link, sample))
                axes = [mv(g, source_world(a, link, sample, True)) for a in ((1,0,0), (0,1,0), (0,0,1))]
                visual_poses[link] = pose([axes[j][i] for i in range(3) for j in range(3)], t)
            cases[label] = dict(positions=sample, link_poses={n: expected_link_pose(n, sample, g, variant == 'policy_boxes_dh') for n in names},
                link_kinds={n: 'frame' if n == 'export_output_reference' else 'physical' for n in names},
                bounds=visual, visual_bounds=visual, collision_bounds=collision, visual_poses=visual_poses)
        return cases
    if variant == 'mesh':
        cases = {}
        for label, q in (('zero', 0.), ('moved', .12)):
            rotations = {'base_link': list(Y_UP), 'caf_1': mm(Y_UP, rotation([0, 0, q])),
                         'follower_1': mm(Y_UP, rotation([0, 0, -2.5*q]))}
            poses = {n: pose(r) for n, r in rotations.items()}
            poses.update(reference_1=pose(rotations['caf_1']), export_output_reference=pose(IDENTITY))
            geometry = {n: bounds([mv(rotations[n], p) for p in itertools.product(*[(a, a+d) for a, d in zip(lo, (.02,.03,.05))])])
                        for n, lo in MESH_LOWS.items()}
            cases[label] = dict(positions={'drive': q}, states={'drive': q, 'follow': -2.5*q}, link_poses=poses,
                link_kinds={n: 'physical' if n in rotations else 'frame' for n in poses}, bounds=geometry,
                visual_poses={n: pose(r) for n, r in rotations.items()}, masses=dict.fromkeys(rotations, .06))
        return cases
    if variant == 'surface':
        return {'zero': dict(positions={}, link_poses={'base_link': pose(Y_UP), 'export_output_reference': pose(IDENTITY)},
            link_kinds={'base_link': 'physical', 'export_output_reference': 'frame'},
            bounds={'base_link': [[0,0,0],[.02,0,.03]]}, visual_poses={'base_link': pose(Y_UP)},
            masses={'base_link': 2}, geometry_dimensions={'base_link': 2},
            geometry_triangles={'base_link': {'visual': 1, 'collision': 1}})}
    if variant == 'hull':
        return oracle(False)
    cases = oracle(True)
    values = cases['zero']
    values['link_poses'] = {n: pose(mm(Y_UP, [matrix[i][j] for i in range(3) for j in range(3)]),
        mv(Y_UP, [matrix[i][3] for i in range(3)])) for n, matrix in values['link_poses'].items()}
    values['link_poses']['export_output_reference'] = pose(IDENTITY)
    values['link_kinds'] = {n: 'frame' if n == 'export_output_reference' else 'physical' for n in values['link_poses']}
    values['visual_poses'] = dict.fromkeys(('base_link', 'mount'), pose(Y_UP))
    visual, collision = {}, {}
    for n, (lo, hi) in MESH_BOUNDS_MM.items():
        if n not in ('base_link', 'mount'):
            continue
        visual[n] = bounds([mv(Y_UP, [v*.001 for v in p]) for p in itertools.product(*zip(lo, hi))])
        width = max(b-a for a,b in zip(lo,hi))/2
        outer = [a+math.ceil((b-a)/width)*width for a,b in zip(lo,hi)]
        collision[n] = bounds([mv(Y_UP, [v*.001 for v in p]) for p in itertools.product(*zip(lo,outer))]) if variant == 'materials_cells' else visual[n]
    values.update(bounds=visual, visual_bounds=visual, collision_bounds=collision,
                  collision_poses={n: pose(Y_UP) for n in ('base_link', 'mount')}, materials=material_oracle())
    return cases


def generate_integrated(output, variant):
    from tests.fixtures import model, write_tetrahedron, MESH_BOUNDS_MM, material_assets
    from tests.test_public_export_policies import retained_fixture
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    name = 'fixture_'+variant
    if variant in POLICIES:
        package, _ = retained_fixture(output, name, POLICIES[variant])
    elif variant in ('mesh', 'surface'):
        if variant == 'mesh':
            component = mesh_assembly()
            config = {'mesh_physics': {'density_kg_m3': 2000}, 'motion_links': 'mimic', 'minimum_principal_inertia': .001}
        else:
            component, body = mesh_root([((0,0,0),(2,0,0),(0,3,0))])
            body.isClosed = False
            config = {'mesh_physics': {'bodies': [supplied()]}, 'minimum_principal_inertia': .01}
        config['output_rotation_preset'] = 'y_up_to_z_up'
        fixture, exported = snapshot_design(component)
        before = source_snapshot(fixture)
        package = ENTRY.export_design(fixture, output/(name+'_description'), config=config)
        assert source_snapshot(fixture) == before and not exported
    else:
        joints, inertial = model(variant != 'hull')
        material = variant.startswith('materials_')
        method = 'convex_decomposition' if variant == 'materials_cells' else 'convex_hull'
        config = {'collision_method': method}
        if material:
            config.update(output_rotation_preset='y_up_to_z_up', visual_materials='diffuse-subset')
        if method == 'convex_decomposition':
            config['convex_decomposition'] = {'resolution': 2}
        def meshes(paths):
            for n,p in paths.items():
                lo,hi=MESH_BOUNDS_MM[n]
                if variant == 'hull':
                    p.write_bytes(stl_bytes(transformed(cell_surface([(0,0,0),(1,0,0),(0,1,0)]),
                        scale=((hi[0]-lo[0])/2,(hi[1]-lo[1])/2,hi[2]-lo[2]),translation=lo)))
                else:
                    write_tetrahedron(p,MESH_BOUNDS_MM[n])
        package = write_ros2_package(output/(name+'_description'), name, joints, inertial,
            write_meshes=meshes,
            config=config, visual_assets=material_assets() if material else None)
    (output/(name+'.expected.json')).write_text(json.dumps(integrated_oracle(variant), indent=2)+'\n')
    return package


class IntegratedPoliciesTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name)

    def test_mesh_rotation_floor_alias_frames_mimic_literal_points_mass_tensor_and_limits(self):
        READS.clear()
        package = generate_integrated(self.output, 'mesh')
        robot = ET.parse(package/'urdf/fixture_mesh.urdf').getroot()
        receipt = json.loads((package/'config/robot_data.yaml').read_text())
        self.assertEqual(len(READS), 2)
        self.assertEqual(receipt['source_identifiers']['links']['caf_1'], 'café:1')
        self.assertEqual(receipt['independent_joints'], ['drive'])
        self.assertAlmostEqual(receipt['joints']['drive']['lower_limit'], -.2)
        self.assertAlmostEqual(receipt['joints']['drive']['upper_limit'], .2)
        self.assertAlmostEqual(receipt['joints']['follow']['lower_limit'], -.5)
        self.assertEqual(receipt['initial_positions'], {'drive': 0., 'follow': 0.})
        self.assertEqual(len(robot.findall('ros2_control/joint/command_interface')), 1)
        for q in (0., .12, -.17):
            sample = {'drive': q, 'follow': -2.5*q}
            for n, lo in MESH_LOWS.items():
                r = mm(Y_UP, rotation([0,0,0 if n == 'base_link' else sample['drive' if n == 'caf_1' else 'follow']]))
                link = robot.find("link[@name='%s']" % n)
                for kind in ('visual', 'collision'):
                    element = link.find(kind)
                    actual = [xml_pose_world(robot,n,element.find('origin'),[v*.001 for v in p],sample) for p in stl_points(package/'meshes'/(n+'.stl'))]
                    wanted = [mv(r,p) for p in itertools.product(*[(a,a+d) for a,d in zip(lo,(.02,.03,.05))])]
                    self.assertEqual(len(actual),len(wanted))
                    for a in actual:
                        self.assertLess(min(math.dist(a,b) for b in wanted),1e-10)
                self.assertAlmostEqual(float(link.find('inertial/mass').get('value')), .06)
                self.assertEqual([float(link.find('inertial/inertia').get(k)) for k in ('ixx','iyy','izz')], [.001]*3)
                self.assertEqual([float(link.find('inertial/inertia').get(k)) for k in ('ixy','ixz','iyz')], [0.]*3)
                center = [a+d/2 for a,d in zip(lo,(.02,.03,.05))]
                for a,b in zip(xml_world(robot,n,vector(link.find('inertial/origin'),'xyz'),sample),mv(r,center)):
                    self.assertAlmostEqual(a,b,delta=1e-10)
        for n in ('reference_1','export_output_reference'):
            self.assertEqual(list(robot.find("link[@name='%s']" % n)), [])

    def test_obj_rotation_all_hull_cell_parts_resources_and_original_visual_bytes(self):
        from tests.fixtures import material_assets
        assets = material_assets()
        for variant in ('materials_hull','materials_cells'):
            package = generate_integrated(self.output/variant, variant)
            robot = ET.parse(package/('urdf/fixture_'+variant+'.urdf')).getroot()
            receipt = json.loads((package/'config/robot_data.yaml').read_text())
            for bundle in assets.values():
                for relative,data in bundle['files'].items(): self.assertEqual((package/relative).read_bytes(),data)
            key = 'convex_hull' if variant.endswith('hull') else 'collision_decomposition'
            for name, entry in receipt[key]['links'].items():
                link = robot.find("link[@name='%s']" % name)
                self.assertEqual(len(link.findall('collision')),len(entry['assets']))
                for collision,relative in zip(link.findall('collision'),entry['assets']):
                    self.assertEqual(collision.find('origin').attrib,link.find('visual/origin').attrib)
                    self.assertEqual(Path(mesh_path(collision.find('geometry/mesh').get('filename'),package,package/'urdf/model.urdf')),package/relative)
                    for p in stl_points(package/relative):
                        actual=xml_pose_world(robot,name,collision.find('origin'),[v*.001 for v in p],{})
                        for a,b in zip(actual,mv(Y_UP,[v*.001 for v in p])): self.assertAlmostEqual(a,b,delta=1e-10)
            self.assertTrue(all(len(d['surfaces'])==4 and len(d['images'])==3 for d in receipt['visual_assets'].values()))

    def test_generated_reference_casefold_link_and_joint_namespace_preserves_source_ids(self):
        from URDF_Exporter.utils.names import ExportedJoints
        for source_name in ('Export_Output_Reference','EXPORT_OUTPUT_TO_BASE_LINK'):
            joints, inertial = scene()
            inertial[source_name] = inertial.pop('carriage')
            inertial[source_name] = {'kind':'frame','source_path':'RAW/'+source_name}
            joints['slide']['child'] = source_name
            joint_name = 'Export_Output_To_Base_Link' if source_name == 'Export_Output_Reference' else 'source_slide'
            joints[joint_name] = joints.pop('slide')
            identifiers = {'links':{source_name:'RAW/'+source_name},'joints':{joint_name:'CAD Joint'}}
            joints = ExportedJoints(joints,identifiers)
            package = write_ros2_package(self.output/source_name/'same_description','same',joints,inertial,
                write_meshes=lambda paths: [__import__('tests.fixtures',fromlist=['write_tetrahedron']).write_tetrahedron(p,((0,0,0),(10,20,30))) for p in paths.values()],
                config={'output_rotation_preset':'y_up_to_z_up'})
            receipt = json.loads((package/'config/robot_data.yaml').read_text())
            names = list(receipt['links'])+list(receipt['joints'])
            self.assertEqual(len(names),len({n.casefold() for n in names}))
            self.assertEqual(receipt['source_identifiers'],identifiers)
            ref = receipt['root_frame']['output_reference_link']
            self.assertIn('Fixed Frame: '+ref,(package/'viewers/display.rviz').read_text())
            self.assertEqual(receipt['link_sources'][source_name]['kind'],'frame')

    def test_native_compound_dh_rejects_before_geometry_publication(self):
        component,_,_ = assembly('cylindrical')
        fixture,exported = design(component)
        with self.assertRaisesRegex(ValueError,'dh_common_normal.*compound'):
            ENTRY.export_design(fixture,self.output/'bad_description',config={'generalized_joints':'decompose','frame_convention':'dh_common_normal'})
        self.assertFalse((self.output/'bad_description').exists())
        self.assertEqual(exported,[])

    def test_generalized_mimic_rotation_floor_preserve_full_helper_and_endpoint_poses(self):
        from tests.fixtures import motion_oracle
        from tests.test_public_export_policies import pose_matrix
        component,joint,_ = assembly('cylindrical',reverse=True)
        component.motionLinks = [native_link(joint,reversed=True)]
        fixture,exported = snapshot_design(component)
        before = source_snapshot(fixture)
        package = ENTRY.export_design(fixture,self.output/'compound_description',config={
            'generalized_joints':'decompose','motion_links':'mimic',
            'output_rotation_preset':'y_up_to_z_up','minimum_principal_inertia':.1})
        robot = ET.parse(package/'urdf/compound.urdf').getroot()
        for values in motion_oracle('mimic').values():
            for name,matrix in values['link_poses'].items():
                wanted = pose(mm(Y_UP,[matrix[i][j] for i in range(3) for j in range(3)]),
                              mv(Y_UP,[matrix[i][3] for i in range(3)]))
                actual = pose_matrix(robot,name,values['states'])
                for a,b in zip(sum(actual,[]),sum(wanted,[])): self.assertAlmostEqual(a,b,delta=1e-10)
        self.assertEqual(source_snapshot(fixture),before)
        self.assertEqual(exported,['base','body'])
        receipt=json.loads((package/'config/robot_data.yaml').read_text())
        self.assertEqual(receipt['independent_joints'],['fusion_helper_compound__rotate'])

    def test_policy_preserves_nonzero_common_initial_states_and_disabled_package_bytes(self):
        from tests.test_package import model,write_meshes
        joints,inertial=model()
        for options,folder in ((None,'plain'),({'output_rotation':None,'minimum_principal_inertia':None},'disabled')):
            write_ros2_package(self.output/folder/'same_description','same',joints,inertial,write_meshes=write_meshes,config=options)
        a,b=[self.output/f/'same_description' for f in ('plain','disabled')]
        self.assertEqual({str(p.relative_to(a)):p.read_bytes() for p in a.rglob('*') if p.is_file()},
                         {str(p.relative_to(b)):p.read_bytes() for p in b.rglob('*') if p.is_file()})
        joints,inertial=scene()
        joints['turn'].update(lower_limit=.2,upper_limit=.5)
        joints['slide'].update(lower_limit=.04,upper_limit=.1,mimic={'joint':'turn','multiplier':.2,'offset':0.})
        package=write_ros2_package(self.output/'initial_description','initial',joints,inertial,
            write_meshes=lambda paths:[__import__('tests.fixtures',fromlist=['write_tetrahedron']).write_tetrahedron(p,BOUNDS[n]) for n,p in paths.items()],
            config={'output_rotation_preset':'y_up_to_z_up','minimum_principal_inertia':.004})
        receipt=json.loads((package/'config/robot_data.yaml').read_text())
        self.assertAlmostEqual(receipt['initial_positions']['turn'],.2)
        self.assertAlmostEqual(receipt['initial_positions']['slide'],.04)
        robot=ET.parse(package/'urdf/initial.urdf').getroot()
        for n,expected in receipt['initial_positions'].items():
            self.assertAlmostEqual(float(robot.find("ros2_control/joint[@name='%s']/state_interface/param" % n).text),expected)

    def test_supplied_mesh_physics_accepts_validated_writer_visual_bundle(self):
        from URDF_Exporter.utils import utils
        from URDF_Exporter.core import Link
        from tests.fixtures import material_assets
        component,_=mesh_root(cell_surface([(0,0,0)]))
        fixture,_=snapshot_design(component)
        config={'mesh_physics':{'bodies':[supplied()]},'visual_materials':'diffuse-subset','output_rotation_preset':'y_up_to_z_up'}
        sources=utils.link_sources(component,config=config)
        inertial,_=Link.make_inertial_dict(sources,'ok')
        assets=material_assets()['base_link']
        package=write_ros2_package(self.output/'bundle_description','bundle',{},inertial,
            write_meshes=lambda paths:utils.export_stl(fixture,paths,config=config,sources=sources),config=config,
            visual_assets={'base_link':assets})
        receipt=json.loads((package/'config/robot_data.yaml').read_text())
        self.assertEqual(receipt['mesh_physics']['base_link'][0]['policy'],'caller-supplied-central-SI-physics')
        for relative,data in assets['files'].items(): self.assertEqual((package/relative).read_bytes(),data)

    def test_collision_floor_receipts_do_not_claim_unchanged_physics(self):
        from tests.fixtures import model,write_tetrahedron,MESH_BOUNDS_MM
        for method,key in (('convex_hull','convex_hull'),('convex_decomposition','collision_decomposition')):
            joints,inertial=model(True)
            package=write_ros2_package(self.output/method/'floor_description','floor',joints,inertial,
                write_meshes=lambda paths:[write_tetrahedron(p,MESH_BOUNDS_MM[n]) for n,p in paths.items()],
                config={'collision_method':method,'convex_decomposition':{'resolution':2},
                        'output_rotation_preset':'y_up_to_z_up','minimum_principal_inertia':.1})
            receipt=json.loads((package/'config/robot_data.yaml').read_text())
            self.assertEqual(receipt[key]['physics_policy'],'mass-unchanged; output-inertia-explicit-spectral-floor')
            self.assertFalse(any('visual and CAD physics are unchanged' in w for w in receipt['warnings']))
            robot=ET.parse(package/'urdf/floor.urdf').getroot()
            for n in inertial:
                self.assertEqual(float(robot.find("link[@name='%s']/inertial/mass" % n).get('value')),inertial[n]['mass'])
                self.assertTrue(receipt['export_policies']['receipts']['inertia'][n]['changed'])

    def test_registry_oracles_agree_with_serialized_all_link_poses(self):
        from tests.fixtures import OPTIONAL_VARIANTS,generate_fixture,fixture_oracle
        from tests.test_public_export_policies import pose_matrix
        for variant in OPTIONAL_VARIANTS[4:]:
            package=generate_fixture(self.output/variant,variant)
            robot=ET.parse(package/('urdf/fixture_'+variant+'.urdf')).getroot()
            if variant == 'hull':
                from tests.fixtures import MESH_BOUNDS_MM
                receipt=json.loads((package/'config/robot_data.yaml').read_text())
                for n,entry in receipt['convex_hull']['links'].items():
                    lo,hi=MESH_BOUNDS_MM[n]
                    box_volume=math.prod((b-a)*.001 for a,b in zip(lo,hi))
                    self.assertAlmostEqual(entry['hull_volume_m3'],.875*box_volume,delta=1e-15)
                    self.assertGreater(entry['hull_volume_m3'],.75*box_volume)
                    self.assertLess(entry['hull_volume_m3'],box_volume)
            for values in fixture_oracle(variant).values():
                self.assertEqual({e.get('name') for e in robot.findall('link')},set(values['link_poses']))
                for name,wanted in values['link_poses'].items():
                    actual=pose_matrix(robot,name,values.get('states',values['positions']))
                    for a,b in zip(sum(actual,[]),sum(wanted,[])): self.assertAlmostEqual(a,b,delta=1e-10)

    def test_mesh_native_face_material_combination_has_explicit_diagnostic(self):
        component,_ = mesh_root(cell_surface([(0,0,0)]))
        fixture,exported = design(component)
        with self.assertRaisesRegex(ValueError,'MESH_BODY_VISUAL_MATERIALS_UNSUPPORTED'):
            ENTRY.export_design(fixture,self.output/'bad_description',config={'mesh_physics':{'density_kg_m3':1000},'visual_materials':'strict'})
        self.assertEqual(exported,[])
        self.assertEqual(list(self.output.iterdir()),[])

    def test_explicit_surface_contract_positive_area_and_strict_negative_controls(self):
        vertices = [(0,0,0),(2,0,0),(0,3,0)]
        check_geometry(vertices,[(0,1,2)],dimension=2,expected_faces=1)
        dimensions,counts = geometry_contract(integrated_oracle('surface')['zero'],{'base_link'})
        self.assertEqual(dimensions,{'base_link':2})
        self.assertEqual(counts['base_link']['visual'],1)
        for points,faces,options in ((vertices,[(0,1,2)],{}), (vertices,[],{'dimension':2}),
                (vertices,[(0,0,2)],{'dimension':2}), ([(0,0,0),(math.nan,0,0),(0,3,0)],[(0,1,2)],{'dimension':2}),
                (vertices,[(0,1,2)],{'dimension':2,'expected_faces':2})):
            with self.assertRaises(AssertionError): check_geometry(points,faces,**options)
        with self.assertRaisesRegex(AssertionError,'solid dimension'):
            check_geometry([(0,0,0),(1,0,1),(0,1,1),(1,1,2)],[(0,1,2),(1,3,2),(2,1,0),(2,3,1)])
        with self.assertRaises(AssertionError): geometry_contract({'geometry_dimensions':{'base_link':2}},{'base_link'})
        package = generate_integrated(self.output,'surface')
        with self.assertRaisesRegex(AssertionError,'Missing or external mesh'):
            mesh_path('package://fixture_surface_description/meshes/gone.stl',package,package/'urdf/model.urdf')

    def test_combined_cancellation_and_late_failure_preserve_source_and_prior_output(self):
        component=mesh_assembly()
        fixture,_=snapshot_design(component)
        before=source_snapshot(fixture)
        prior=self.output/'prior.txt'; prior.write_bytes(b'prior')
        config={'mesh_physics':{'density_kg_m3':2000},'motion_links':'mimic','output_rotation_preset':'y_up_to_z_up','collision_method':'convex_hull'}
        with self.assertRaises(CancelledError):
            ENTRY.export_design(fixture,self.output/'cancel_description',config=config,cancelled=lambda:True)
        with patch('URDF_Exporter.core.Write.hull_stl',side_effect=RuntimeError('late hull failure')):
            with self.assertRaisesRegex(RuntimeError,'late hull failure'):
                ENTRY.export_design(fixture,self.output/'failed_description',config=config)
        self.assertEqual(source_snapshot(fixture),before)
        self.assertEqual(prior.read_bytes(),b'prior')
        self.assertEqual(list(self.output.iterdir()),[prior])


if __name__ == '__main__':
    unittest.main()
