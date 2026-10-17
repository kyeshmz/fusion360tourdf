from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

from tests.test_fusion_inputs import Body, occurrence, root
from tests.test_package import write_meshes
from URDF_Exporter.core import Link, Write
from URDF_Exporter.utils import materials, utils


class Color:
    objectType = 'adsk::core::Color'

    def __init__(self, red, green, blue, opacity=255):
        self.red, self.green, self.blue, self.opacity = red, green, blue, opacity


class ColorProperty:
    objectType = 'adsk::core::ColorProperty'

    def __init__(self, value, texture=False, multiple=False):
        self.value = value
        self.hasConnectedTexture = texture
        self.hasMultipleValues = multiple


class Properties:
    def __init__(self, properties):
        self.properties = properties

    def itemById(self, identifier):
        return self.properties.get(identifier)


class LegacyAppearance:
    objectType = 'adsk::core::Appearance'

    def __init__(self, properties):
        self.appearanceProperties = Properties(properties)


class ModernAppearance(LegacyAppearance):
    def __init__(self, color, properties=None):
        super().__init__(properties or {})
        self.color = color


class Material:
    objectType = 'adsk::core::Material'

    def __init__(self, appearance):
        self.appearance = appearance


def selected_source(body_appearance, override=None):
    body = Body('selected', 0)
    body.appearance = body_appearance
    owner = occurrence('base:1', [body], component='base_link')
    owner.appearance = override
    owner.component.material = Material(ModernAppearance(Color(255, 0, 0)))
    return utils.link_sources(root([owner]))['base_link']


class FusionMaterialsTest(unittest.TestCase):
    def test_supported_legacy_shader_properties_preserve_byte_rgba(self):
        for identifier in ('opaque_albedo', 'metal_f0', 'transparent_color', 'generic_diffuse'):
            with self.subTest(identifier=identifier):
                appearance = LegacyAppearance({identifier: ColorProperty(Color(1, 128, 255, 128))})
                self.assertEqual(materials.appearance_to_rgba(appearance),
                                 (0.00392156862745098, 0.5019607843137255,
                                  1.0, 0.5019607843137255))

    def test_modern_color_preserves_small_bytes_and_opacity(self):
        self.assertEqual(materials.appearance_to_rgba(ModernAppearance(Color(1, 128, 255, 128))),
                         (0.00392156862745098, 0.5019607843137255,
                          1.0, 0.5019607843137255))
        self.assertEqual(materials.appearance_to_rgba(ModernAppearance(Color(1, 1, 1, 1))),
                         (0.00392156862745098,) * 4)
        self.assertEqual(materials.appearance_to_rgba(ModernAppearance(Color(0, 0, 0, 0))),
                         (0.0, 0.0, 0.0, 0.0))

    def test_modern_primary_color_precedes_legacy_and_compatibility(self):
        appearance = ModernAppearance(Color(255, 0, 0),
                                      {'opaque_albedo': ColorProperty(Color(0, 255, 0))})
        appearance.diffuse = SimpleNamespace(red=0, green=0, blue=255, opacity=1)
        self.assertEqual(materials.appearance_to_rgba(appearance), (1.0, 0.0, 0.0, 1.0))
        legacy = appearance.appearanceProperties.itemById('opaque_albedo')
        legacy.hasConnectedTexture = True
        self.assertEqual(materials.appearance_to_rgba(appearance), (1.0, 0.0, 0.0, 1.0))
        appearance.color = None
        self.assertIsNone(materials.appearance_to_rgba(appearance))
        legacy.hasConnectedTexture = False
        self.assertEqual(materials.appearance_to_rgba(appearance), (0.0, 1.0, 0.0, 1.0))

    def test_native_color_in_compatibility_slot_is_still_bytes(self):
        self.assertEqual(materials.appearance_to_rgba(SimpleNamespace(diffuse=Color(1, 1, 1, 1))),
                         (0.00392156862745098,) * 4)
        self.assertEqual(materials.appearance_to_rgba(Color(255, 0, 0)),
                         (1.0, 0.0, 0.0, 1.0))

    def test_compatibility_colors_use_one_rgb_scale_without_rounding(self):
        for slot in ('diffuse', 'baseColor', 'color'):
            with self.subTest(slot=slot):
                color = SimpleNamespace(red=1, green=128, blue=255, opacity=0.25)
                self.assertEqual(materials.appearance_to_rgba(SimpleNamespace(**{slot: color})),
                                 (0.00392156862745098, 0.5019607843137255, 1.0, 0.25))
        normalized = SimpleNamespace(red=0.1, green=0.2, blue=0.3333333333333,
                                     alpha=0.123456789)
        self.assertEqual(materials.appearance_to_rgba(SimpleNamespace(diffuse=normalized)),
                         (0.1, 0.2, 0.3333333333333, 0.123456789))

    def test_invalid_native_channels_and_opacity_do_not_become_colors(self):
        self.assertEqual(materials.appearance_to_rgba(ModernAppearance(Color(255, 0, 0))),
                         (1.0, 0.0, 0.0, 1.0))
        for channel in ('red', 'green', 'blue', 'opacity'):
            for invalid in (None, 'invalid', -1, 256, float('nan'), float('inf'), -float('inf')):
                with self.subTest(channel=channel, invalid=invalid):
                    color = Color(255, 128, 0)
                    setattr(color, channel, invalid)
                    self.assertIsNone(materials.appearance_to_rgba(ModernAppearance(color)))
            color = Color(255, 128, 0)
            delattr(color, channel)
            self.assertIsNone(materials.appearance_to_rgba(ModernAppearance(color)))

    def test_invalid_compatibility_colors_do_not_get_clamped_or_default_alpha(self):
        self.assertEqual(materials.appearance_to_rgba(SimpleNamespace(
            diffuse=SimpleNamespace(red=0.1, green=0.2, blue=0.3))), (0.1, 0.2, 0.3, 1.0))
        for channel in ('red', 'green', 'blue', 'opacity'):
            for invalid in ('invalid', -1, 256, float('nan'), float('inf')):
                with self.subTest(channel=channel, invalid=invalid):
                    color = SimpleNamespace(red=0.1, green=0.2, blue=0.3, opacity=1)
                    setattr(color, channel, invalid)
                    self.assertIsNone(materials.appearance_to_rgba(SimpleNamespace(diffuse=color)))

    def test_unsupported_properties_textures_and_values_return_fallback_signal(self):
        valid = ColorProperty(Color(255, 0, 0))
        self.assertEqual(materials.appearance_to_rgba(LegacyAppearance({'opaque_albedo': valid})),
                         (1.0, 0.0, 0.0, 1.0))
        unsupported = (
            {},
            {'unknown_shader_color': valid},
            {'opaque_albedo': ColorProperty(None, texture=True)},
            {'opaque_albedo': ColorProperty(Color(255, 0, 0), texture=True)},
            {'opaque_albedo': ColorProperty(Color(255, 0, 0), multiple=True)},
            {'opaque_albedo': SimpleNamespace(objectType='adsk::core::ColorProperty',
                                              value=Color(255, 0, 0))},
            {'opaque_albedo': SimpleNamespace(objectType='adsk::core::FloatProperty', value=255)},
            {'opaque_albedo': ColorProperty(SimpleNamespace(
                objectType='adsk::core::Texture', red=255, green=0, blue=0, opacity=255))},
            {'opaque_albedo': ColorProperty(SimpleNamespace(red=255, green=0, blue=0, opacity=255))},
            {'opaque_albedo': ColorProperty('not a color')},
            {'opaque_albedo': ColorProperty(None, texture=True), 'generic_diffuse': valid},
        )
        for properties in unsupported:
            with self.subTest(properties=properties):
                appearance = LegacyAppearance(properties)
                appearance.diffuse = SimpleNamespace(red=1, green=0, blue=0)
                self.assertIsNone(materials.appearance_to_rgba(appearance))
        self.assertIsNone(materials.appearance_to_rgba(None))

    def test_material_appearance_and_override_order(self):
        occurrence = SimpleNamespace(
            appearance=ModernAppearance(Color(255, 0, 0)),
            material=Material(LegacyAppearance({'metal_f0': ColorProperty(Color(0, 255, 0))})),
            physicalMaterial=Material(ModernAppearance(Color(0, 0, 255))),
            component=SimpleNamespace(
                appearance=ModernAppearance(Color(255, 255, 0)),
                material=Material(ModernAppearance(Color(255, 0, 255))),
                physicalMaterial=Material(ModernAppearance(Color(0, 255, 255)))))
        self.assertEqual(materials.occurrence_rgba(occurrence), (1.0, 0.0, 0.0, 1.0))
        occurrence.appearance = None
        self.assertEqual(materials.occurrence_rgba(occurrence), (0.0, 1.0, 0.0, 1.0))
        occurrence.material = None
        self.assertEqual(materials.occurrence_rgba(occurrence), (0.0, 0.0, 1.0, 1.0))
        occurrence.physicalMaterial = None
        self.assertEqual(materials.occurrence_rgba(occurrence), (1.0, 1.0, 0.0, 1.0))
        occurrence.component.appearance = None
        self.assertEqual(materials.occurrence_rgba(occurrence), (1.0, 0.0, 1.0, 1.0))
        occurrence.component.material = None
        self.assertEqual(materials.occurrence_rgba(occurrence), (0.0, 1.0, 1.0, 1.0))
        occurrence.component.physicalMaterial = None
        self.assertIsNone(materials.occurrence_rgba(occurrence))
        self.assertIsNone(materials.occurrence_rgba(None))

    def test_unavailable_native_getters_return_fallback_signal(self):
        class BrokenProperties:
            def itemById(self, identifier):
                raise RuntimeError('Unavailable shader property')

        class BrokenOccurrence:
            @property
            def appearance(self):
                raise RuntimeError('Unavailable appearance')

        occurrence = BrokenOccurrence()
        occurrence.material = Material(LegacyAppearance({}))
        occurrence.material.appearance.appearanceProperties = BrokenProperties()
        occurrence.component = SimpleNamespace(appearance=ModernAppearance(Color(0, 255, 0)))
        self.assertEqual(materials.occurrence_rgba(occurrence.component), (0.0, 1.0, 0.0, 1.0))
        self.assertIsNone(materials.occurrence_rgba(occurrence))
        self.assertIsNone(materials.occurrence_rgba(SimpleNamespace(
            appearance=None, material=occurrence.material, component=occurrence.component)))

    def test_generated_urdf_uses_native_color_or_silver_for_unsupported_shader(self):
        cases = (
            (LegacyAppearance({'transparent_color': ColorProperty(Color(1, 128, 255, 128))}),
             (0.00392156862745098, 0.5019607843137255, 1.0, 0.5019607843137255)),
            (LegacyAppearance({'opaque_albedo': ColorProperty(None, texture=True)}),
             (0.7, 0.7, 0.7, 1.0)),
        )
        with tempfile.TemporaryDirectory(prefix='fusion-materials-') as temporary:
            for index, (appearance, expected) in enumerate(cases):
                inertial = {'base_link': {'mass': 1, 'center_of_mass': [0, 0, 0],
                                          'inertia': [1, 1, 1, 0, 0, 0],
                                          'color_rgba': materials.occurrence_rgba(
                                              SimpleNamespace(appearance=appearance))}}
                package = Write.write_ros2_package(
                    Path(temporary) / ('material_%s_description' % index), 'material', {}, inertial,
                    write_meshes=write_meshes)
                robot = ET.parse(package / 'urdf/material.xacro')
                color = robot.find('link/visual/material/color')
                self.assertEqual(tuple(float(value) for value in color.get('rgba').split()), expected)

    def test_parser_and_writer_import_without_adsk(self):
        result = subprocess.run([sys.executable, '-c', '''
import builtins
original_import = builtins.__import__
def no_adsk(name, *args, **kwargs):
    if name == 'adsk' or name.startswith('adsk.'):
        raise AssertionError('Pure material parser and writer must not import adsk')
    return original_import(name, *args, **kwargs)
builtins.__import__ = no_adsk
from URDF_Exporter.utils import materials
from URDF_Exporter.core import Write
from types import SimpleNamespace
color = SimpleNamespace(objectType='adsk::core::Color', red=255, green=0, blue=0, opacity=255)
print(materials.appearance_to_rgba(SimpleNamespace(color=color)))
'''], cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), '(1.0, 0.0, 0.0, 1.0)')


class FusionLinkAppearanceTest(unittest.TestCase):
    def assert_exported_color(self, source, expected_rgba, expected_xml):
        inertial, status = Link.make_inertial_dict({'base_link': source}, 'ok')
        self.assertEqual(status, 'ok')
        with self.subTest(output='inertial'):
            self.assertEqual(inertial['base_link']['color_rgba'], expected_rgba)
        with tempfile.TemporaryDirectory(prefix='fusion-link-color-') as temporary:
            package = Write.write_ros2_package(
                Path(temporary) / 'color_description', 'color', {}, inertial,
                write_meshes=write_meshes)
            robot = ET.parse(package / 'urdf/color.xacro')
            with self.subTest(output='xml'):
                self.assertEqual(robot.find('link/visual/material/color').get('rgba'), expected_xml)

    def test_current_selected_body_precedes_component_material(self):
        for appearance in (ModernAppearance(Color(0, 255, 0)),
                           LegacyAppearance({'opaque_albedo': ColorProperty(Color(0, 255, 0))})):
            with self.subTest(api=type(appearance).__name__):
                self.assert_exported_color(selected_source(appearance),
                                           (0.0, 1.0, 0.0, 1.0), '0 1 0 1')

    def test_occurrence_override_precedes_current_body_appearance(self):
        source = selected_source(ModernAppearance(Color(0, 255, 0)),
                                 ModernAppearance(Color(0, 0, 255)))
        self.assert_exported_color(source, (0.0, 0.0, 1.0, 1.0), '0 0 1 1')

    def test_unsupported_occurrence_or_body_appearance_stops_material_fallback(self):
        unsupported = (LegacyAppearance({}),
                       LegacyAppearance({'opaque_albedo': ColorProperty(Color(255, 0, 0), texture=True)}))
        for appearance in unsupported:
            for target in ('occurrence', 'body'):
                with self.subTest(appearance=appearance, target=target):
                    source = selected_source(
                        appearance if target == 'body' else ModernAppearance(Color(0, 255, 0)),
                        appearance if target == 'occurrence' else None)
                    source.bodies[0].material = Material(ModernAppearance(Color(0, 0, 255)))
                    self.assert_exported_color(source, None, '0.7 0.7 0.7 1')

    def test_absent_appearances_allow_selected_body_and_component_materials(self):
        source = selected_source(None)
        self.assert_exported_color(source, (1.0, 0.0, 0.0, 1.0), '1 0 0 1')
        source.bodies[0].material = Material(ModernAppearance(Color(0, 0, 255)))
        self.assert_exported_color(source, (0.0, 0.0, 1.0, 1.0), '0 0 1 1')
        source.bodies[0].material = None
        source.owner.component.material = None
        source.owner.component.physicalMaterial = Material(
            LegacyAppearance({'metal_f0': ColorProperty(Color(0, 255, 255))}))
        self.assert_exported_color(source, (0.0, 1.0, 1.0, 1.0), '0 1 1 1')

    def test_only_canonical_selected_bodies_supply_color(self):
        for appearance, expected, xml in (
                (ModernAppearance(Color(0, 255, 0)), (0.0, 1.0, 0.0, 1.0), '0 1 0 1'),
                (None, None, '0.7 0.7 0.7 1')):
            with self.subTest(selected_appearance=appearance):
                body = Body('selected', 0)
                body.appearance = appearance
                hidden = Body('hidden', 10, visible=False)
                hidden.appearance = ModernAppearance(Color(255, 0, 0))
                owner = occurrence('base:1', [hidden, body], component='base_link')
                owner.appearance = None
                source = utils.link_sources(root([owner]))['base_link']
                self.assertEqual(source.bodies, (body,))
                omitted = Body('omitted after selection', 20)
                omitted.appearance = ModernAppearance(Color(0, 0, 255))
                owner.bRepBodies = [hidden, omitted]
                owner.childOccurrences = [occurrence('base:1+child:1', [omitted])]
                body.isVisible = False
                self.assert_exported_color(source, expected, xml)

    def test_unreadable_appearance_is_not_an_absent_override(self):
        class UnreadableAppearance(SimpleNamespace):
            @property
            def appearance(self):
                raise self.error('Unavailable appearance')

        for error in (RuntimeError, AttributeError):
            for target in ('occurrence', 'body'):
                with self.subTest(error=error.__name__, target=target):
                    source = selected_source(ModernAppearance(Color(0, 255, 0)))
                    original = source.owner if target == 'occurrence' else source.bodies[0]
                    attributes = {key: value for key, value in vars(original).items()
                                  if key != 'appearance'}
                    unreadable = UnreadableAppearance(error=error, **attributes)
                    if target == 'occurrence':
                        source = utils.LinkSource(unreadable, source.bodies)
                    else:
                        unreadable.getPhysicalProperties = original.getPhysicalProperties
                        source = utils.LinkSource(source.owner, (unreadable,))
                    self.assert_exported_color(source, None, '0.7 0.7 0.7 1')

    def test_first_present_selected_body_appearance_is_deterministic(self):
        source = selected_source(LegacyAppearance({}))
        later = Body('later', 10)
        later.appearance = ModernAppearance(Color(0, 255, 0))
        source = utils.LinkSource(source.owner, source.bodies + (later,))
        self.assert_exported_color(source, None, '0.7 0.7 0.7 1')
        first = source.bodies[0]
        first.appearance = ModernAppearance(Color(0, 0, 255))
        self.assert_exported_color(source, (0.0, 0.0, 1.0, 1.0), '0 0 1 1')
        first.appearance = None
        self.assert_exported_color(source, (0.0, 1.0, 0.0, 1.0), '0 1 0 1')

    def test_color_extraction_does_not_set_source_properties(self):
        class ReadOnlyAttributes:
            def __init__(self, attributes):
                self.__dict__.update(attributes)

            def __setattr__(self, key, value):
                raise AssertionError('Extraction changed source property ' + key)

        source = selected_source(ModernAppearance(Color(0, 255, 0)))
        body = source.bodies[0]
        read_only_body = ReadOnlyAttributes(dict(vars(body), getPhysicalProperties=body.getPhysicalProperties))
        owner = ReadOnlyAttributes(dict(vars(source.owner),
                                        component=ReadOnlyAttributes(vars(source.owner.component))))
        self.assert_exported_color(utils.LinkSource(owner, (read_only_body,)),
                                   (0.0, 1.0, 0.0, 1.0), '0 1 0 1')


if __name__ == '__main__':
    unittest.main()
