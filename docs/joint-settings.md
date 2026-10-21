# Joint effort and velocity settings

`joint_limits` sets effort and velocity for individual moving joints in a ROS 2 export.
Ordinary Fusion joints and as-built joints use the same options.
These settings do not edit the CAD design or override its travel bounds.

## Python configuration

Pass a mapping from exact exported joint names to the fields you want to override:

```python
export_design(design, destination, config={
    'effort': 100.0,
    'velocity': 2.0,
    'joint_limits': {
        'drive': {'effort': 4.0, 'velocity': 0.8},
        'lift': {'effort': 6.0},
    },
})
```

In this example, `drive` uses effort `4.0` and velocity `0.8`.
`lift` uses effort `6.0` and the global velocity `2.0`.
Other moving joints use the global effort `100.0` and velocity `2.0`.
Both globals default to `100.0` when omitted.
An omitted or empty `joint_limits` mapping leaves those defaults unchanged.
An empty field table inherits both globals but still requires a valid moving joint name.

The pure-Python `write_ros2_package` entry point accepts the same `config` mapping.
It resolves settings on its own copy of the supplied joint dictionaries before mesh callbacks or publication.
The input joint dictionaries and configuration remain unchanged on success and on validation failure.

## TOML configuration

The Fusion script reads `URDF_Exporter/xacro_export.toml` when that file exists.
Use a table for each exported joint name:

```toml
effort = 100.0
velocity = 2.0

[joint_limits.drive]
effort = 4.0
velocity = 0.8

[joint_limits.lift]
effort = 6.0
```

For Python callers, `load_config(script_dir, overrides)` combines that directory's TOML file with call overrides.
`export_design` accepts the resulting configuration through its `config` argument.
`export_design` does not read a TOML file itself.

## Names and validation

The selector is the joint's final sanitized export name, not its Fusion display name or entity token.
Copy the name from a `joint` element in the generated URDF or from `config/robot_data.yaml`.
For example, a top-level joint named `drive pin` exports as `drive_pin`.
A joint named `pivot` in assembly context `left:1` exports as `left_1_pivot`.
Repeated assembly instances need their distinct full exported names.
Selectors are case-sensitive and exact.
There is no wildcard, suffix, native-identity, or display-name matching.

The exporter raises `ValueError` for these inputs before it publishes a package:

- A selector that does not name an extracted joint, including a suppressed or excluded joint.
- A fixed joint, including a locked joint or a generated rigid attachment.
- Two source joints with the same sanitized export name.
- A value other than a mapping of joint names to field mappings.
- A field other than `effort` or `velocity`.
- A field value that is not a finite positive real number representable as a Python float.

Booleans, numeric strings, zero, negative values, NaN, and infinity are invalid.
The loader normalizes valid values to floats and copies nested tables.
It does not mutate the caller's mapping or the global defaults.

## Precedence and units

Call overrides replace TOML global options.
Per-joint tables merge by joint name and field, with call fields taking precedence over TOML fields.
A field omitted from both per-joint tables inherits the resulting global value.
An empty call table does not remove a TOML field or joint table.
Invalid per-joint TOML fields remain errors even if a call override supplies a replacement.

For direct `write_ros2_package` callers, each field resolves in this order:

1. The explicit per-name `joint_limits` field.
2. The existing field in the supplied joint dictionary.
3. The global option, or its default when omitted.

A partial or empty per-name table preserves existing explicit joint fields.
An explicit per-name field replaces the existing value before model validation.
Fusion extraction supplies global values in the joint dictionaries, so omitted per-name fields inherit those globals.
One pure resolver owns selector binding for both extraction and the final writer boundary.
Extraction calls it before orientation checks so invalid selectors are not hidden by disconnected source topology.

| Joint type | `effort` | `velocity` |
| --- | --- | --- |
| Revolute or continuous | Torque in N m | Angular speed in rad/s |
| Prismatic | Force in N | Linear speed in m/s |

The generated URDF and Xacro store the resolved values in each moving joint's `limit` element.
`config/robot_data.yaml` stores the same resolved values.
The existing `ros2_control` joints and controller membership keep the same joint names.
Those control assets do not add separate effort or velocity limit parameters.
These settings are description metadata, not proof that hardware enforces the limits.

## Travel bounds and limitations

Lower and upper travel bounds remain authoritative Fusion motion data relative to the exported current pose.
For example, a revolute joint at `0.25` rad with CAD bounds `[-1, 2]` exports bounds `[-1.25, 1.75]`.
A slider at `5` cm with CAD bounds `[0, 20]` cm exports bounds `[-0.05, 0.15]` m.
Effort and velocity overrides do not change either result.
Continuous joints accept both settings without adding lower or upper bounds.

There is no travel-bound override or generalized-joint decomposition option.
Existing loop reduction remains unchanged.
A closing joint omitted from the URDF tree retains its resolved values in the sidecar's `closing_joints` metadata.
Closing-joint selectors are validated against the complete joint set before tree reduction.
Fixed closing joints cannot receive effort or velocity settings.

Offline coverage uses `python3 -m unittest tests.test_joint_settings -v`.
It exercises the public exporter with Fusion API doubles and the direct pure-Python writer, then inspects generated package files.
It does not verify an original issue's CAD document, live Fusion behavior, or ROS runtime enforcement.
