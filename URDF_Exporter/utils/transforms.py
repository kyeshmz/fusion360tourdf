"""Pure 3D transform math for fusion360tourdf (no Fusion API imports).

Single owner for all spatial math so Joint/Link/Write share one convention.

Conventions (match URDF):
- Matrices from Fusion ``Matrix3D.asArray()`` are 16-element row-major:
  ``[R00,R01,R02,Tx, R10,R11,R12,Ty, R20,R21,R22,Tz, 0,0,0,1]``
  with translations in centimeters.
- URDF ``rpy`` is extrinsic XYZ Euler: ``R = Rz(yaw) * Ry(pitch) * Rx(roll)``.
- Lengths convert cm -> m by dividing by 100.
"""

import math

IDENTITY_3X3 = (1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0)


def mat3_mul(a, b):
    """Row-major 3x3 multiply: returns a * b."""
    return (
        a[0] * b[0] + a[1] * b[3] + a[2] * b[6],
        a[0] * b[1] + a[1] * b[4] + a[2] * b[7],
        a[0] * b[2] + a[1] * b[5] + a[2] * b[8],
        a[3] * b[0] + a[4] * b[3] + a[5] * b[6],
        a[3] * b[1] + a[4] * b[4] + a[5] * b[7],
        a[3] * b[2] + a[4] * b[5] + a[5] * b[8],
        a[6] * b[0] + a[7] * b[3] + a[8] * b[6],
        a[6] * b[1] + a[7] * b[4] + a[8] * b[7],
        a[6] * b[2] + a[7] * b[5] + a[8] * b[8],
    )


def mat3_transpose(m):
    """Transpose a row-major 3x3 (= inverse for orthonormal rotations)."""
    return (m[0], m[3], m[6],
            m[1], m[4], m[7],
            m[2], m[5], m[8])


def mat3_from_rpy(rpy):
    """Build a row-major 3x3 from (roll, pitch, yaw)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz * Ry * Rx
    return (
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    )


def rpy_from_mat3(m):
    """Convert a row-major 3x3 rotation to URDF (roll, pitch, yaw).

    Handles gimbal lock (pitch = +/-pi/2) by zeroing yaw, matching the
    approach used by haoyan-dev/fusion2urdf's Transform and Adriaeik's
    _rotation_to_rpy.
    """
    r11, r12, r13, r21, r22, r23, r31, r32, r33 = m
    sp = -r31
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(math.cos(pitch)) < 1e-9:
        roll = math.atan2(-r23, r22)
        yaw = 0.0
    else:
        roll = math.atan2(r32, r33)
        yaw = math.atan2(r21, r11)
    return (roll, pitch, yaw)


def rotation3_from_array16(arr):
    """Extract the row-major 3x3 rotation from a Fusion 16-array."""
    return (arr[0], arr[1], arr[2],
            arr[4], arr[5], arr[6],
            arr[8], arr[9], arr[10])


def translation_m_from_array16_cm(arr):
    """Extract translation in meters from a Fusion 16-array in cm."""
    return [arr[3] / 100.0, arr[7] / 100.0, arr[11] / 100.0]


def xyz_rpy_from_array16_cm(arr):
    """Split a Fusion 16-array (cm) into (xyz meters, rpy radians)."""
    return translation_m_from_array16_cm(arr), list(rpy_from_mat3(rotation3_from_array16(arr)))


def relative_pose(parent_array16_cm, child_array16_cm):
    """Return child pose in parent frame as (xyz meters, rpy radians).

    Computes ``parent^-1 * child`` with proper rotation handling
    (Adriaeik's R_parent^-1 * R_child rule). Falls back to translation-only
    difference when either matrix is missing.
    """
    if parent_array16_cm is None or child_array16_cm is None:
        return None
    try:
        rp = rotation3_from_array16(parent_array16_cm)
        rc = rotation3_from_array16(child_array16_cm)
        r_rel = mat3_mul(mat3_transpose(rp), rc)
        tp = translation_m_from_array16_cm(parent_array16_cm)
        tc = translation_m_from_array16_cm(child_array16_cm)
        # t_rel = R_parent^T * (t_child - t_parent)
        diff = [tc[0] - tp[0], tc[1] - tp[1], tc[2] - tp[2]]
        rpt = mat3_transpose(rp)
        t_rel = [rpt[0] * diff[0] + rpt[1] * diff[1] + rpt[2] * diff[2],
                 rpt[3] * diff[0] + rpt[4] * diff[1] + rpt[5] * diff[2],
                 rpt[6] * diff[0] + rpt[7] * diff[1] + rpt[8] * diff[2]]
        return t_rel, list(rpy_from_mat3(r_rel))
    except Exception:
        return None


def rotate_vector_by_mat3(vec, mat3):
    """Apply a row-major 3x3 rotation to a 3-vector: returns m * v."""
    return [mat3[0] * vec[0] + mat3[1] * vec[1] + mat3[2] * vec[2],
            mat3[3] * vec[0] + mat3[4] * vec[1] + mat3[5] * vec[2],
            mat3[6] * vec[0] + mat3[7] * vec[1] + mat3[8] * vec[2]]


def normalize(vec):
    """Return the unit vector for vec, or [0,0,0] when degenerate."""
    try:
        norm = math.sqrt(sum(v * v for v in vec))
        if norm < 1e-12:
            return [0.0, 0.0, 0.0]
        return [v / norm for v in vec]
    except Exception:
        return [0.0, 0.0, 0.0]


def inverse_pose(xyz_m, rpy):
    """Invert a rigid pose: given frame T in root, return root in frame.

    Used for link visual origins: meshes are exported in the root frame
    (WYSIWYG), so the visual origin must undo the link frame pose.
    """
    try:
        r = mat3_from_rpy(rpy)
        rt = mat3_transpose(r)
        t_inv = [- (rt[0] * xyz_m[0] + rt[1] * xyz_m[1] + rt[2] * xyz_m[2]),
                 - (rt[3] * xyz_m[0] + rt[4] * xyz_m[1] + rt[5] * xyz_m[2]),
                 - (rt[6] * xyz_m[0] + rt[7] * xyz_m[1] + rt[8] * xyz_m[2])]
        return t_inv, list(rpy_from_mat3(rt))
    except Exception:
        return [-v for v in xyz_m], [0.0, 0.0, 0.0]


def format_float(value, digits=6):
    """Format a float compactly without truncating tiny physics values.

    URDF inertia/mass values below 1e-6 (e.g. small screws, sensor heads)
    were truncated to 0.0 by ``round(x, 6)`` in older exporters
    (fixed upstream by newtonjeri with scientific notation). Uses general
    format with 9 significant digits: round-trips through float() and
    stays readable for normal magnitudes.
    """
    try:
        f = float(value)
    except Exception:
        return '0'
    if f == 0.0:
        return '0'
    text = '{:.9g}'.format(f)
    # Normalize -0 to 0 for cleaner URDF.
    if text in ('-0', '-0.0'):
        return '0'
    return text


def format_vector(vec, digits=6):
    """Format an xyz/rpy vector for URDF attributes."""
    rounded = []
    for v in vec:
        try:
            rounded.append(round(float(v), digits))
        except Exception:
            rounded.append(0.0)
    # Clean -0.0 for readability.
    return ' '.join('0' if x == 0 else str(x) for x in rounded)
