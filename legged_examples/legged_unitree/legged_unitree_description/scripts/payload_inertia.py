#!/usr/bin/env python3
"""Mass properties of the Aliengo payloads (NUC and slam box) from their STEP files, for urdf/common/nuc.xacro and
urdf/common/slam_box.xacro.

Needs OpenCascade, which is not a ROS dependency: install it in a throwaway venv and run the script with it
    python3 -m venv /tmp/occ && /tmp/occ/bin/pip install cadquery-ocp
    /tmp/occ/bin/python payload_inertia.py ~/RaNAV/src/legged_control/Nuc.step ~/RaNAV/src/legged_control/NewSlamBox.step

The STEP files carry no materials, so every solid gets a density from the first rule of RULES whose pattern is in its
assembly path (part names), and parts of known mass (the Airy, the IMU) get that mass spread over their volume. The
NUC STEP has the chassis and the thermal module but no board: the NUC itself is a uniform box of its chassis bounding
box, of NUC_UNIT_MASS. All of this only sets how the mass is distributed: the xacros multiply the inertia per kg
printed here by the WEIGHED mass of the assembly, so the estimated mass printed here is only a placeholder.

Frame of each payload (the frame of its URDF link): center of the bottom face of the bounding box of the whole
assembly, with the CAD axes. In both files x is the long/forward axis and z points up: the Airy in the slam box is
pitched about y by +5 deg, its dome axis leaning to +x, as the Airy of robot.xacro pitched forward.
Prints SI units: size of the bounding box, estimated mass, CoM, inertia about the CoM divided by the mass, and for the
slam box the pose of the Airy (center of its base, the origin of the rslidar frame).
"""
import math
import re
import sys

import numpy as np
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.Bnd import Bnd_Box
from OCP.collections import Sequence_TDF_Label
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool

STEEL = 7850.0       # kg/m^3: screws, nuts, VESA bracket
PLASTIC = 1250.0     # kg/m^3: solid PLA/PETG; printed parts with infill weigh less (the weighed mass fixes it)
NUC_UNIT_MASS = 0.55  # kg, typical "slim" NUC with RAM and SSD, without the power supply

# (regex on the assembly path, density [kg/m^3] or ("mass", total kg of all the solids it matches) or "nuc_unit")
RULES = [
    (r"GR_ALL_ASM", "nuc_unit"),                 # Nuc.step: the NUC chassis
    (r"vesa_bracket", STEEL),
    (r"Airy 3D\+FOV", ("mass", 0.230)),         # RoboSense Airy datasheet: 230 g
    (r"3DM-CX5", ("mass", 0.008)),              # MicroStrain 3DM-CX5 IMU, ~8 g
    (r"ISO 4762|ISO 4032|SCREW", STEEL),
    (r".", PLASTIC),                            # housings, plates, cover, roof, pillars
]
AIRY_BODY = r"Airy 3D\+FOV"   # the largest of its solids is the body: CAD frame at the center of the base, dome along y


def read(path):
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(path) != IFSelect_RetDone:
        sys.exit(f"cannot read {path}")
    reader.Transfer(doc)
    return doc, XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())   # the labels live as long as doc


def label_name(label):
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else "?"


def leaves(st):
    """(path, shape in the file frame, global placement, shape in its own frame) of every leaf of the assembly."""
    out = []

    def walk(label, loc, path):
        if st.IsReference_s(label):
            ref = TDF_Label()
            st.GetReferredShape_s(label, ref)
            walk(ref, loc.Multiplied(st.GetLocation_s(label)), path + [label_name(label)])
        elif st.IsAssembly_s(label):
            comps = Sequence_TDF_Label()
            st.GetComponents_s(label, comps)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), loc, path)
        else:
            local = st.GetShape_s(label)
            out.append((" / ".join(path), local.Moved(loc), loc, local))

    roots = Sequence_TDF_Label()
    st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [label_name(roots.Value(i))])
    return out


def bbox(shapes, use_triangulation=True):
    box = Bnd_Box()
    for s in shapes:
        BRepBndLib.Add_s(s, box, use_triangulation)
    mn, mx = box.CornerMin(), box.CornerMax()
    return np.array([mn.X(), mn.Y(), mn.Z()]), np.array([mx.X(), mx.Y(), mx.Z()])


def has_solid(shape):
    return TopExp_Explorer(shape, TopAbs_SOLID).More()


def volume(shape):
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, p)
    return p


def rule_of(path):
    for i, (pattern, rule) in enumerate(RULES):
        if re.search(pattern, path):
            return i, rule
    raise ValueError(path)


def analyze(path):
    doc, st = read(path)
    parts = [lf for lf in leaves(st) if has_solid(lf[1])]
    lo, hi = bbox([p[1] for p in parts])
    origin = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])  # mm, center of the bottom face

    # volume of each rule, for the rules given as a total mass
    groups = {}
    for p, shape, _, _ in parts:
        i, rule = rule_of(p)
        groups.setdefault(i, []).append(shape)

    total = GProp_GProps(gp_Pnt(*origin))
    table = []
    for i, shapes in groups.items():
        rule = RULES[i][1]
        vols = [volume(s) for s in shapes]
        v = sum(abs(q.Mass()) for q in vols) * 1e-9  # m^3
        if rule == "nuc_unit":
            # uniform box of the chassis bounding box instead of its sheet metal
            blo, bhi = bbox(shapes)
            box = BRepPrimAPI_MakeBox(gp_Pnt(*blo), gp_Pnt(*bhi)).Shape()
            q = volume(box)
            total.Add(q, NUC_UNIT_MASS / (q.Mass() * 1e-9) * 1e-9)
            table.append((RULES[i][0], NUC_UNIT_MASS, f"box {bhi[0]-blo[0]:.1f} x {bhi[1]-blo[1]:.1f} x {bhi[2]-blo[2]:.1f} mm"))
            continue
        rho = rule[1] / v if isinstance(rule, tuple) else rule
        for q in vols:
            total.Add(q, math.copysign(rho * 1e-9, q.Mass()))   # mm^3 -> m^3; a reversed solid has a negative volume
        table.append((RULES[i][0], rho * v, f"{v*1e6:.1f} cm3 at {rho:.0f} kg/m3"))

    mass = total.Mass()
    com = np.array([total.CentreOfMass().X(), total.CentreOfMass().Y(), total.CentreOfMass().Z()])
    m = total.MatrixOfInertia()   # about the CoM, CAD axes, mm^2 kg
    inertia = np.array([[m.Value(r, c) for c in (1, 2, 3)] for r in (1, 2, 3)]) * 1e-6

    print(f"== {path}")
    for pattern, kg, note in table:
        print(f"   {pattern:28s} {kg:7.3f} kg  ({note})")
    size = (hi - lo) / 1000
    print(f"   bounding box {size[0]:.4f} x {size[1]:.4f} x {size[2]:.4f} m; frame origin at CAD ({origin[0]:.2f}, "
          f"{origin[1]:.2f}, {origin[2]:.2f}) mm")
    c = (com - origin) / 1000
    k = inertia / mass
    print(f"   estimated mass {mass:.3f} kg (weigh it)")
    print(f"   com     {c[0]:.4f} {c[1]:.4f} {c[2]:.4f}")
    print(f"   inertia/mass ixx {k[0,0]:.6f} ixy {k[0,1]:.6f} ixz {k[0,2]:.6f} iyy {k[1,1]:.6f} iyz {k[1,2]:.6f} "
          f"izz {k[2,2]:.6f}  [m^2]")
    s = size
    print(f"   (uniform box of the bounding box: ixx {(s[1]**2+s[2]**2)/12:.6f} iyy {(s[0]**2+s[2]**2)/12:.6f} "
          f"izz {(s[0]**2+s[1]**2)/12:.6f})")

    airy = [lf for lf in parts if re.search(AIRY_BODY, lf[0])]
    if airy:
        # the body is the largest solid of the Airy model, the others are its connector and screws
        _, _, loc, local = max(airy, key=lambda lf: abs(volume(lf[1]).Mass()))
        t = loc.Transformation()
        R = np.array([[t.Value(r, col) for col in (1, 2, 3)] for r in (1, 2, 3)])
        tr = t.TranslationPart()
        axis = R[:, 1]   # the Airy CAD model has its dome along its y axis, origin at the center of the base
        llo, lhi = bbox([local], False)
        pos = (np.array([tr.X(), tr.Y(), tr.Z()]) - origin) / 1000
        pitch = math.degrees(math.atan2(axis[0], axis[2]))
        print(f"   Airy base center {pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}, dome axis ({axis[0]:.4f}, {axis[1]:.4f}, "
              f"{axis[2]:.4f}) = pitch {pitch:.2f} deg; body {lhi[1]-llo[1]:.1f} mm high from the base "
              f"(local y {llo[1]:.2f}..{lhi[1]:.2f})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for f in sys.argv[1:]:
        analyze(f)
