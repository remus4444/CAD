"""
Buckeye Solar Racing - Surya Suspension Load Casing
By Mehmet Kara (Python translation)
Solves pickup-point forces for 2G bump, cornering, and braking load cases.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3D projection)


# =====================================================================
# Vehicle Parameters
# =====================================================================
m   = 319            # vehicle mass [kg]
g   = 9.81           # gravitational constant [m/s^2]
W   = m * g          # vehicle weight [N]
WB  = 1.518          # vehicle wheelbase [m]
TW  = (558.575 * 2) / 1000   # trackwidth [m]
hcg = 0.318          # CG height [m]
Wf  = W * (1 - (643.7 / 1518))   # weight on front axle [N]
Wr  = W * (643.7 / 1518)         # weight on rear axle [N]


# =====================================================================
# Load Data
# =====================================================================
data_file = './WC Forces & Moments Data/corneringandbrakingdata.csv'   # pull CarSim data
T = pd.read_csv(data_file)

Ax = T['Ax'].to_numpy()   # save long G
Ay = T['Ay'].to_numpy()   # save lat G

# Cornering snapshot: |Ay| ~ 1G, |Ax| ~ 0G
idxC = int(np.argmin(np.abs(np.abs(Ay) - 1.0) + np.abs(Ax)))
print(f"Cornering snapshot  @ t = {T['Time'].iloc[idxC]:.3f} s "
      f"(Ax={Ax[idxC]:.3f}G, Ay={Ay[idxC]:.3f}G)")

# Braking snapshot: |Ax| ~ 1G, |Ay| ~ 0G
idxB = int(np.argmin(np.abs(np.abs(Ax) - 1.0) + np.abs(Ay)))
print(f"Braking    snapshot @ t = {T['Time'].iloc[idxB]:.3f} s "
      f"(Ax={Ax[idxB]:.3f}G, Ay={Ay[idxB]:.3f}G)")


def extract_wc(row):
    """Extract forces and moments at all four wheel centers for a given row."""
    return {
        'Fx': np.array([T['Fx_WC_L1'].iloc[row], T['Fx_WC_L2'].iloc[row],
                        T['Fx_WC_R1'].iloc[row], T['Fx_WC_R2'].iloc[row]]),
        'Fy': np.array([T['Fy_WC_L1'].iloc[row], T['Fy_WC_L2'].iloc[row],
                        T['Fy_WC_R1'].iloc[row], T['Fy_WC_R2'].iloc[row]]),
        'Fz': np.array([T['Fz_WC_L1'].iloc[row], T['Fz_WC_L2'].iloc[row],
                        T['Fz_WC_R1'].iloc[row], T['Fz_WC_R2'].iloc[row]]),
        'Mx': np.array([T['Mx_WC_L1'].iloc[row], T['Mx_WC_L2'].iloc[row],
                        T['Mx_WC_R1'].iloc[row], T['Mx_WC_R2'].iloc[row]]),
        'My': np.array([T['My_WC_L1'].iloc[row], T['My_WC_L2'].iloc[row],
                        T['My_WC_R1'].iloc[row], T['My_WC_R2'].iloc[row]]),
        'Mz': np.array([T['Mz_WC_L1'].iloc[row], T['Mz_WC_L2'].iloc[row],
                        T['Mz_WC_R1'].iloc[row], T['Mz_WC_R2'].iloc[row]]),
    }


WC_corner = extract_wc(idxC)
WC_brake  = extract_wc(idxB)


# =====================================================================
# Suspension geometry
# Convention: SHARK [X, Y, Z] mm right-side coords
# Script: metres, FL mirrors Y, FR uses Y as-is
# =====================================================================
def s2v_L(p):
    return np.array([p[0] / 1000.0, -p[1] / 1000.0, p[2] / 1000.0])


def s2v_R(p):
    return np.array([p[0] / 1000.0,  p[1] / 1000.0, p[2] / 1000.0])


# Front right pickup points [mm]
LW_front_sh = np.array([1622.7781, 193.1,    301.01  ])
LW_rear_sh  = np.array([1383.8492, 193.1,    296.5788])
LW_outer_sh = np.array([1524.8965, 459.4677, 210.3089])
UW_front_sh = np.array([1548.297,  213.1,    446.1002])
UW_rear_sh  = np.array([1352.1572, 213.1,    430.5774])
UW_outer_sh = np.array([1507.3132, 440.2202, 427.0743])
TR_inner_sh = np.array([1618,      213.8197, 407.513 ])
TR_outer_sh = np.array([1617.9983, 482.4502, 367.7329])
WC_front_sh = np.array([1517.9999, 558.6902, 282.8009])

# DUMMY front damper mount points - REPLACE WITH REAL SHARK COORDINATES
# Lower mount: on the lower wishbone, near the outer ball joint, offset up
# Upper mount: on the chassis, inboard and well above the lower mount
FDamp_lower_sh = np.array([1524.0, 420.0, 260.0])   # DUMMY: lower mount on LW [mm]
FDamp_upper_sh = np.array([1480.0, 213.0, 520.0])   # DUMMY: upper mount on chassis [mm]

geom = {'FL': {}, 'FR': {}, 'RL': {}, 'RR': {}}

# Front left pickup points
geom['FL']['UW_A']   = s2v_L(UW_front_sh)
geom['FL']['UW_B']   = s2v_L(UW_rear_sh)
geom['FL']['UW_C']   = s2v_L(UW_outer_sh)
geom['FL']['LW_D']   = s2v_L(LW_front_sh)
geom['FL']['LW_E']   = s2v_L(LW_rear_sh)
geom['FL']['LW_F']   = s2v_L(LW_outer_sh)
geom['FL']['TR_G']   = s2v_L(TR_inner_sh)
geom['FL']['TR_H']   = s2v_L(TR_outer_sh)
geom['FL']['Damp_L'] = s2v_L(FDamp_lower_sh)
geom['FL']['Damp_U'] = s2v_L(FDamp_upper_sh)

# Front right pickup points
geom['FR']['UW_A']   = s2v_R(UW_front_sh)
geom['FR']['UW_B']   = s2v_R(UW_rear_sh)
geom['FR']['UW_C']   = s2v_R(UW_outer_sh)
geom['FR']['LW_D']   = s2v_R(LW_front_sh)
geom['FR']['LW_E']   = s2v_R(LW_rear_sh)
geom['FR']['LW_F']   = s2v_R(LW_outer_sh)
geom['FR']['TR_G']   = s2v_R(TR_inner_sh)
geom['FR']['TR_H']   = s2v_R(TR_outer_sh)
geom['FR']['Damp_L'] = s2v_R(FDamp_lower_sh)
geom['FR']['Damp_U'] = s2v_R(FDamp_upper_sh)

WC_pos = {
    'FL': s2v_L(WC_front_sh),
    'FR': s2v_R(WC_front_sh),
    'RL': None,
    'RR': None,
}

# Rear pickup points [mm]
TA_front_sh    = np.array([344.0235, 260,      245.4731])
TA_rear_sh     = np.array([344.0235, 82.454,   245.4731])
TA_tip_sh      = np.array([0,        396.675,  282.8   ])
TR_rear_in_sh  = np.array([118.5761, 287.4165, 257.1598])
TR_rear_out_sh = np.array([-1.3699,  285.2965, 270.1741])
WC_rear_sh     = np.array([0,        558.575,  282.8   ])

# DUMMY rocker mechanism points - REPLACE WITH REAL SHARK COORDINATES
# The rocker is a rigid triangular body with three nodes:
#   N1: pivot on the trailing arm (this is the only suspension-side connection)
#   N2: coilover lower end (coilover other end goes to chassis)
#   N3: rod end (rod other end goes to chassis)
# Two chassis anchor points complete the loops:
#   Coil_chassis: coilover upper mount on chassis
#   Rod_chassis : rod chassis-side mount
Rkr_N1_sh         = np.array([ 50.0,  380.0,  320.0])  # rocker pivot ON trailing arm
Rkr_N2_sh         = np.array([100.0,  300.0,  450.0])  # rocker coilover node
Rkr_N3_sh         = np.array([150.0,  330.0,  430.0])  # rocker rod node
Coil_chassis_sh   = np.array([200.0,  200.0,  550.0])  # coilover top on chassis
Rod_chassis_sh    = np.array([300.0,  150.0,  500.0])  # rod chassis-side mount

# Rear left pickup points
geom['RL']['P1']           = s2v_L(TA_front_sh)
geom['RL']['P2']           = s2v_L(TA_rear_sh)
geom['RL']['T']            = s2v_L(TA_tip_sh)
geom['RL']['TR_Q']         = s2v_L(TR_rear_in_sh)
geom['RL']['TR_S']         = s2v_L(TR_rear_out_sh)
geom['RL']['Rkr_N1']       = s2v_L(Rkr_N1_sh)         # pivot on trailing arm
geom['RL']['Rkr_N2']       = s2v_L(Rkr_N2_sh)         # coilover node on rocker
geom['RL']['Rkr_N3']       = s2v_L(Rkr_N3_sh)         # rod node on rocker
geom['RL']['Coil_chassis'] = s2v_L(Coil_chassis_sh)   # coilover top
geom['RL']['Rod_chassis']  = s2v_L(Rod_chassis_sh)    # rod chassis end

# Rear right pickup points
geom['RR']['P1']           = s2v_R(TA_front_sh)
geom['RR']['P2']           = s2v_R(TA_rear_sh)
geom['RR']['T']            = s2v_R(TA_tip_sh)
geom['RR']['TR_Q']         = s2v_R(TR_rear_in_sh)
geom['RR']['TR_S']         = s2v_R(TR_rear_out_sh)
geom['RR']['Rkr_N1']       = s2v_R(Rkr_N1_sh)
geom['RR']['Rkr_N2']       = s2v_R(Rkr_N2_sh)
geom['RR']['Rkr_N3']       = s2v_R(Rkr_N3_sh)
geom['RR']['Coil_chassis'] = s2v_R(Coil_chassis_sh)
geom['RR']['Rod_chassis']  = s2v_R(Rod_chassis_sh)

WC_pos['RL'] = s2v_L(WC_rear_sh)
WC_pos['RR'] = s2v_R(WC_rear_sh)


# =====================================================================
# Helper functions
# =====================================================================
def unit_vec(A, B):
    """Unit vector from B toward A."""
    v = np.asarray(A, dtype=float) - np.asarray(B, dtype=float)
    return v / np.linalg.norm(v)


def solve_dwb(Fwc, Mwc, g_s, WC):
    """
    Four-member double wishbone solver: UW + LW + TR + damper (6x4 system).
    """
    Fwc = np.asarray(Fwc, dtype=float).flatten()
    Mwc = np.asarray(Mwc, dtype=float).flatten()
    WC  = np.asarray(WC,  dtype=float).flatten()

    uUW   = unit_vec(g_s['UW_C'],   (g_s['UW_A'] + g_s['UW_B']) / 2.0)
    uLW   = unit_vec(g_s['LW_F'],   (g_s['LW_D'] + g_s['LW_E']) / 2.0)
    uTR   = unit_vec(g_s['TR_H'],    g_s['TR_G'])
    uDamp = unit_vec(g_s['Damp_U'],  g_s['Damp_L'])   # front damper unit vector

    rUW   = g_s['UW_C']   - WC
    rLW   = g_s['LW_F']   - WC
    rTR   = g_s['TR_H']   - WC
    rDamp = g_s['Damp_L'] - WC

    cUW   = np.cross(rUW,   uUW)
    cLW   = np.cross(rLW,   uLW)
    cTR   = np.cross(rTR,   uTR)
    cDamp = np.cross(rDamp, uDamp)

    # 6x4 over-determined system solved via least squares
    A_top = np.column_stack([uUW, uLW, uTR, uDamp])     # 3x4
    A_bot = np.column_stack([cUW, cLW, cTR, cDamp])     # 3x4
    A     = np.vstack([A_top, A_bot])                   # 6x4

    b = np.concatenate([-Fwc, -Mwc])                    # 6

    x, *_ = np.linalg.lstsq(A, b, rcond=None)

    return {
        'F_upperWishbone': x[0] * uUW,
        'F_lowerWishbone': x[1] * uLW,
        'F_tieRod':        x[2] * uTR,
        'F_damper':        x[3] * uDamp,
        'magnitudes': {'UW': x[0], 'LW': x[1], 'TR': x[2], 'Damp': x[3]},
    }


def solve_ta(Fwc, Mwc, g_s, WC):
    """
    Trailing arm with rocker mechanism (4 unknown scalars, 6x4 system).

    Members loading the trailing arm:
      * Trailing arm itself (axial scalar along arm)            [1 unknown]
      * Tie rod (axial scalar)                                  [1 unknown]
      * Rocker pivot reaction (3 components)                    -- determined
                                                                  by rocker
                                                                  equilibrium
                                                                  from the
                                                                  coilover and
                                                                  rod scalars

    The rocker is a rigid body in static equilibrium. The pin joint at N1
    on the trailing arm carries a 3D force; the coilover and rod carry pure
    axial forces. By Newton's third law the force the rocker applies to
    the trailing arm at N1 equals (coilover_axial + rod_axial), each in
    its own line of action. The pivot itself contributes no independent
    unknown -- it is fully determined by the coilover and rod scalars.

    So the four scalars solved for are:
        x[0] : arm axial
        x[1] : tie rod axial
        x[2] : coilover axial (positive = tension along uCoil from rocker
                               toward chassis anchor)
        x[3] : rod axial      (positive = tension along uRod  from rocker
                               toward chassis anchor)

    The force the rocker applies to the trailing arm at N1 is:
        F_pivot_on_arm = -(x[2]*uCoil_from_rocker + x[3]*uRod_from_rocker)
    because the rocker is in equilibrium with those two member forces and
    a pivot reaction equal in magnitude/opposite in sign.
    """
    Fwc = np.asarray(Fwc, dtype=float).flatten()
    Mwc = np.asarray(Mwc, dtype=float).flatten()
    WC  = np.asarray(WC,  dtype=float).flatten()

    # Existing trailing-arm members
    uArm = unit_vec(g_s['T'],    (g_s['P1'] + g_s['P2']) / 2.0)
    uTR  = unit_vec(g_s['TR_S'],  g_s['TR_Q'])

    # Coilover and rod unit vectors, defined as pointing from the rocker
    # nodes toward their chassis anchors (so positive scalar = tension).
    uCoil = unit_vec(g_s['Coil_chassis'], g_s['Rkr_N2'])
    uRod  = unit_vec(g_s['Rod_chassis'],  g_s['Rkr_N3'])

    # The pivot reaction applied to the trailing arm at N1 is the negative
    # sum of the coilover and rod forces acting on the rocker (Newton's 3rd
    # law for the rocker as a rigid body in static equilibrium).
    # Force on rocker from coilover scalar s_coil = +s_coil * uCoil
    # Force on rocker from rod scalar      s_rod  = +s_rod  * uRod
    # Pivot reaction on rocker (from arm)        = -(s_coil*uCoil + s_rod*uRod)
    # By Newton's 3rd law, force from rocker onto arm at N1
    #   F_pivot_on_arm = +(s_coil*uCoil + s_rod*uRod)

    # Moment arms about wheel centre
    rArm   = g_s['T']      - WC
    rTR    = g_s['TR_S']   - WC
    rPivot = g_s['Rkr_N1'] - WC   # rocker pivot is on the trailing arm

    cArm = np.cross(rArm, uArm)
    cTR  = np.cross(rTR,  uTR)

    # Coilover and rod each contribute force uCoil/uRod at the pivot point
    # (because that is the point on the trailing arm where the rocker
    #  transmits its reaction to the arm).
    cCoil = np.cross(rPivot, uCoil)
    cRod  = np.cross(rPivot, uRod)

    # 6x4 over-determined system
    A_top = np.column_stack([uArm, uTR, uCoil, uRod])
    A_bot = np.column_stack([cArm, cTR, cCoil, cRod])
    A     = np.vstack([A_top, A_bot])

    b = np.concatenate([-Fwc, -Mwc])

    x, *_ = np.linalg.lstsq(A, b, rcond=None)

    s_arm, s_tr, s_coil, s_rod = x

    # Reaction force from the rocker into the trailing arm at N1
    F_pivot_on_arm = s_coil * uCoil + s_rod * uRod

    return {
        'F_trailingArm': s_arm * uArm,
        'F_tieRod':      s_tr  * uTR,
        'F_coilover':    s_coil * uCoil,    # force on rocker, along line to chassis
        'F_rod':         s_rod  * uRod,     # force on rocker, along line to chassis
        'F_pivot_on_arm': F_pivot_on_arm,   # force the rocker puts on the arm at N1
        'magnitudes': {
            'Arm':  s_arm,
            'TR':   s_tr,
            'Coil': s_coil,
            'Rod':  s_rod,
        },
    }


# =====================================================================
# Geometry diagnostics
# =====================================================================
print("\n--- Front Member Unit Vectors (sanity check) ---")
uUW_FL   = unit_vec(geom['FL']['UW_C'],   (geom['FL']['UW_A'] + geom['FL']['UW_B']) / 2.0)
uLW_FL   = unit_vec(geom['FL']['LW_F'],   (geom['FL']['LW_D'] + geom['FL']['LW_E']) / 2.0)
uTR_FL   = unit_vec(geom['FL']['TR_H'],    geom['FL']['TR_G'])
uFDmp_FL = unit_vec(geom['FL']['Damp_U'],  geom['FL']['Damp_L'])
print(f"FL UW   unit vec: [{uUW_FL[0]:7.4f}, {uUW_FL[1]:7.4f}, {uUW_FL[2]:7.4f}]")
print(f"FL LW   unit vec: [{uLW_FL[0]:7.4f}, {uLW_FL[1]:7.4f}, {uLW_FL[2]:7.4f}]")
print(f"FL TR   unit vec: [{uTR_FL[0]:7.4f}, {uTR_FL[1]:7.4f}, {uTR_FL[2]:7.4f}]")
print(f"FL Damp unit vec: [{uFDmp_FL[0]:7.4f}, {uFDmp_FL[1]:7.4f}, {uFDmp_FL[2]:7.4f}]  "
      "<-- should have significant Z")

print("\n--- Rear Member Unit Vectors (sanity check) ---")
uArm_RL  = unit_vec(geom['RL']['T'],            (geom['RL']['P1'] + geom['RL']['P2']) / 2.0)
uTR_RL   = unit_vec(geom['RL']['TR_S'],          geom['RL']['TR_Q'])
uCoil_RL = unit_vec(geom['RL']['Coil_chassis'],  geom['RL']['Rkr_N2'])
uRod_RL  = unit_vec(geom['RL']['Rod_chassis'],   geom['RL']['Rkr_N3'])
print(f"RL Arm  unit vec: [{uArm_RL[0]:7.4f}, {uArm_RL[1]:7.4f}, {uArm_RL[2]:7.4f}]")
print(f"RL TR   unit vec: [{uTR_RL[0]:7.4f}, {uTR_RL[1]:7.4f}, {uTR_RL[2]:7.4f}]")
print(f"RL Coil unit vec: [{uCoil_RL[0]:7.4f}, {uCoil_RL[1]:7.4f}, {uCoil_RL[2]:7.4f}]  "
      "<-- coilover line of action (rocker -> chassis)")
print(f"RL Rod  unit vec: [{uRod_RL[0]:7.4f}, {uRod_RL[1]:7.4f}, {uRod_RL[2]:7.4f}]  "
      "<-- rod line of action (rocker -> chassis)")


# =====================================================================
# Print helpers
# =====================================================================
def print_wc_header(label):
    print(f"\n--- Wheel Center Inputs ({label}) ---")
    print(f"{'Corner':<8} {'Fx (N)':>10} {'Fy (N)':>10} {'Fz (N)':>10} "
          f"{'Mx (Nm)':>10} {'My (Nm)':>10} {'Mz (Nm)':>10}")
    print('-' * 74)


def print_pickup_header(label):
    print(f"\n--- Pickup Point Forces ({label}) ---")
    print(f"{'Corner':<8} {'Member':<6} {'Fx (N)':>10} {'Fy (N)':>10} {'Fz (N)':>10}")
    print('-' * 48)


def print_force_row(corner, member, F):
    print(f"{corner:<8} {member:<6} {F[0]:>10.2f} {F[1]:>10.2f} {F[2]:>10.2f}")


# =====================================================================
# Load case 1: 2G bump
# =====================================================================
print("\n========== 2G BUMP ==========")

# Wheel center inputs (symmetric, no lateral/longitudinal load)
print_wc_header('2G Bump')
Fz_bump = (m * 2 * g) / 4.0
bump_corners = ['FL', 'FR', 'RL', 'RR']
for c in bump_corners:
    print(f"{c:<8} {0.0:>10.2f} {0.0:>10.2f} {Fz_bump:>10.2f} "
          f"{0.0:>10.2f} {0.0:>10.2f} {0.0:>10.2f}")

print_pickup_header('2G Bump')

Fwc_bump = np.array([0.0, 0.0, Fz_bump])
Mwc_zero = np.array([0.0, 0.0, 0.0])

sides_front = ['FL', 'FR']
sides_rear  = ['RL', 'RR']
wc_front    = [WC_pos['FL'], WC_pos['FR']]
wc_rear     = [WC_pos['RL'], WC_pos['RR']]

bump = {}
for k, s in enumerate(sides_front):
    bump[s] = solve_dwb(Fwc_bump, Mwc_zero, geom[s], wc_front[k])
    print_force_row(s, 'UW',   bump[s]['F_upperWishbone'])
    print_force_row(s, 'LW',   bump[s]['F_lowerWishbone'])
    print_force_row(s, 'TR',   bump[s]['F_tieRod'])
    print_force_row(s, 'Damp', bump[s]['F_damper'])

for k, s in enumerate(sides_rear):
    bump[s] = solve_ta(Fwc_bump, Mwc_zero, geom[s], wc_rear[k])
    print_force_row(s, 'Arm',  bump[s]['F_trailingArm'])
    print_force_row(s, 'TR',   bump[s]['F_tieRod'])
    print_force_row(s, 'Coil', bump[s]['F_coilover'])
    print_force_row(s, 'Rod',  bump[s]['F_rod'])


# =====================================================================
# Load case 2: 1G cornering
# =====================================================================
print("\n========== CORNERING (Ay~1G) ==========")

sides_all  = ['FL', 'FR', 'RL', 'RR']
wc_all     = [WC_pos['FL'], WC_pos['FR'], WC_pos['RL'], WC_pos['RR']]
# CSV column order: L1=FL(idx0), L2=RL(idx1), R1=FR(idx2), R2=RR(idx3)
# Re-map so fronts come first, then rears
corner_map = [0, 2, 1, 3]

Fx_c, Fy_c, Fz_c = WC_corner['Fx'], WC_corner['Fy'], WC_corner['Fz']
Mx_c, My_c, Mz_c = WC_corner['Mx'], WC_corner['My'], WC_corner['Mz']

print_wc_header('Cornering')
for k, s in enumerate(sides_all):
    ci = corner_map[k]
    print(f"{s:<8} {Fx_c[ci]:>10.2f} {Fy_c[ci]:>10.2f} {Fz_c[ci]:>10.2f} "
          f"{Mx_c[ci]:>10.2f} {My_c[ci]:>10.2f} {Mz_c[ci]:>10.2f}")

print_pickup_header('Cornering')

corner = {}
for k, s in enumerate(sides_all):
    ci = corner_map[k]
    Fw = np.array([Fx_c[ci], Fy_c[ci], Fz_c[ci]])
    Mw = np.array([Mx_c[ci], My_c[ci], Mz_c[ci]])
    wc = wc_all[k]
    if k <= 1:
        corner[s] = solve_dwb(Fw, Mw, geom[s], wc)
        print_force_row(s, 'UW',   corner[s]['F_upperWishbone'])
        print_force_row(s, 'LW',   corner[s]['F_lowerWishbone'])
        print_force_row(s, 'TR',   corner[s]['F_tieRod'])
        print_force_row(s, 'Damp', corner[s]['F_damper'])
    else:
        corner[s] = solve_ta(Fw, Mw, geom[s], wc)
        print_force_row(s, 'Arm',  corner[s]['F_trailingArm'])
        print_force_row(s, 'TR',   corner[s]['F_tieRod'])
        print_force_row(s, 'Coil', corner[s]['F_coilover'])
        print_force_row(s, 'Rod',  corner[s]['F_rod'])


# =====================================================================
# Load case 3: 1G braking
# =====================================================================
print("\n========== BRAKING (Ax~1G) ==========")

Fx_b, Fy_b, Fz_b = WC_brake['Fx'], WC_brake['Fy'], WC_brake['Fz']
Mx_b, My_b, Mz_b = WC_brake['Mx'], WC_brake['My'], WC_brake['Mz']

print_wc_header('Braking')
for k, s in enumerate(sides_all):
    bi = corner_map[k]
    print(f"{s:<8} {Fx_b[bi]:>10.2f} {Fy_b[bi]:>10.2f} {Fz_b[bi]:>10.2f} "
          f"{Mx_b[bi]:>10.2f} {My_b[bi]:>10.2f} {Mz_b[bi]:>10.2f}")

print_pickup_header('Braking')

brake = {}
for k, s in enumerate(sides_all):
    bi = corner_map[k]
    Fw = np.array([Fx_b[bi], Fy_b[bi], Fz_b[bi]])
    Mw = np.array([Mx_b[bi], My_b[bi], Mz_b[bi]])
    wc = wc_all[k]
    if k <= 1:
        brake[s] = solve_dwb(Fw, Mw, geom[s], wc)
        print_force_row(s, 'UW',   brake[s]['F_upperWishbone'])
        print_force_row(s, 'LW',   brake[s]['F_lowerWishbone'])
        print_force_row(s, 'TR',   brake[s]['F_tieRod'])
        print_force_row(s, 'Damp', brake[s]['F_damper'])
    else:
        brake[s] = solve_ta(Fw, Mw, geom[s], wc)
        print_force_row(s, 'Arm',  brake[s]['F_trailingArm'])
        print_force_row(s, 'TR',   brake[s]['F_tieRod'])
        print_force_row(s, 'Coil', brake[s]['F_coilover'])
        print_force_row(s, 'Rod',  brake[s]['F_rod'])


# =====================================================================
# Tabulated summary
# =====================================================================
print("\n========== FULL SUMMARY (XYZ Components) ==========")
print(f"{'Load Case':<12} {'Corner':<8} {'Member':<6} "
      f"{'Fx (N)':>10} {'Fy (N)':>10} {'Fz (N)':>10}")
print('-' * 62)

lc_labels = ['2G Bump', 'Cornering', 'Braking']
all_data  = [bump, corner, brake]

for lc in range(3):
    d = all_data[lc]
    for k in range(2):
        s = sides_all[k]
        for member, key in [('UW',   'F_upperWishbone'),
                            ('LW',   'F_lowerWishbone'),
                            ('TR',   'F_tieRod'),
                            ('Damp', 'F_damper')]:
            F = d[s][key]
            print(f"{lc_labels[lc]:<12} {s:<8} {member:<6} "
                  f"{F[0]:>10.2f} {F[1]:>10.2f} {F[2]:>10.2f}")
    for k in range(2, 4):
        s = sides_all[k]
        for member, key in [('Arm',  'F_trailingArm'),
                            ('TR',   'F_tieRod'),
                            ('Coil', 'F_coilover'),
                            ('Rod',  'F_rod')]:
            F = d[s][key]
            print(f"{lc_labels[lc]:<12} {s:<8} {member:<6} "
                  f"{F[0]:>10.2f} {F[1]:>10.2f} {F[2]:>10.2f}")
    print('-' * 62)


# =====================================================================
# 3D Visualization
# =====================================================================
def plot_arm(ax, A, B, col):
    ax.plot([A[0], B[0]], [A[1], B[1]], [A[2], B[2]],
            '-o', color=col, linewidth=2, markersize=4, markerfacecolor=col)


def draw_force_arrow(ax, origin, F, col, scale):
    if np.linalg.norm(F) < 1e-6:
        return
    ax.quiver(origin[0], origin[1], origin[2],
              F[0] * scale, F[1] * scale, F[2] * scale,
              color=col, linewidth=1.8, arrow_length_ratio=0.2)


def set_axes_equal(ax):
    """Make 3D axes have equal scale (matplotlib doesn't do this natively)."""
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    centers = np.mean(limits, axis=1)
    radius  = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
    ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
    ax.set_zlim3d([centers[2] - radius, centers[2] + radius])


c_bump   = (0.20, 0.60, 1.00)
c_corner = (1.00, 0.45, 0.00)
c_brake  = (0.85, 0.15, 0.15)

case_names  = ['2G Bump', 'Cornering', 'Braking']
case_data   = [bump, corner, brake]
case_colors = [c_bump, c_corner, c_brake]

for lc in range(3):
    fig = plt.figure(figsize=(6, 6))
    fig.canvas.manager.set_window_title(case_names[lc])
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_title(case_names[lc], fontsize=13, fontweight='bold')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
    ax.view_init(elev=25, azim=45)

    col     = case_colors[lc]
    lc_data = case_data[lc]

    # Compute global scale so the largest force = 0.25m arrow
    max_len  = 0.25
    all_mags = []
    for k, s in enumerate(sides_all):
        d = lc_data[s]
        if k <= 1:
            all_mags.append(np.linalg.norm(d['F_upperWishbone']))
            all_mags.append(np.linalg.norm(d['F_lowerWishbone']))
            all_mags.append(np.linalg.norm(d['F_tieRod']))
            all_mags.append(np.linalg.norm(d['F_damper']))
        else:
            all_mags.append(np.linalg.norm(d['F_trailingArm']))
            all_mags.append(np.linalg.norm(d['F_tieRod']))
            all_mags.append(np.linalg.norm(d['F_coilover']))
            all_mags.append(np.linalg.norm(d['F_rod']))
    arrow_scale = max_len / max(all_mags)

    for k, s in enumerate(sides_all):
        wc  = wc_all[k]
        g_s = geom[s]
        d   = lc_data[s]

        if k <= 1:   # double wishbone front
            plot_arm(ax, g_s['UW_A'],   g_s['UW_C'],   (0.5, 0.5, 0.5))
            plot_arm(ax, g_s['UW_B'],   g_s['UW_C'],   (0.5, 0.5, 0.5))
            plot_arm(ax, g_s['LW_D'],   g_s['LW_F'],   (0.3, 0.3, 0.3))
            plot_arm(ax, g_s['LW_E'],   g_s['LW_F'],   (0.3, 0.3, 0.3))
            plot_arm(ax, g_s['TR_G'],   g_s['TR_H'],   (0.6, 0.3, 0.0))
            plot_arm(ax, g_s['Damp_L'], g_s['Damp_U'], (0.0, 0.6, 0.3))   # front damper
            draw_force_arrow(ax, g_s['UW_C'],   d['F_upperWishbone'], col, arrow_scale)
            draw_force_arrow(ax, g_s['LW_F'],   d['F_lowerWishbone'], col, arrow_scale)
            draw_force_arrow(ax, g_s['TR_H'],   d['F_tieRod'],        col, arrow_scale)
            draw_force_arrow(ax, g_s['Damp_L'], d['F_damper'],        col, arrow_scale)
        else:        # trailing arm rear with rocker mechanism
            # Trailing arm body and tie rod
            plot_arm(ax, g_s['P1'],   g_s['T'],    (0.5, 0.5, 0.5))
            plot_arm(ax, g_s['P2'],   g_s['T'],    (0.5, 0.5, 0.5))
            plot_arm(ax, g_s['TR_Q'], g_s['TR_S'], (0.6, 0.3, 0.0))
            # Rocker triangle (3 edges between N1, N2, N3)
            plot_arm(ax, g_s['Rkr_N1'], g_s['Rkr_N2'], (0.3, 0.2, 0.7))
            plot_arm(ax, g_s['Rkr_N2'], g_s['Rkr_N3'], (0.3, 0.2, 0.7))
            plot_arm(ax, g_s['Rkr_N3'], g_s['Rkr_N1'], (0.3, 0.2, 0.7))
            # Coilover (rocker N2 -> chassis) in green
            plot_arm(ax, g_s['Rkr_N2'], g_s['Coil_chassis'], (0.0, 0.6, 0.3))
            # Rod (rocker N3 -> chassis) in dark orange
            plot_arm(ax, g_s['Rkr_N3'], g_s['Rod_chassis'],  (0.85, 0.45, 0.0))
            # Force arrows
            draw_force_arrow(ax, g_s['T'],      d['F_trailingArm'], col, arrow_scale)
            draw_force_arrow(ax, g_s['TR_S'],   d['F_tieRod'],      col, arrow_scale)
            draw_force_arrow(ax, g_s['Rkr_N2'], d['F_coilover'],    col, arrow_scale)
            draw_force_arrow(ax, g_s['Rkr_N3'], d['F_rod'],         col, arrow_scale)

        ax.scatter(wc[0], wc[1], wc[2], color='k', s=40)

    set_axes_equal(ax)
    ax.grid(True)

plt.show()