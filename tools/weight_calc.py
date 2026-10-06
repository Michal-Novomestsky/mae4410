import math
import warnings
from pathlib import Path

import matplotlib.pyplot as plt

from tools.isa import get_a
from data.constants import *

# Cruise Breguet uses L/D at max-range condition (Raymer / Roskam 0.866 factor).
# V_cruise already embeds the 1.316 V*/V cruise-altitude choice from get_lift_calcs.
CRUISE_LD_FACTOR = 0.866

# Variable loads: CG stations only; mass applied via w_fuel / w_payload args.
VARIABLE_MASS_COMPONENTS = frozenset({"wings.ftank", "fuselage.payload"})

def _collect_mass_entries(node, path=(), manifest=None):
    """Recursively collect mass section fields (weight, CGx/y/z) from specs."""
    if manifest is None:
        manifest = {}
    if not isinstance(node, dict):
        return manifest

    mass = node.get("mass")
    if isinstance(mass, dict):
        entry = {}
        if "weight" in mass:
            entry["weight"] = mass["weight"]
        for axis in ("CGx", "CGy", "CGz"):
            if axis in mass:
                entry[axis] = mass[axis]
        if entry:
            component = ".".join(path) if path else "aircraft"
            manifest[component] = entry

    for name, child in node.items():
        if name not in ("mass", "units"):
            _collect_mass_entries(child, path + (name,), manifest)

    return manifest

def load_oew_from_specs(specs):
    """Return the summed OEW and component weight manifest (excludes payload/fuel)."""
    mass_manifest = _collect_mass_entries(specs)
    weight_manifest = {
        component: entry["weight"]
        for component, entry in mass_manifest.items()
        if "weight" in entry and component not in VARIABLE_MASS_COMPONENTS
    }
    if not weight_manifest:
        return None, {}
    return sum(weight_manifest.values()), weight_manifest

def load_payload_weight_from_specs(specs):
    """Design payload mass from fuselage.payload."""
    mass_manifest = _collect_mass_entries(specs)
    payload = mass_manifest.get("fuselage.payload")
    if payload is None or "weight" not in payload:
        raise ValueError("fuselage.payload.mass.weight is missing from specs")
    return payload["weight"]

def _add_variable_mass(moment, total_mass, mass_manifest, component, weight, cg_axes):
    """Add a variable load (fuel/payload) at its CG station."""
    if not weight:
        return total_mass
    entry = mass_manifest.get(component)
    if entry is None:
        raise ValueError(f"{component} mass entry is missing from specs")
    missing = [axis for axis in cg_axes if axis not in entry]
    if missing:
        raise ValueError(
            f"{component} is missing required CG fields: {', '.join(missing)}"
        )
    total_mass += weight
    for axis in cg_axes:
        moment[axis] += weight * entry[axis]
    return total_mass

def get_cg(specs, w_fuel=0.0, w_payload=0.0):
    """Mass-weighted CG from OEW components plus optional fuel and payload."""
    mass_manifest = _collect_mass_entries(specs)
    cg_axes = ("CGx", "CGy", "CGz")

    moment = {axis: 0.0 for axis in cg_axes}
    total_mass = 0.0

    for component, entry in mass_manifest.items():
        # Fuel/payload are CG stations only; mass applied via args below.
        if component in VARIABLE_MASS_COMPONENTS:
            continue

        missing = [field for field in ("weight", *cg_axes) if field not in entry]
        if missing:
            raise ValueError(
                f"Mass entry '{component}' is missing required fields: {', '.join(missing)}"
            )

        weight = entry["weight"]
        total_mass += weight
        for axis in cg_axes:
            moment[axis] += weight * entry[axis]

    total_mass = _add_variable_mass(
        moment, total_mass, mass_manifest, "wings.ftank", w_fuel, cg_axes
    )
    total_mass = _add_variable_mass(
        moment, total_mass, mass_manifest, "fuselage.payload", w_payload, cg_axes
    )

    if total_mass == 0:
        raise ValueError("No mass contributions available to compute CG")

    return {axis: moment[axis] / total_mass for axis in cg_axes}

# J. Roskam, Airplane Design: Preliminary sizing of airplanes, DARCorporation, 1985.
def get_w_ij_warmup():
    return 0.990

def get_w_ij_taxi():
    return 0.990

def get_w_ij_takeoff():
    return 0.995

def get_w_ij_climb():
    return 0.980

def get_w_ij_cruise(R, c_t, M, L_D, z):
    a = get_a(z)
    V = M * a
    L_D *= CRUISE_LD_FACTOR

    return math.exp(-R * G_GRAV * c_t / (V * L_D))

def get_w_ij_loiter(E, c_t, L_D):
    return math.exp(-E * G_GRAV * c_t /L_D)

def get_w_ij_descent():
    return 0.990

def get_w_ij_landing_shutdown():
    return 0.992

def get_w_fuel_0(w_flight_profile, safety_factor):
    w_start_end = 1
    for w_i_j in w_flight_profile:
        w_start_end *= w_i_j[1]

    return (1 + safety_factor) * (1 - w_start_end)

def get_flight_weights(w_flight_profile, mtow):
    weights = [mtow]
    for w_i_j in w_flight_profile:
        w_prev = weights[-1]
        weights.append(w_prev * w_i_j[1])

    return weights[1:] # Chop out initial value (MTOW)

def get_w_empty_0(mtow):
    # TODO add other correlations, not just Raymer
    A = 0.97
    C = -0.06
    return A*mtow**C

def mtow_iter(w_payload, w_fuel_0, w_empty_0=None, oew=None):
    if oew is None:
        return w_payload/(1 - w_fuel_0 - w_empty_0)
    if w_empty_0 is not None:
        raise ValueError(f"OEW not None ({round(oew, 2)} and empty weight fraction not None ({round(w_empty_0, 2)}. Specify one or the other.")
    return (w_payload + oew)/(1 - w_fuel_0) 

def get_mtow(w_payload, w_flight_profile, safety_factor, oew=None, use_raymer=False, w_empty_0_init=0.5, max_iters=10):
    mtow_chain = {
        "i": [],
        "mtow": [],
        "w_payload": [],
        "w_empty_0": [],
        "oew": [],
        "w_fuel_0": [],
        "w_fuel": [],
        "eps": [],
    }

    w_fuel_0 = get_w_fuel_0(w_flight_profile, safety_factor)
    
    # Iteratively solve for OEW using Raymer
    if use_raymer or oew is None:
        mtow_prev = 0
        w_empty_0 = w_empty_0_init
        
        for i in range(max_iters):
            mtow = mtow_iter(w_payload, w_fuel_0, w_empty_0)
            eps = abs((mtow - mtow_prev)/mtow)

            w_empty_0 = get_w_empty_0(mtow)
            mtow_prev = mtow

            mtow_chain["i"].append(i)
            mtow_chain["mtow"].append(mtow)
            mtow_chain["w_payload"].append(w_payload)
            mtow_chain["w_empty_0"].append(w_empty_0)
            mtow_chain["oew"].append(w_empty_0*mtow)
            mtow_chain["w_fuel_0"].append(w_fuel_0)
            mtow_chain["w_fuel"].append(w_fuel_0*mtow)
            mtow_chain["eps"].append(eps)

    # Get OEW from CAD
    else:
        mtow = mtow_iter(w_payload, w_fuel_0, oew=oew)
        mtow_chain["i"].append(0)
        mtow_chain["mtow"].append(mtow)
        mtow_chain["w_payload"].append(w_payload)
        mtow_chain["w_empty_0"].append(oew/mtow)
        mtow_chain["oew"].append(oew)
        mtow_chain["w_fuel_0"].append(w_fuel_0)
        mtow_chain["w_fuel"].append(w_fuel_0*mtow)
        mtow_chain["eps"].append(0)

    return mtow_chain

def get_weight_calcs(specs, specs_path, flight_profile, w_payload, safety_factor_fuel, use_raymer=False, max_iters=10):

    oew, weight_manifest = load_oew_from_specs(specs)
    if oew is None:
        warnings.warn(
            f"No OEW entries found in specs.json. Falling back to iterative Raymer solution",
            UserWarning,
        )

    mtow_chain = get_mtow(
        w_payload=w_payload,
        w_flight_profile=flight_profile,
        safety_factor=safety_factor_fuel,
        oew=oew,
        use_raymer=use_raymer,
        max_iters=max_iters,
    )

    flight_weights = get_flight_weights(flight_profile, mtow_chain["mtow"][-1])
    cg = get_cg(
        specs,
        w_fuel=mtow_chain["w_fuel"][-1],
        w_payload=mtow_chain["w_payload"][-1],
    )

    weight_calcs = {
        "CGx": cg["CGx"],
        "CGy": cg["CGy"],
        "CGz": cg["CGz"],
        "mtow": mtow_chain["mtow"][-1],
        "w_payload": mtow_chain["w_payload"][-1],
        "w_fuel": mtow_chain["w_fuel"][-1],
        "oew": mtow_chain["oew"][-1],
        "w_fuel_0": mtow_chain["w_fuel_0"][-1],
        "w_empty_0": mtow_chain["w_empty_0"][-1],
        "eps": mtow_chain["eps"][-1],
        "flight_breakdown": {
                               "weights": {flight_profile[i][0]: flight_weights[i] for i in range(len(flight_profile))},
                               "weight_fractions": {flight_profile[i][0]: flight_profile[i][1] for i in range(len(flight_profile))}
        }
    }

    if not use_raymer:
        weight_calcs["oew_breakdown"] = weight_manifest

    return weight_calcs


def breguet_range_m(V, L_D, W0, W1, c_t=C_T, g=G_GRAV):
    return (V * L_D / (g * c_t)) * math.log(W0 / W1)


def _mission_pi_pre_post(weight_fractions):
    """Fixed non-cruise weight-fraction products around the cruise segment."""
    pre_keys = ("warmup", "taxi", "takeoff", "climb")
    post_keys = ("loiter", "descent", "landing and shutdown")
    pi_pre = math.prod(weight_fractions[k] for k in pre_keys)
    pi_post = math.prod(weight_fractions[k] for k in post_keys)
    return pi_pre, pi_post


def cruise_range_from_fuel(W_TO, W_fuel, pi_pre, pi_post, V, L_D, safety_factor):
    """
    Cruise-only Breguet range with fixed pre/post mission fractions.

    Fuel loaded includes contingency: W_fuel = (1+sf)*(W_TO - W_land).
    Only the cruise ratio varies with available fuel.
    """
    w_cr = (1.0 - W_fuel / (W_TO * (1.0 + safety_factor))) / (pi_pre * pi_post)
    if not (0.0 < w_cr < 1.0):
        raise ValueError(
            f"Invalid cruise weight fraction w_cr={w_cr:.6f} for "
            f"W_TO={W_TO:.1f} kg, W_fuel={W_fuel:.1f} kg"
        )
    W_cr_i = W_TO * pi_pre
    W_cr_f = W_cr_i * w_cr
    return breguet_range_m(V, L_D, W_cr_i, W_cr_f)


def write_payload_range_diagram(
    weight_calcs,
    lift_calcs,
    w_fuel_max,
    out_path,
    w_payload_design=None,
    safety_factor=SAFETY_FACTOR_FUEL,
):
    mtow = weight_calcs["mtow"]
    oew = weight_calcs["oew"]
    w_fuel = weight_calcs["w_fuel"]
    if w_payload_design is None:
        w_payload_design = weight_calcs["w_payload"]
    weight_fractions = weight_calcs["flight_breakdown"]["weight_fractions"]

    if w_fuel > w_fuel_max:
        raise ValueError(
            f"Design fuel ({w_fuel:.1f} kg) exceeds tank capacity "
            f"w_fuel_max ({w_fuel_max:.1f} kg)"
        )

    V = lift_calcs["V_cruise"]
    L_D = lift_calcs["(c_L/c_D)_star"] * CRUISE_LD_FACTOR
    pi_pre, pi_post = _mission_pi_pre_post(weight_fractions)

    w_payload_max_range = mtow - oew - w_fuel_max
    if w_payload_max_range < 0:
        raise ValueError(
            f"Full tanks ({w_fuel_max:.1f} kg) exceed MTOW−OEW "
            f"({mtow - oew:.1f} kg); cannot form max-range corner"
        )

    # (label, range_m, payload_kg)
    points = [
        ("Zero range", 0.0, w_payload_design),
        (
            "Harmonic",
            cruise_range_from_fuel(
                mtow, w_fuel, pi_pre, pi_post, V, L_D, safety_factor
            ),
            w_payload_design,
        ),
        (
            "Max range",
            cruise_range_from_fuel(
                mtow, w_fuel_max, pi_pre, pi_post, V, L_D, safety_factor
            ),
            w_payload_max_range,
        ),
        (
            "Ultimate",
            cruise_range_from_fuel(
                oew + w_fuel_max,
                w_fuel_max,
                pi_pre,
                pi_post,
                V,
                L_D,
                safety_factor,
            ),
            0.0,
        ),
    ]

    ranges_km = [p[1] / 1e3 for p in points]
    payloads_t = [p[2] / 1e3 for p in points]
    labels = [p[0] for p in points]

    fig, ax = plt.subplots(figsize=(9.5, 5.8), layout="constrained")
    ax.plot(
        ranges_km,
        payloads_t,
        color="#1f4e79",
        linewidth=2.2,
        marker="o",
        markersize=7,
        markerfacecolor="#f4a261",
        markeredgecolor="#1f4e79",
        markeredgewidth=1.4,
        zorder=3,
    )
    ax.fill_between(ranges_km, payloads_t, color="#1f4e79", alpha=0.08, zorder=1)

    # Stagger labels so near-coincident harmonic / max-range corners stay readable.
    label_offsets = [
        (10, 12),
        (-95, 14),
        (12, -28),
        (-20, 14),
    ]
    for (x, y, label), offset in zip(
        zip(ranges_km, payloads_t, labels), label_offsets
    ):
        ax.annotate(
            f"{label}\n{x:.0f} km, {y:.1f} t",
            xy=(x, y),
            xytext=offset,
            textcoords="offset points",
            fontsize=8.5,
            color="#274c77",
            ha="left",
            va="bottom",
            bbox={
                "boxstyle": "round,pad=0.25",
                "facecolor": "white",
                "edgecolor": "#c9d6e3",
                "alpha": 0.92,
            },
            arrowprops={
                "arrowstyle": "-",
                "color": "#9bb0c4",
                "lw": 0.8,
            },
            zorder=4,
        )

    rfp_range_km = R / 1e3
    ax.axvline(
        rfp_range_km,
        color="#e76f51",
        linestyle="--",
        linewidth=1.2,
        alpha=0.85,
        label=f"RFP range ({rfp_range_km:.0f} km)",
        zorder=2,
    )

    ax.set_xlabel("Range (km)")
    ax.set_ylabel("Payload (t)")
    ax.set_title("Payload–Range Diagram")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.grid(True, which="major", linestyle=":", linewidth=0.8, alpha=0.7)
    ax.legend(loc="upper right", framealpha=0.95)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)

    return {
        "points": [
            {"label": lab, "range_km": r_km, "payload_t": p_t}
            for lab, r_km, p_t in zip(labels, ranges_km, payloads_t)
        ],
        "path": str(out_path),
    }


def _cg_limits_m(np_mm, mac_mm, sm_min=SM_MIN, sm_max=SM_MAX):
    """Forward/aft CG limits [m] from NP and static-margin band."""
    x_fwd_m = (np_mm - sm_max * mac_mm) * 1e-3
    x_aft_m = (np_mm - sm_min * mac_mm) * 1e-3
    return x_fwd_m, x_aft_m


def _state_point(specs, oew, w_payload, w_fuel, label=None):
    """Single (CG, W) state from OEW + payload + fuel."""
    weight_kg = oew + w_payload + w_fuel
    cg = get_cg(specs, w_fuel=w_fuel, w_payload=w_payload)
    point = {
        "weight_t": weight_kg / 1e3,
        "cg_m": cg["CGx"] * 1e-3,
        "w_payload_kg": w_payload,
        "w_fuel_kg": w_fuel,
    }
    if label is not None:
        point["label"] = label
    return point


def sample_tank_fill(specs, oew, w_payload, w_fuel_start, w_fuel_end, n_points=41):
    """
    CG vs weight while changing wing fuel at fixed payload.

    With a single tank CG station:
    x_CG(W) = x_f + W_ZFW (x_ZFW - x_f) / W
    """
    if n_points < 2:
        raise ValueError(f"n_points must be >= 2, got {n_points}")
    fuels = [
        w_fuel_start + (w_fuel_end - w_fuel_start) * i / (n_points - 1)
        for i in range(n_points)
    ]
    return [_state_point(specs, oew, w_payload, wf) for wf in fuels]


def _box_corner(cg_m, weight_t):
    """Synthetic boundary point for rectilinear payload edges (not a load state)."""
    return {
        "cg_m": cg_m,
        "weight_t": weight_t,
        "w_payload_kg": None,
        "w_fuel_kg": None,
    }


def box_payload_edge(p_start, p_end):
    """
    Rectilinear edge between two corner states (conservative payload bound).

    Knee at (end CG, start weight): horizontal at start weight, then vertical
    at end CG (e.g. OEW → OEW+payload uses payload CG at OEW weight).
    """
    knee = _box_corner(p_end["cg_m"], p_start["weight_t"])
    return [p_start, knee, p_end]


def sample_mtow_trade(
    specs, oew, mtow, w_payload_start, w_payload_end, n_points=41
):
    """Constant-MTOW edge: W_p + W_f = MTOW − OEW."""
    if n_points < 2:
        raise ValueError(f"n_points must be >= 2, got {n_points}")
    samples = []
    for i in range(n_points):
        frac = i / (n_points - 1)
        wp = w_payload_start + frac * (w_payload_end - w_payload_start)
        wf = mtow - oew - wp
        samples.append(_state_point(specs, oew, wp, wf))
    return samples


def loading_hull_boundary(
    specs, oew, mtow, w_payload_max, w_fuel_max, n_points=41
):
    """
    Closed boundary of loading hull H (conservative, certification-style).

    Fuel edges: tank fill / drain (fixed tank CG). Payload edges: rectilinear
    box steps between corner vertices (not single-station payload hyperbolas).

    Walk from OEW:
      ZFW box edge → max-payload tank fill → MTOW trade →
      full-fuel box edge → zero-payload tank drain.
    """
    w_fuel_design = mtow - oew - w_payload_max
    w_payload_full = mtow - oew - w_fuel_max
    if w_fuel_design < 0:
        raise ValueError(
            f"Design payload ({w_payload_max:.1f} kg) exceeds MTOW−OEW "
            f"({mtow - oew:.1f} kg)"
        )
    if w_payload_full < 0:
        raise ValueError(
            f"Full tanks ({w_fuel_max:.1f} kg) exceed MTOW−OEW "
            f"({mtow - oew:.1f} kg)"
        )
    if w_fuel_design > w_fuel_max + 1e-6:
        raise ValueError(
            f"Design fuel ({w_fuel_design:.1f} kg) exceeds tank capacity "
            f"({w_fuel_max:.1f} kg)"
        )

    mtow_full_tanks = _state_point(specs, oew, w_payload_full, w_fuel_max)

    corners = [
        _state_point(specs, oew, 0.0, 0.0, "OEW"),
        _state_point(specs, oew, w_payload_max, 0.0, "OEW + payload"),
        _state_point(specs, oew, w_payload_max, w_fuel_design, "Design MTOW"),
        _state_point(specs, oew, 0.0, w_fuel_max, "OEW + full tanks"),
    ]

    segments = [
        box_payload_edge(corners[0], corners[1]),
        sample_tank_fill(specs, oew, w_payload_max, 0.0, w_fuel_design, n_points),
        sample_mtow_trade(
            specs, oew, mtow, w_payload_max, w_payload_full, n_points
        ),
        box_payload_edge(mtow_full_tanks, corners[3]),
        sample_tank_fill(specs, oew, 0.0, w_fuel_max, 0.0, n_points),
    ]

    boundary = []
    for i, seg in enumerate(segments):
        pts = seg if i == 0 else seg[1:]
        boundary.extend(pts)
    return boundary, corners


def _clip_poly_to_rect(vertices, x_min, x_max, y_min, y_max):
    """Sutherland–Hodgman clip of polygon [(x,y), ...] to axis-aligned rectangle."""

    def _inside(p, edge):
        x, y = p
        if edge == "left":
            return x >= x_min
        if edge == "right":
            return x <= x_max
        if edge == "bottom":
            return y >= y_min
        return y <= y_max  # top

    def _intersect(p1, p2, edge):
        x1, y1 = p1
        x2, y2 = p2
        dx, dy = x2 - x1, y2 - y1
        if edge == "left":
            t = (x_min - x1) / dx if dx != 0 else 0.0
            return (x_min, y1 + t * dy)
        if edge == "right":
            t = (x_max - x1) / dx if dx != 0 else 0.0
            return (x_max, y1 + t * dy)
        if edge == "bottom":
            t = (y_min - y1) / dy if dy != 0 else 0.0
            return (x1 + t * dx, y_min)
        t = (y_max - y1) / dy if dy != 0 else 0.0
        return (x1 + t * dx, y_max)

    out = list(vertices)
    for edge in ("left", "right", "bottom", "top"):
        if not out:
            return []
        inp = out
        out = []
        s = inp[-1]
        for e in inp:
            if _inside(e, edge):
                if not _inside(s, edge):
                    out.append(_intersect(s, e, edge))
                out.append(e)
            elif _inside(s, edge):
                out.append(_intersect(s, e, edge))
            s = e
    return out


def write_weight_balance_diagram(
    specs,
    weight_calcs,
    w_fuel_max,
    out_path,
    sm_min=SM_MIN,
    sm_max=SM_MAX,
    n_boundary=41,
):
    """
    Weight–CG diagram: certified region H ∩ E.

    H  = loading hull (boxy payload edges + hyperbolic fuel fills), W≤MTOW
    E  = SM envelope [OEW, MTOW] × [x_fwd, x_aft]

    CG axis in metres from nose; weight axis in tonnes.
    """
    mtow = weight_calcs["mtow"]
    oew = weight_calcs["oew"]
    w_payload = weight_calcs["w_payload"]
    w_fuel = weight_calcs["w_fuel"]

    wing_geom = specs["wings"]["main"]["geometry"]
    np_mm = wing_geom["NP"]
    mac_mm = wing_geom["MAC"]
    x_fwd_m, x_aft_m = _cg_limits_m(np_mm, mac_mm, sm_min=sm_min, sm_max=sm_max)

    w_fuel_design = mtow - oew - w_payload
    if abs(w_fuel_design - w_fuel) > 1.0:
        raise ValueError(
            f"Design fuel ({w_fuel:.1f} kg) inconsistent with MTOW−OEW−payload "
            f"({w_fuel_design:.1f} kg)"
        )

    boundary_h, corners = loading_hull_boundary(
        specs,
        oew,
        mtow,
        w_payload_max=w_payload,
        w_fuel_max=w_fuel_max,
        n_points=n_boundary,
    )

    oew_t = oew / 1e3
    mtow_t = mtow / 1e3
    poly_h = [(p["cg_m"], p["weight_t"]) for p in boundary_h]
    poly_cert = _clip_poly_to_rect(poly_h, x_fwd_m, x_aft_m, oew_t, mtow_t)

    envelope_cg = [x_fwd_m, x_aft_m, x_aft_m, x_fwd_m, x_fwd_m]
    envelope_w = [oew_t, oew_t, mtow_t, mtow_t, oew_t]
    h_cgs = [p[0] for p in poly_h] + [poly_h[0][0]]
    h_ws = [p[1] for p in poly_h] + [poly_h[0][1]]

    fig, ax = plt.subplots(figsize=(9.5, 5.8), layout="constrained")

    # SM envelope E
    ax.fill(
        envelope_cg,
        envelope_w,
        color="#2a9d8f",
        alpha=0.08,
        zorder=1,
        label="SM envelope (E)",
    )
    ax.plot(
        envelope_cg,
        envelope_w,
        color="#2a9d8f",
        linewidth=1.5,
        linestyle="--",
        zorder=2,
    )

    # Loading hull H outline (dashed; certified fill sits on top inside E)
    ax.plot(
        h_cgs,
        h_ws,
        color="#1f4e79",
        linewidth=1.6,
        linestyle="--",
        zorder=3,
        label="Loading hull (H)",
    )

    # Certified H ∩ E (fill + edge only; no duplicate solid trace)
    if len(poly_cert) >= 3:
        cert_cgs = [p[0] for p in poly_cert] + [poly_cert[0][0]]
        cert_ws = [p[1] for p in poly_cert] + [poly_cert[0][1]]
        ax.fill(
            cert_cgs,
            cert_ws,
            facecolor="#1f4e79",
            edgecolor="#1f4e79",
            alpha=0.28,
            linewidth=2.2,
            zorder=4,
            label=r"Certified (H $\cap$ E)",
        )

    ax.plot(
        [p["cg_m"] for p in corners],
        [p["weight_t"] for p in corners],
        linestyle="None",
        marker="o",
        markersize=7,
        markerfacecolor="#f4a261",
        markeredgecolor="#1f4e79",
        markeredgewidth=1.4,
        zorder=6,
    )

    label_offsets = [
        (8, -22),
        (8, 10),
        (-100, 12),
        (-110, -8),
    ]
    for p, offset in zip(corners, label_offsets):
        ax.annotate(
            f"{p['label']}\n{p['cg_m']:.2f} m, {p['weight_t']:.1f} t",
            xy=(p["cg_m"], p["weight_t"]),
            xytext=offset,
            textcoords="offset points",
            fontsize=8.5,
            color="#274c77",
            ha="left",
            va="bottom",
            bbox={
                "boxstyle": "round,pad=0.25",
                "facecolor": "white",
                "edgecolor": "#c9d6e3",
                "alpha": 0.92,
            },
            arrowprops={
                "arrowstyle": "-",
                "color": "#9bb0c4",
                "lw": 0.8,
            },
            zorder=7,
        )

    ax.axvline(x_fwd_m, color="#e76f51", linestyle=":", linewidth=1.1, alpha=0.9, zorder=2)
    ax.axvline(x_aft_m, color="#e9c46a", linestyle=":", linewidth=1.1, alpha=0.9, zorder=2)

    ax.set_xlabel("CG (m from nose)")
    ax.set_ylabel("Weight (t)")
    ax.set_title("Weight and Balance Diagram")
    ax.grid(True, which="major", linestyle=":", linewidth=0.8, alpha=0.7)
    ax.legend(loc="best", framealpha=0.95)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=160)
    plt.close(fig)

    return {
        "envelope": {
            "cg_fwd_m": x_fwd_m,
            "cg_aft_m": x_aft_m,
            "w_min_t": oew_t,
            "w_max_t": mtow_t,
            "sm_min": sm_min,
            "sm_max": sm_max,
            "np_m": np_mm * 1e-3,
            "mac_m": mac_mm * 1e-3,
        },
        "points": corners,
        "certified": {
            "n_vertices": len(poly_cert),
            "cg_min_m": min((p[0] for p in poly_cert), default=None),
            "cg_max_m": max((p[0] for p in poly_cert), default=None),
            "w_min_t": min((p[1] for p in poly_cert), default=None),
            "w_max_t": max((p[1] for p in poly_cert), default=None),
        },
        "path": str(out_path),
    }