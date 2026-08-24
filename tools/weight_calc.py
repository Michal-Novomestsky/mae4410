import math
import warnings

from tools.isa import get_a
from data.constants import *

def _collect_mass_entries(node, path=(), manifest=None):
    """Recursively collect component weights from mass sections."""
    if manifest is None:
        manifest = {}
    if not isinstance(node, dict):
        return manifest

    mass = node.get("mass")
    if isinstance(mass, dict) and "weight" in mass:
        component = ".".join(path) if path else "aircraft"
        manifest[component] = mass["weight"]

    for name, child in node.items():
        if name not in ("mass", "units"):
            _collect_mass_entries(child, path + (name,), manifest)

    return manifest

def load_oew_from_specs(specs):
    """Return the summed OEW and component weight manifest."""
    weight_manifest = _collect_mass_entries(specs)
    if not weight_manifest:
        return None, {}
    return sum(weight_manifest.values()), weight_manifest

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
    L_D *= 0.866

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

    weight_calcs = {
        "mtow": mtow_chain["mtow"][-1],
        "w_payload": mtow_chain["w_payload"][-1],
        "w_fuel": mtow_chain["w_fuel"][-1],
        "oew": mtow_chain["oew"][-1],
        "w_fuel_0": mtow_chain["w_fuel_0"][-1],
        "w_empty_0": mtow_chain["w_empty_0"][-1],
        "eps": mtow_chain["eps"][-1],
        "flight_breakdown": {flight_profile[i][0]: flight_weights[i] for i in range(len(flight_profile))}
    }

    if not use_raymer:
        weight_calcs["oew_breakdown"] = weight_manifest

    return weight_calcs