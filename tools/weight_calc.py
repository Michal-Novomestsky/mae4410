import math
from multiprocessing import Value
import warnings

from tools.isa import get_a
from tools.constants import *

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

# TODO find the right name for this (w_payload + w_crew) - what is it? It's not OEW
def get_w_payload(safety_factor):
    raise NotImplementedError

# TODO Should this just be a const instead?
def get_w_ij_warmup_takeoff():
    return 0.970

def get_w_ij_climb():
    return 0.985

def get_w_ij_descent():
    warnings.warn("Assuming descent fuel consumption equal to climb", UserWarning)
    return get_w_ij_climb()

def get_w_ij_landing():
    return 0.995

def get_w_ij_cruise(R, c_t, M, L_D, z):
    a = get_a(z)
    V = M * a

    warnings.warn("Assuming plane cruising at 0.866(L/D)", UserWarning)
    L_D *= 0.866

    return math.exp(-R * G_GRAV * c_t / (V * L_D))

def get_w_ij_loiter(E, c_t, L_D):
    return math.exp(-E * G_GRAV * c_t /L_D)

# TODO better way to do this?
def get_w_fuel_0(w_flight_profile, safety_factor):
    w_start_end = 1
    for w_i_j in w_flight_profile:
        w_start_end *= w_i_j

    return (1 + safety_factor) * (1 - w_start_end)

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

def get_mtow(w_payload, w_flight_profile, safety_factor, oew=None, w_empty_0_init=0.5, max_iters=10):
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
    if oew is None:
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

def get_weight_calcs(specs, specs_path, flight_profile):
    # TODO LOAD FROM specs
    W_PAYLOAD = 57493  # Estimate from Michal's W1 work
    SAFETY_FACTOR_FUEL = 0.05  # 5% trip fuel contingency
    MAX_ITERS = 10

    oew, weight_manifest = load_oew_from_specs(specs)
    if oew is None:
        warnings.warn(
            f"No mass entries found in {specs_path}; "
            "falling back to iterative OEW solution",
            UserWarning,
        )

    mtow_chain = get_mtow(
        w_payload=W_PAYLOAD,
        w_flight_profile=flight_profile,
        safety_factor=SAFETY_FACTOR_FUEL,
        oew=oew,
        max_iters=MAX_ITERS,
    )

    weight_calcs = {
        "mtow": mtow_chain["mtow"][-1],
        "w_payload": mtow_chain["w_payload"][-1],
        "w_fuel": mtow_chain["w_fuel"][-1],
        "oew": mtow_chain["oew"][-1],
        "oew_breakdown": weight_manifest,
        "w_fuel_0": mtow_chain["w_fuel_0"][-1],
        "w_empty_0": mtow_chain["w_empty_0"][-1],
        "eps": mtow_chain["eps"][-1],
    }

    return weight_calcs