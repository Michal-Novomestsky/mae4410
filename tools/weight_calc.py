import math
import warnings

from tools.isa import get_a
from tools.constants import *

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
def get_w_fuel_0(w_ij_chain, safety_factor):
    w_start_end = 1
    for w_i_j in w_ij_chain:
        w_start_end *= w_i_j

    return (1 + safety_factor) * (1 - w_start_end)

def get_w_empty_0(mtow):
    # Placeholder empty-weight fraction until a regression is fitted
    warnings.warn("Using constant empty-weight fraction placeholder", UserWarning)
    return 0.55


def mtow_iter(w_payload, w_fuel_0, w_empty_0):
    return w_payload/(1 - w_fuel_0 - w_empty_0)

def get_mtow(w_payload, w_ij_chain, w_empty_0_init, safety_factor, max_iters=10):
    mtow_prev = 0
    w_empty_0 = w_empty_0_init
    mtow_chain = {
        "i": [],
        "mtow": [],
        "w_empty_0": [],
        "w_fuel_0": [],
        "eps": [],
    }

    w_fuel_0 = get_w_fuel_0(w_ij_chain, safety_factor)
    
    # Iteratively solve
    for i in range(max_iters):
        mtow = mtow_iter(w_payload, w_fuel_0, w_empty_0)
        eps = (mtow - mtow_prev)/mtow

        w_empty_0 = get_w_empty_0(mtow)
        mtow_prev = mtow

        mtow_chain["i"].append(i)
        mtow_chain["mtow"].append(mtow)
        mtow_chain["w_empty_0"].append(w_empty_0)
        mtow_chain["w_fuel_0"].append(w_fuel_0)
        mtow_chain["eps"].append(eps)

    return mtow_chain
