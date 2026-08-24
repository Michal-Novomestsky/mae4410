import math

from scipy.optimize import root_scalar

from data.constants import FT2M, G_GRAV
from tools.isa import get_a, get_rho

def get_K(AR, e):
    return 1/(math.pi * AR * e)

def get_c_L_star(AR, e, c_D0):
    K = get_K(AR, e)
    return math.sqrt(c_D0/K)

def get_c_D_star(AR, e, c_D0):
    K = get_K(AR, e)
    c_L_star = get_c_L_star(AR, e, c_D0)
    return 2 * K * c_L_star**2

def get_c_L_c_D_star(AR, e, c_D0):
    c_L_star = get_c_L_star(AR, e, c_D0)
    c_D_star = get_c_D_star(AR, e, c_D0)
    return c_L_star/c_D_star

def get_V_star(AR, e, c_D0, rho, wing_loading):
    c_L_star = get_c_L_star(AR, e, c_D0)
    return math.sqrt(2 * wing_loading / (rho * c_L_star))

def get_alt(AR, e, c_D0, M_cruise, wing_loading, V_factor=1.0, z_min=0.0, z_max=20e3):
    "Compute the altitude z such that M_cruise * a == V_factor * V_star. DOESN'T ACCOUNT FOR M_DD"
    def residual(z):
        rho = get_rho(z)
        a = get_a(z)
        V_star = get_V_star(AR, e, c_D0, rho, wing_loading)
        V_cruise = M_cruise * a
        return V_factor * V_star - V_cruise

    sol = root_scalar(residual, bracket=(z_min, z_max))
    if not sol.converged:
        raise RuntimeError(f"Altitude root find failed: {sol.flag}")
    return sol.root

def get_cruise_alt(AR, e, c_D0, M_cruise, wing_loading, z_min=0.0, z_max=20e3):
    '''Compute the altitude z such that M_cruise * a == 1.316 * V_star'''
    return get_alt(AR, e, c_D0, M_cruise, wing_loading, V_factor=1.316)
    
def get_ceiling_alt(AR, e, c_D0, M_max_cruise, wing_loading, z_min=0.0, z_max=20e3):
    '''Compute the altitude z such that M_cruise * a == V_star'''
    return get_alt(AR, e, c_D0, M_max_cruise, wing_loading, V_factor=1.0)

def get_lift_calcs(specs, e, c_D0, M_max_cruise, mtow):
    specs = specs["wings"]["main"]["geometry"]
    AR = specs["AR"]
    span = specs["span"] * 1e-3 # mm to m

    S_wing = span**2 / AR
    wing_loading = mtow * G_GRAV / S_wing

    z_cruise = get_cruise_alt(AR, e, c_D0, M_max_cruise, wing_loading)
    z_ceil = get_ceiling_alt(AR, e, c_D0, M_max_cruise, wing_loading)

    lift_calcs = {
        "AR": AR,
        "span": span,
        "S_wing": S_wing,
        "wing_loading": wing_loading,
        "K": get_K(AR, e),
        "e": e,
        "c_D0": c_D0,
        "c_L_star": get_c_L_star(AR, e, c_D0),
        "c_D_star": get_c_D_star(AR, e, c_D0),
        "(c_L/c_D)_star": get_c_L_c_D_star(AR, e, c_D0),
        "V_cruise": M_max_cruise * get_a(z_cruise),
        "M_cruise": M_max_cruise,
        "cruise_alt (ft)": z_cruise / FT2M,
        "cruise_ceiling (ft)": z_ceil / FT2M,
    }

    return lift_calcs
