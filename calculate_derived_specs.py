import json
import warnings
from pathlib import Path

from tools.constants import FT2M
from tools.weight_calc import (
    get_weight_calcs,
    get_w_ij_climb,
    get_w_ij_cruise,
    get_w_ij_descent,
    get_w_ij_landing,
    get_w_ij_loiter,
    get_w_ij_warmup_takeoff,
)

ROOT = Path(__file__).resolve().parent
SPECS_PATH = ROOT / "data" / "specs.json"
DERIVED_SPECS_PATH = ROOT / "data" / "specs_derived.json"

C_T = 14e-6  # kg/Ns Estimate from slides
M = 0.85
L_D = 20  # Approximation from slides

R = 8000e3  # 8000 km
E_LOITER = 30 * 60  # 30 min
LOITER_ALT = 1500 * FT2M
CRUISE_ALT = 35000 * FT2M

def calculate_derived_specs(
    specs_path=SPECS_PATH,
    out_path=DERIVED_SPECS_PATH,
):
    with open(specs_path) as specs_file:
        specs = json.load(specs_file)

    warnings.warn(
        "Loiter altitude specified in RFP, but not used in endurance eqn",
        UserWarning,
    )
    flight_profile = [
        get_w_ij_warmup_takeoff(),
        get_w_ij_climb(),
        get_w_ij_cruise(R, C_T, M, L_D, CRUISE_ALT),
        get_w_ij_loiter(E_LOITER, C_T, L_D),
        get_w_ij_descent(),
        get_w_ij_landing(),
    ]

    weight_calcs = get_weight_calcs(specs, specs_path, flight_profile)
    
    derived_specs = {
        "weight_calcs": weight_calcs
    }

    with open(out_path, "w") as derived_specs_file:
        json.dump(derived_specs, derived_specs_file, indent=4)
        derived_specs_file.write("\n")

    return derived_specs


if __name__ == "__main__":
    derived_specs = calculate_derived_specs()
    print(json.dumps(derived_specs, indent=4))
    print(f"Wrote {DERIVED_SPECS_PATH}")
