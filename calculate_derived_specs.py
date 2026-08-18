import json
from pathlib import Path

from data.constants import *
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

def calculate_derived_specs(
    specs_path=SPECS_PATH,
    out_path=DERIVED_SPECS_PATH,
):
    with open(specs_path) as specs_file:
        specs = json.load(specs_file)

    # Run calculations
    flight_profile = [
        get_w_ij_warmup_takeoff(),
        get_w_ij_climb(),
        get_w_ij_cruise(R, C_T, M, L_D, CRUISE_ALT),
        get_w_ij_loiter(E_LOITER, C_T, L_D),
        get_w_ij_descent(),
        get_w_ij_landing(),
    ]
    weight_calcs = get_weight_calcs(specs, specs_path, flight_profile)
    
    # Write derived specs to file
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
