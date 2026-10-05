import json
import argparse
from pathlib import Path

from data.constants import *
from tools.weight_calc import (
    get_weight_calcs,
    get_w_ij_warmup,
    get_w_ij_taxi,
    get_w_ij_takeoff,
    get_w_ij_climb,
    get_w_ij_cruise,
    get_w_ij_loiter,
    get_w_ij_descent,
    get_w_ij_landing_shutdown,
    write_payload_range_diagram,
)
from tools.lift_calc import get_lift_calcs

ROOT = Path(__file__).resolve().parent
SPECS_PATH = ROOT / "data" / "specs.json"
DERIVED_SPECS_PATH = ROOT / "data" / "specs_derived.json"
PAYLOAD_RANGE_PATH = ROOT / "data" / "payload_range.png"


def calculate_derived_specs(
    specs_path=SPECS_PATH,
    out_path=DERIVED_SPECS_PATH,
    payload_range_path=PAYLOAD_RANGE_PATH,
    use_raymer=False,
):
    with open(specs_path) as specs_file:
        specs = json.load(specs_file)

    # Initial guesses
    weight_calcs = {"mtow": 250e3}
    lift_calcs = {
        "(c_L/c_D)_star": 20,
        "cruise_alt (ft)": 25e3,
    }

    # Iterate onto solution (calculations are coupled)
    for _ in range(MAX_ITERS):
        flight_profile = [
            ("warmup", get_w_ij_warmup()),
            ("taxi", get_w_ij_taxi()),
            ("takeoff", get_w_ij_takeoff()),
            ("climb", get_w_ij_climb()),
            ("cruise", get_w_ij_cruise(R, C_T, MAX_CRUISE_MACH, lift_calcs["(c_L/c_D)_star"], lift_calcs["cruise_alt (ft)"] * FT2M)),
            ("loiter", get_w_ij_loiter(E_LOITER, C_T, lift_calcs["(c_L/c_D)_star"])),
            ("descent", get_w_ij_descent()),
            ("landing and shutdown", get_w_ij_landing_shutdown()),
        ]

        weight_calcs = get_weight_calcs(
            specs,
            specs_path,
            flight_profile,
            W_PAYLOAD,
            SAFETY_FACTOR_FUEL,
            use_raymer,
            MAX_ITERS,
        )

        lift_calcs = get_lift_calcs(
            specs,
            e=OSTWALD_E,
            c_D0=C_D0,
            M_max_cruise=MAX_CRUISE_MACH,
            mtow=weight_calcs["mtow"],
        )

    w_fuel_max = specs["wings"]["ftank"]["mass"]["weight_max"]
    payload_range = write_payload_range_diagram(
        weight_calcs,
        lift_calcs,
        w_fuel_max=w_fuel_max,
        out_path=payload_range_path,
    )

    # Write derived specs to file
    derived_specs = {
        "weight_calcs": weight_calcs,
        "lift_calcs": lift_calcs,
        "payload_range": payload_range,
    }

    with open(out_path, "w") as derived_specs_file:
        json.dump(derived_specs, derived_specs_file, indent=4)
        derived_specs_file.write("\n")

    return derived_specs


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--use-raymer", action="store_true", default=False)
    args = parser.parse_args()

    derived_specs = calculate_derived_specs(
        specs_path=SPECS_PATH,
        out_path=DERIVED_SPECS_PATH,
        use_raymer=args.use_raymer,
    )

    print(json.dumps(derived_specs, indent=4))
    print(f"Wrote {DERIVED_SPECS_PATH}")
    print(f"Wrote {derived_specs['payload_range']['path']}")
