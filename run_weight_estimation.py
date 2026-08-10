from tools.constants import FT2M
from tools.weight_calc import get_mtow, get_w_payload, get_w_ij_climb, get_w_ij_cruise, get_w_ij_descent, get_w_ij_landing, get_w_ij_loiter, get_w_ij_warmup_takeoff
from tools.constants import *

# Placeholder inputs — replace with mission/aircraft values when available
W_PAYLOAD = 5000.0  # payload + crew weight
SAFETY_FACTOR_FUEL = 0.06 # SF on fuel weight fraction
W_EMPTY_0_INIT = 0.55  # initial empty-weight fraction guess
MAX_ITERS = 10

R = 8000e3 # 8000 km
E_LOITER = 30*60 # 30 min
c_t = 14e-6 # kg/Ns Estimate from slides
M = 0.85
L_D = 20 # Approximation from slides
CRUISE_ALT = 35000 * FT2M # TODO SERVICE CEILING NOT CRUISING ALT



def _print_mtow_chain(mtow_chain, decimals=3):
    headers = list(mtow_chain.keys())
    rows = zip(*(mtow_chain[h] for h in headers))
    fmt = f".{decimals}f"

    col_widths = [len(h) for h in headers]
    formatted = []
    for row in rows:
        cells = tuple(f"{float(v):{fmt}}" for v in row)
        formatted.append(cells)
        col_widths = [max(w, len(c)) for w, c in zip(col_widths, cells)]

    header_line = "  ".join(h.ljust(w) for h, w in zip(headers, col_widths))
    sep_line = "  ".join("-" * w for w in col_widths)
    print(header_line)
    print(sep_line)
    for cells in formatted:
        print("  ".join(c.ljust(w) for c, w in zip(cells, col_widths)))


if __name__ == "__main__":
    w_ij_chain = [
        get_w_ij_warmup_takeoff(),
        get_w_ij_climb(),
        get_w_ij_cruise(R, c_t, M, L_D, CRUISE_ALT),
        get_w_ij_loiter(E_LOITER, c_t, L_D),
        get_w_ij_descent(),
        get_w_ij_landing(),
    ]  

    mtow_chain = get_mtow(
        W_PAYLOAD,
        w_ij_chain,
        W_EMPTY_0_INIT,
        SAFETY_FACTOR_FUEL,
        max_iters=MAX_ITERS,
    )
    _print_mtow_chain(mtow_chain)
