from tools.weight_calc import get_mtow

# Placeholder inputs — replace with mission/aircraft values when available
W_PAYLOAD = 5000.0  # payload + crew weight
W_FUEL_0 = 0.25  # fuel weight fraction W_fuel / W_0
W_EMPTY_0_INIT = 0.55  # initial empty-weight fraction guess
MAX_ITERS = 10


def _print_mtow_chain(mtow_chain):
    headers = ("i", "mtow", "w_empty_0", "eps")
    rows = list(
        zip(
            mtow_chain["i"],
            mtow_chain["mtow"],
            mtow_chain["w_empty_0"],
            mtow_chain["eps"],
        )
    )

    col_widths = [len(h) for h in headers]
    formatted = []
    for row in rows:
        cells = (
            f"{row[0]:d}",
            f"{row[1]:.6g}",
            f"{row[2]:.6g}",
            f"{row[3]:.6g}",
        )
        formatted.append(cells)
        col_widths = [max(w, len(c)) for w, c in zip(col_widths, cells)]

    header_line = "  ".join(h.ljust(w) for h, w in zip(headers, col_widths))
    sep_line = "  ".join("-" * w for w in col_widths)
    print(header_line)
    print(sep_line)
    for cells in formatted:
        print("  ".join(c.ljust(w) for c, w in zip(cells, col_widths)))


if __name__ == "__main__":
    mtow_chain = get_mtow(
        W_PAYLOAD,
        W_FUEL_0,
        W_EMPTY_0_INIT,
        max_iters=MAX_ITERS,
    )
    _print_mtow_chain(mtow_chain)
