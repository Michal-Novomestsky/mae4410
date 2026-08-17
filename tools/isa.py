from pathlib import Path

import numpy as np

_ISA_CSV = Path(__file__).resolve().parent.parent / "data" / "isa.csv"

# CSV header -> short name used by the getters
_COLUMNS = {
    "h (m)": "h",
    "theta": "theta",
    "delta": "delta",
    "sigma": "sigma",
    "T (K)": "T",
    "p (Pa)": "p",
    "rho (kg/m^3)": "rho",
    "a (m/s)": "a",
}


def _load_table():
    data = np.genfromtxt(_ISA_CSV, delimiter=",", names=True, deletechars="", replace_space=" ")
    table = {_COLUMNS[header]: np.asarray(data[header], dtype=float) for header in _COLUMNS}

    order = np.argsort(table["h"])
    return {name: column[order] for name, column in table.items()}


_TABLE = _load_table()


def _interp(name, z):
    h = _TABLE["h"]
    if not h[0] <= z <= h[-1]:
        raise ValueError(f"Altitude {z} m is outside the ISA table ({h[0]} to {h[-1]} m); refusing to extrapolate")

    return float(np.interp(z, h, _TABLE[name]))

def get_theta(z):
    return _interp("theta", z)

def get_delta(z):
    return _interp("delta", z)

def get_sigma(z):
    return _interp("sigma", z)

def get_T(z):
    return _interp("T", z)

def get_p(z):
    return _interp("p", z)

def get_rho(z):
    return _interp("rho", z)

def get_a(z):
    return _interp("a", z)
