import warnings


def get_a(z):
    warnings.warn("CURRENTLY ASSUMES FIXED ALTITUDE @ 11km FOR a", UserWarning)
    # Speed of sound in the ISA tropopause (≈11 km), m/s
    return 295.07
