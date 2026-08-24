# Fixed constants
G_GRAV = 9.81
FT2M = 0.3048

# RFP requirements
R = 8000e3  # 8000 km
E_LOITER = 30 * 60  # 30 min
LOITER_ALT = 1500 * FT2M
CRUISE_ALT = 35000 * FT2M # Technically this is the service ceiling TODO rename for clarity
SAFETY_FACTOR_FUEL = 0.05  # 5% trip fuel contingency
MAX_CRUISE_MACH = 0.85

# Aircraft specs
C_T = 14e-6  # kg/Ns Estimate for high-bypass turbofan (Raymer)
W_PAYLOAD = 79396 # https://discordapp.com/channels/1534147646871048272/1534148227849388042/1541292944319840276
OSTWALD_E = 0.85 # Rough estimate for standard e (~0.8-0.9)
C_D0 = 0.02 # Rough estimate (taken from PDR 2025)

# Calculation hyperparams
MAX_ITERS = 10