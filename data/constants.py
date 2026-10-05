# Fixed constants
G_GRAV = 9.81
FT2M = 0.3048

# RFP requirements
R = 8000e3  # 8000 km
E_LOITER = 30 * 60  # 30 min
LOITER_ALT = 1500 * FT2M
SAFETY_FACTOR_FUEL = 0.05  # 5% trip fuel contingency
MAX_CRUISE_MACH = 0.85

# Aircraft specs
C_T = 14e-6  # kg/Ns Estimate for high-bypass turbofan (Raymer)
OSTWALD_E = 0.82 # Taken from PDR (Most likely from OVSP?)
C_D0 = 0.0189 # https://discordapp.com/channels/1534147646871048272/1534148203379687434/1543505525084528640

# Static margin limits as fractions of MAC (forward = SM_MAX, aft = SM_MIN)
SM_MIN = 0.05
SM_MAX = 0.3

# Calculation hyperparams
MAX_ITERS = 10
