# Fixed constants
G_GRAV = 9.81
FT2M = 0.3048

# RFP requirements
R = 8000e3  # 8000 km
E_LOITER = 30 * 60  # 30 min
LOITER_ALT = 1500 * FT2M
CRUISE_ALT = 35000 * FT2M # Technically this is the service ceiling TODO rename for clarity
SAFETY_FACTOR_FUEL = 0.05  # 5% trip fuel contingency
M = 0.85 # Cruise mach

# Aircraft specs
C_T = 14e-6  # kg/Ns Estimate for high-bypass turbofan (Raymer)
L_D = 20  # Ballpark estimate from slides (Raymer)
W_PAYLOAD = 57493  # Estimate from Michal's W1 work

# Calculation hyperparams
MAX_ITERS = 10