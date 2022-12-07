import numpy as np


def ease_out_exp(x: float) -> float:
    x = min(max(x, 0), 1)
    return 1 - np.power(2.0, -10 * x)


def ease_out_quad(x: float) -> float:
    x = min(max(x, 0), 1)
    return 1 - (1 - x) * (1 - x)
