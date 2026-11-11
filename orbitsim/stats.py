import numpy as np


def wilson(k, n, z=1.96):
    """95 % Wilson score interval for a proportion."""
    if n == 0:
        return 0.0, 0.0, 0.0
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, max(0.0, c - h), min(1.0, c + h)


def describe(x):
    x = np.asarray(x, float)
    return dict(n=int(x.size), mean=float(x.mean()), median=float(np.median(x)),
                p95=float(np.percentile(x, 95)), max=float(x.max()))
