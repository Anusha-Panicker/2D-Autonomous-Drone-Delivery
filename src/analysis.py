import numpy as np


def hessian_analysis(derivatives, z, rho=100.0):
    hessian = derivatives.numerical_hessian(z, rho=rho)
    eigenvalues = np.linalg.eigvalsh(hessian)
    tolerance = 1e-6
    if np.all(eigenvalues > tolerance):
        classification = "positive definite"
    elif np.all(eigenvalues >= -tolerance):
        classification = "positive semidefinite"
    elif np.any(eigenvalues > tolerance) and np.any(eigenvalues < -tolerance):
        classification = "indefinite"
    else:
        classification = "negative semidefinite"
    condition_number = derivatives.condition_number(z, rho=rho)
    return {
        "eigenvalues": eigenvalues.tolist(),
        "minimum_eigenvalue": float(np.min(eigenvalues)),
        "maximum_eigenvalue": float(np.max(eigenvalues)),
        "classification": classification,
        "condition_number": float(condition_number) if np.isfinite(condition_number) else None,
        "symmetry_error": float(np.max(np.abs(hessian - hessian.T))),
    }
