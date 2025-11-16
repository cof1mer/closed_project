# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import os

# =======================
#  Ellipsoid LSQ fitting
# =======================
def fit_ellipsoid_lsq(X):
    X = np.asarray(X)
    assert X.ndim == 2 and X.shape[1] == 3
    x, y, z = X[:,0], X[:,1], X[:,2]
    D = np.column_stack([
        x*x, y*y, z*z, x*y, x*z, y*z, x, y, z, np.ones_like(x)
    ])

    U, S, VT = np.linalg.svd(D, full_matrices=False)
    beta = VT[-1, :]

    a,b,c,d,e,f,g,h,i,j = beta
    M = np.array([[a, d/2, e/2],
                  [d/2, b, f/2],
                  [e/2, f/2, c]], dtype=float)
    n = np.array([g, h, i], dtype=float)
    d0 = j

    if np.linalg.det(M) == 0:
        raise ValueError("M singulière : jeu de données trop dégénéré pour un fit d'ellipsoïde.")

    c_center = -0.5 * np.linalg.solve(M, n)
    k = c_center.T @ M @ c_center - d0
    if k <= 0:
        M *= -1.0; n *= -1.0; d0 *= -1.0
        c_center = -0.5 * np.linalg.solve(M, n)
        k = c_center.T @ M @ c_center - d0
        if k <= 0:
            raise ValueError("Ajustement non valide (k <= 0).")

    A = 0.5 * (M / k + (M / k).T)
    return M, n, d0, c_center, A

def map_to_unit_sphere_via_fit(X):
    M, n, d0, c, A = fit_ellipsoid_lsq(X)
    vals, vecs = np.linalg.eigh(A)
    if np.any(vals <= 0):
        raise ValueError("A non-SPD après fit — échantillonnage insuffisant.")
    A_sqrt = vecs @ np.diag(np.sqrt(vals)) @ vecs.T
    print(np.shape(X-c))
    Y = (X - c) @ A_sqrt.T
    np.savez("log/mag_calib.npz",
         bias=c.reshape(3, 1),          # shape (3,1)
         C=A_sqrt.T,                # shape (3,3)
         )
    return Y, {"center": c, "A": A}

# =======================
#  Load user's data
# =======================
path = "log/mag_mercredi.npy"
if not os.path.exists(path):
    raise FileNotFoundError("Je n'ai pas trouvé log/data_imu_mag.npy. Merci de l'uploader puis relance.")

X = np.load(path)
if X.ndim != 2 or X.shape[1] != 3:
    raise ValueError(f"On attend un array (n,3), reçu {X.shape}")

# =======================
#  Mapping by fit
# =======================
Y, info = map_to_unit_sphere_via_fit(X)
norms = np.linalg.norm(Y, axis=1)
print("Centre estimé :", info["center"])
print("Normes après mapping: min=%.4f mean=%.4f max=%.4f" % (norms.min(), norms.mean(), norms.max()))

# =======================
#  Plots
# =======================
def set_axes_equal(ax, P):
    x, y, z = P[:,0], P[:,1], P[:,2]
    max_range = max(x.max()-x.min(), y.max()-y.min(), z.max()-z.min())
    cx, cy, cz = x.mean(), y.mean(), z.mean()
    ax.set_xlim(cx - max_range/2, cx + max_range/2)
    ax.set_ylim(cy - max_range/2, cy + max_range/2)
    ax.set_zlim(cz - max_range/2, cz + max_range/2)

if __name__ == "__main__":
    fig = plt.figure(figsize=(14,6))
    ax1 = fig.add_subplot(1,2,1, projection="3d")
    ax2 = fig.add_subplot(1,2,2, projection="3d")

    ax1.scatter(X[:,0], X[:,1], X[:,2], s=15, alpha=0.8)
    ax1.set_title("Avant")
    ax1.set_xlabel("x"); ax1.set_ylabel("y"); ax1.set_zlabel("z")
    set_axes_equal(ax1, X)

    ax2.scatter(Y[:,0], Y[:,1], Y[:,2], s=15, alpha=0.8)
    u = np.linspace(0, 2*np.pi, 40); v = np.linspace(0, np.pi, 20)
    xs = np.outer(np.cos(u), np.sin(v))
    ys = np.outer(np.sin(u), np.sin(v))
    zs = np.outer(np.ones_like(u), np.cos(v))
    ax2.plot_wireframe(xs, ys, zs, linewidth=0.6, alpha=0.35)
    ax2.set_title("Après (fit ellipsoïde exact)")
    ax2.set_xlabel("x"); ax2.set_ylabel("y"); ax2.set_zlabel("z")
    set_axes_equal(ax2, Y)

    fig.tight_layout()
    plt.show()

