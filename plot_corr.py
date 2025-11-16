import numpy as np
import matplotlib.pyplot as plt


fig = plt.figure(figsize=(14,6))


Ycorr = np.load("log/nord_ouest_haut_corr2.npy")
Y = np.load("log/nord_ouest_haut2.npy")
# Y = np.load("log/ycorr_perpendiculaire_haut64.npy")
# Z = np.load("log/ycorr_perpendiculaire_ouest.npy")
# ax1 = fig.add_subplot(1,1,1, projection="3d")


# u = np.linspace(0, 2*np.pi, 40); v = np.linspace(0, np.pi, 20)
# xs = np.outer(np.cos(u), np.sin(v))
# ys = np.outer(np.sin(u), np.sin(v))
# zs = np.outer(np.ones_like(u), np.cos(v))
# ax1.plot_wireframe(xs, ys, zs, linewidth=0.6, alpha=0.35)
# ax1.scatter(X[:,0], X[:,1], X[:,2], s=15, alpha=0.8, color = "blue")
# ax1.scatter(Y[:,0], Y[:,1], Y[:,2], s=15, alpha=0.8, color = "red")
# ax1.scatter(Z[:,0], Z[:,1], Z[:,2], s=15, alpha=0.8, color = "yellow")
# ax1.set_title("Fit")
# ax1.set_xlabel("x"); ax1.set_ylabel("y"); ax1.set_zlabel("z")

# plt.show()


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


    ax1.scatter(Y[:,0], Y[:,1], Y[:,2], s=15, alpha=0.8)
    ax1.set_title("Avant")
    ax1.set_xlabel("x"); ax1.set_ylabel("y"); ax1.set_zlabel("z")
    set_axes_equal(ax1, Y)

    #ax2.scatter(Ycorr[:,0], Ycorr[:,1], Ycorr[:,2], s=15, alpha=0.8)
    for c,i in zip(["blue", "red", "yellow"], range(3)):
        ax2.scatter(Ycorr[i, 0], Ycorr[i, 1], Ycorr[i, 2], s = 15, alpha = 0.8, color = c)
        print(Ycorr[i, :])

    u = np.linspace(0, 2*np.pi, 40); v = np.linspace(0, np.pi, 20)
    xs = np.outer(np.cos(u), np.sin(v))
    ys = np.outer(np.sin(u), np.sin(v))
    zs = np.outer(np.ones_like(u), np.cos(v))
    ax2.plot_wireframe(xs, ys, zs, linewidth=0.6, alpha=0.35)
    ax2.set_title("Après (fit ellipsoïde exact)")
    ax2.set_xlabel("x"); ax2.set_ylabel("y"); ax2.set_zlabel("z")

    fig.tight_layout()
    plt.show()