# -*- coding: utf-8 -*-
import time
import math
import numpy as np
import sys, os
from pyproj import Proj



projDegree2Meter = Proj("+proj=utm +zone=30 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs +type=crs")


# --- Déclaration des points de références ---

reference_lat_ponton = 48.199020000000004
reference_lon_ponton = -3.0146599999999997

reference_lat_bouee = 48.199245
reference_lon_bouee = -3.0163166666

center_latlon = [48.199707, -3.018784]

reference_x_ponton,reference_y_ponton = projDegree2Meter(reference_lon_ponton,reference_lat_ponton)
reference_x_bouee, reference_y_bouee = projDegree2Meter(reference_lon_bouee,reference_lat_bouee)

bouee = np.array([reference_x_bouee, reference_y_bouee])
center = projDegree2Meter(center_latlon[1], center_latlon[0])
ponton = np.array([reference_x_ponton, reference_y_ponton])

R1 = 240
T_c = 3000
w1 = -1/(3*R1)
t_off = 100

N = 4
moii = 3

def R2(t):
    t = np.asarray(t)
    return 10*np.exp(-t/200) + 5            # shape: (), (n,)

def w2(t):
    t = np.asarray(t)
    return 1/(2*R2(t))                    # shape: (), (n,)

def p(t):
    t = np.asarray(t)
    xy = R1 * np.stack([np.cos(w1*(t + t_off)), np.sin(w1*(t + t_off))], axis=-1)   # (n,2) ou (2,)
    return center + xy

def v(t, i=3):
    t = np.asarray(t)
    phase = w2(t)*t + 2*np.pi*i/N            # shape: (), (n,)
    unit_xy = np.stack([np.cos(phase), np.sin(phase)], axis=-1) # (n,2) ou (2,)
    r = np.asarray(R2(t))[..., np.newaxis]   # -> (), (n,1)  => broadcast OK
    return r * unit_xy

def a(t):
    t = np.asarray(t)
    out = p(t) + v(t, moii)
    return out if t.ndim > 0 else out.squeeze()

# (1000, 2)
points_suivis = a(np.linspace(0, 750, 1000))




# --- Export KML + carte Folium pour les points_suivis (à coller juste après la ligne qui crée points_suivis) ---
from xml.sax.saxutils import escape

# 1) Conversion UTM Zone 30N (EPSG:32630) -> WGS84 (lat, lon)
try:
    from pyproj import Transformer
except ImportError as e:
    raise RuntimeError("pyproj est requis: pip install pyproj") from e

pts = np.asarray(points_suivis, dtype=float)
if pts.ndim != 2 or pts.shape[1] != 2:
    raise ValueError("points_suivis doit être de forme (N,2) [x,y] en mètres UTM.")

xs, ys = pts[:, 0], pts[:, 1]
transformer = Transformer.from_crs("epsg:32630", "epsg:4326", always_xy=True)
tgt_lon, tgt_lat = transformer.transform(xs, ys)  # (x,y)->(lon,lat)

# Filtre de sécurité (NaN/inf, bornes WGS84)
m = (np.isfinite(tgt_lat) & np.isfinite(tgt_lon) &
     (tgt_lat >= -90) & (tgt_lat <= 90) &
     (tgt_lon >= -180) & (tgt_lon <= 180))
tgt_lat = np.asarray(tgt_lat)[m]
tgt_lon = np.asarray(tgt_lon)[m]
if tgt_lat.size < 2:
    raise ValueError("Trop peu de points valides pour tracer/exporter.")

# 2) Écriture KML (uniquement les points à suivre)
def write_kml_targets(target_lat, target_lon, out_kml="points_suivis.kml"):
    def coords_str(lats, lons):
        return "\n          " + "\n          ".join(f"{lo:.8f},{la:.8f},0" for la, lo in zip(lats, lons))

    name_doc = escape("Trajet du point à suivre")
    name_traj = escape("Point suivi")
    name_start = escape("Départ point suivi")
    name_end = escape("Arrivée point suivi")

    kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>{name_doc}</name>

    <Style id="target">
      <LineStyle><color>ff0000ff</color><width>4</width></LineStyle> <!-- aabbggrr -->
    </Style>

    <Placemark>
      <name>{name_traj}</name>
      <styleUrl>#target</styleUrl>
      <LineString>
        <tessellate>1</tessellate>
        <coordinates>{coords_str(target_lat, target_lon)}</coordinates>
      </LineString>
    </Placemark>

    <Placemark>
      <name>{name_start}</name>
      <Point><coordinates>{target_lon[0]:.8f},{target_lat[0]:.8f},0</coordinates></Point>
    </Placemark>

    <Placemark>
      <name>{name_end}</name>
      <Point><coordinates>{target_lon[-1]:.8f},{target_lat[-1]:.8f},0</coordinates></Point>
    </Placemark>

  </Document>
</kml>
"""
    with open(out_kml, "w", encoding="utf-8") as f:
        f.write(kml)
    print(f"KML (points suivis) écrit: {out_kml}")

write_kml_targets(tgt_lat, tgt_lon, out_kml="log/points_suivis.kml")

# 3) Carte Folium (HTML) avec la trajectoire du point suivi
try:
    import folium
except ImportError as e:
    raise RuntimeError("folium est requis: pip install folium") from e

center = [float(np.mean(tgt_lat)), float(np.mean(tgt_lon))]
m = folium.Map(location=center, zoom_start=16, control_scale=True)

tgt_path = [[float(a), float(b)] for a, b in zip(tgt_lat, tgt_lon)]
folium.PolyLine(tgt_path, weight=4, opacity=0.9, color="red", tooltip="Point à suivre").add_to(m)
folium.Marker([float(tgt_lat[0]), float(tgt_lon[0])],
              popup="Départ point suivi", tooltip="Départ point suivi",
              icon=folium.Icon(color="red")).add_to(m)
folium.Marker([float(tgt_lat[-1]), float(tgt_lon[-1])],
              popup="Arrivée point suivi", tooltip="Arrivée point suivi",
              icon=folium.Icon(color="red")).add_to(m)

m.fit_bounds(tgt_path)
m.save("log/points_suivis.html")
print("Carte Folium écrite: points_suivis.html")
# Option pratique :
# import webbrowser, os; webbrowser.open(f"file://{os.path.abspath('points_suivis.html')}")
