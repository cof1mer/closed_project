#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import numpy as np

def load_npz(path):
    data = np.load(path, allow_pickle=False)
    # Clés attendues
    for k in ("lat", "lon", "points_suivis"):
        if k not in data.files:
            raise KeyError(f"Clé manquante '{k}' dans {path}. Clés dispo: {data.files}")
    lat = np.asarray(data["lat"], dtype=float)
    lon = np.asarray(data["lon"], dtype=float)
    pts = np.asarray(data["points_suivis"], dtype=float)  # (N,2) en mètres UTM (x, y)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError("points_suivis doit être de forme (N,2) [x,y] en mètres.")
    return lat, lon, pts

def sanitize_latlon(lat, lon):
    lat = np.asarray(lat, dtype=float)
    lon = np.asarray(lon, dtype=float)
    m = np.isfinite(lat) & np.isfinite(lon)
    lat = lat[m]
    lon = lon[m]
    m2 = (lat >= -90) & (lat <= 90) & (lon >= -180) & (lon <= 180)
    lat = lat[m2]
    lon = lon[m2]
    if lat.size < 1:
        raise ValueError("Pas de points GPS valides (lat/lon).")
    return lat, lon

def utm30_to_wgs84(xs, ys):
    """Convertit UTM zone 30N (EPSG:32630) -> WGS84 (lat, lon)."""
    try:
        from pyproj import Transformer
    except ImportError:
        print("pyproj n'est pas installé. Fais: pip install pyproj")
        sys.exit(1)
    transformer = Transformer.from_crs("epsg:32630", "epsg:4326", always_xy=True)
    lon, lat = transformer.transform(xs, ys)  # always_xy=True => (x,y)->(lon,lat)
    return np.asarray(lat), np.asarray(lon)

def write_folium_map(boat_lat, boat_lon, target_lat, target_lon, out_html):
    try:
        import folium
    except ImportError:
        print("folium n'est pas installé. Fais: pip install folium")
        sys.exit(1)

    center = [float(np.mean(boat_lat)), float(np.mean(boat_lon))]
    m = folium.Map(location=center, zoom_start=16, control_scale=True)

    # Trajectoire bateau (bleu)
    boat_path = [[float(a), float(b)] for a, b in zip(boat_lat, boat_lon)]
    folium.PolyLine(boat_path, weight=4, opacity=0.9, color="blue", tooltip="Bateau").add_to(m)
    folium.Marker([float(boat_lat[0]), float(boat_lon[0])], popup="Départ bateau", tooltip="Départ bateau").add_to(m)
    folium.Marker([float(boat_lat[-1]), float(boat_lon[-1])], popup="Arrivée bateau", tooltip="Arrivée bateau").add_to(m)

    # Trajectoire point suivi (rouge)
    tgt_path = [[float(a), float(b)] for a, b in zip(target_lat, target_lon)]
    folium.PolyLine(tgt_path, weight=3, opacity=0.9, color="red", tooltip="Point suivi").add_to(m)
    folium.Marker([float(target_lat[0]), float(target_lon[0])], popup="Départ point suivi", tooltip="Départ point suivi", icon=folium.Icon(color="red")).add_to(m)
    folium.Marker([float(target_lat[-1]), float(target_lon[-1])], popup="Arrivée point suivi", tooltip="Arrivée point suivi", icon=folium.Icon(color="red")).add_to(m)

    # Caler la vue sur l'ensemble des points
    all_pts = boat_path + tgt_path
    m.fit_bounds(all_pts)

    # Couches + légende simple
    folium.LayerControl().add_to(m)

    m.save(out_html)
    print(f"Carte Folium écrite: {out_html}")

from xml.sax.saxutils import escape

def write_dual_kml(boat_lat, boat_lon, target_lat, target_lon, out_kml):
    def coords_str(lats, lons):
        return "\n          " + "\n          ".join(f"{lo:.8f},{la:.8f},0" for la, lo in zip(lats, lons))

    # TOUT texte visible doit être échappé
    name_doc = escape("Trajets bateau & point suivi")
    name_traj_boat = escape("Trajet bateau")
    name_start_boat = escape("Départ bateau")
    name_end_boat = escape("Arrivée bateau")
    name_folder_boat = escape("Bateau")
    name_traj_target = escape("Trajet point suivi")
    name_start_target = escape("Départ point suivi")
    name_end_target = escape("Arrivée point suivi")
    name_folder_target = escape("Point suivi")

    kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>{name_doc}</name>

    <Style id="boat">
      <LineStyle><color>ff0000ff</color><width>4</width></LineStyle>
    </Style>
    <Style id="target">
      <LineStyle><color>ff000000</color><width>3</width></LineStyle>
    </Style>

    <Folder>
      <name>{name_folder_boat}</name>
      <Placemark>
        <name>{name_traj_boat}</name>
        <styleUrl>#boat</styleUrl>
        <LineString><tessellate>1</tessellate><coordinates>{coords_str(boat_lat, boat_lon)}</coordinates></LineString>
      </Placemark>
      <Placemark><name>{name_start_boat}</name><Point><coordinates>{boat_lon[0]:.8f},{boat_lat[0]:.8f},0</coordinates></Point></Placemark>
      <Placemark><name>{name_end_boat}</name><Point><coordinates>{boat_lon[-1]:.8f},{boat_lat[-1]:.8f},0</coordinates></Point></Placemark>
    </Folder>

    <Folder>
      <name>{name_folder_target}</name>
      <Placemark>
        <name>{name_traj_target}</name>
        <styleUrl>#target</styleUrl>
        <LineString><tessellate>1</tessellate><coordinates>{coords_str(target_lat, target_lon)}</coordinates></LineString>
      </Placemark>
      <Placemark><name>{name_start_target}</name><Point><coordinates>{target_lon[0]:.8f},{target_lat[0]:.8f},0</coordinates></Point></Placemark>
      <Placemark><name>{name_end_target}</name><Point><coordinates>{target_lon[-1]:.8f},{target_lat[-1]:.8f},0</coordinates></Point></Placemark>
    </Folder>

  </Document>
</kml>
"""
    with open(out_kml, "w", encoding="utf-8") as f:
        f.write(kml)
    print(f"KML écrit: {out_kml}")

def main():
    parser = argparse.ArgumentParser(description="Affiche sur carte la trajectoire du bateau ET du point suivi depuis log_data.npz.")
    parser.add_argument("--source", "-s", default="log/log_data.npz", help="Fichier .npz (par défaut: log/log_data.npz)")
    parser.add_argument("--html", default="carte.html", help="Fichier HTML Folium de sortie (par défaut: carte.html)")
    parser.add_argument("--kml", help="(Optionnel) Fichier KML de sortie avec les deux tracés")
    parser.add_argument("--downsample", type=int, default=0, help="Garder 1 point sur N (0 ou 1 = aucun)")
    args = parser.parse_args()

    if not os.path.exists(args.source):
        print(f"Source '{args.source}' introuvable.")
        sys.exit(1)

    # Charge données
    lat, lon, pts = load_npz(args.source)
    lat, lon = sanitize_latlon(lat, lon)

    if args.downsample and args.downsample > 1:
        step = args.downsample
        lat, lon = lat[::step], lon[::step]
        pts = pts[::step, :]

    # Convertit points_suivis (x,y UTM 30N) -> (lat, lon)
    xs, ys = pts[:, 0], pts[:, 1]
    tgt_lat, tgt_lon = utm30_to_wgs84(xs, ys)

    # Carte Folium
    write_folium_map(lat, lon, tgt_lat, tgt_lon, args.html)

    # KML optionnel
    if args.kml:
        write_dual_kml(lat, lon, tgt_lat, tgt_lon, args.kml)

if __name__ == "__main__":
    main()
