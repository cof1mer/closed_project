#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import numpy as np
from datetime import datetime

# Coordonnées de la bouée utilisées dans control_bouee.py
BOUEE_LAT = 48.199245
BOUEE_LON = -3.0163166666

def load_gps_from_npz(path):
    data = np.load(path)
    # On accepte plusieurs clés possibles par tolérance
    lat_keys = [k for k in data.files if k.lower() in ("lat", "lats", "latitude")]
    lon_keys = [k for k in data.files if k.lower() in ("lon", "lons", "longitude")]
    if not lat_keys or not lon_keys:
        raise KeyError(f"Fichier NPZ '{path}' sans clés lat/lon attendues. Clés trouvées: {data.files}")
    lat = np.asarray(data[lat_keys[0]]).astype(float)
    lon = np.asarray(data[lon_keys[0]]).astype(float)
    return lat, lon

def load_gps_from_delim(path, delimiter=";"):
    """
    Charge un CSV/TXT délimité (par défaut ;) avec un header contenant 'lat' et 'lon'.
    Retombe en mode 'meilleur effort' si pas de header.
    """
    # Essai avec header
    try:
        arr = np.genfromtxt(path, delimiter=delimiter, names=True, dtype=None, encoding="utf-8", invalid_raise=False)
        cols = [c.lower() for c in arr.dtype.names] if arr.dtype.names else []
        if arr.dtype.names and ("lat" in cols and "lon" in cols):
            lat = np.asarray(arr["lat"], dtype=float)
            lon = np.asarray(arr["lon"], dtype=float)
            return lat, lon
    except Exception:
        pass

    # Essai sans header: on tente colonnes fixes -> lat, lon
    arr = np.genfromtxt(path, delimiter=delimiter, dtype=float, invalid_raise=False)
    if arr.ndim == 1 and arr.size >= 2:
        # Une seule ligne
        lat, lon = np.array([arr[0]]), np.array([arr[1]])
        return lat, lon
    elif arr.ndim == 2 and arr.shape[1] >= 2:
        lat = arr[:, 0]
        lon = arr[:, 1]
        return lat, lon

    raise ValueError(f"Impossible d'interpréter '{path}'. Attendu colonnes lat;lon ou header lat/lon.")

def sanitize_series(lat, lon):
    lat = np.asarray(lat, dtype=float)
    lon = np.asarray(lon, dtype=float)
    m = np.isfinite(lat) & np.isfinite(lon)
    lat = lat[m]
    lon = lon[m]
    # Filtre simple: lat dans [-90,90], lon dans [-180,180]
    m2 = (lat >= -90) & (lat <= 90) & (lon >= -180) & (lon <= 180)
    lat = lat[m2]
    lon = lon[m2]
    if lat.size < 1:
        raise ValueError("Pas de points GPS valides après nettoyage.")
    return lat, lon

def write_kml(lats, lons, out_kml, name=None):
    name = name or os.path.basename(out_kml)
    coords_str = " ".join([f"{lon:.8f},{lat:.8f},0" for lat, lon in zip(lats, lons)])
    start_lon, start_lat = float(lons[0]), float(lats[0])
    end_lon, end_lat = float(lons[-1]), float(lats[-1])

    kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>{name}</name>

    <Style id="pathStyle">
      <LineStyle>
        <color>7f00ffff</color> <!-- aabbggrr (ici semi-transp cyan) -->
        <width>4</width>
      </LineStyle>
    </Style>

    <Placemark>
      <name>Trajet</name>
      <styleUrl>#pathStyle</styleUrl>
      <LineString>
        <tessellate>1</tessellate>
        <coordinates>
          {coords_str}
        </coordinates>
      </LineString>
    </Placemark>

    <Placemark>
      <name>Départ</name>
      <Point><coordinates>{start_lon:.8f},{start_lat:.8f},0</coordinates></Point>
    </Placemark>

    <Placemark>
      <name>Arrivée</name>
      <Point><coordinates>{end_lon:.8f},{end_lat:.8f},0</coordinates></Point>
    </Placemark>

    <Placemark>
      <name>Bouée</name>
      <Point><coordinates>{BOUEE_LON:.8f},{BOUEE_LAT:.8f},0</coordinates></Point>
    </Placemark>

  </Document>
</kml>
"""
    with open(out_kml, "w", encoding="utf-8") as f:
        f.write(kml)

def write_folium_map(lats, lons, out_html):
    try:
        import folium
    except ImportError:
        print("Folium n'est pas installé. Installe-le avec: pip install folium")
        return False

    center = [float(np.mean(lats)), float(np.mean(lons))]
    m = folium.Map(location=center, zoom_start=16, control_scale=True)

    # Trace
    path = [[float(lat), float(lon)] for lat, lon in zip(lats, lons)]
    folium.PolyLine(path, weight=4, opacity=0.8).add_to(m)

    # Marqueurs départ / arrivée
    folium.Marker([float(lats[0]), float(lons[0])], popup="Départ", tooltip="Départ").add_to(m)
    folium.Marker([float(lats[-1]), float(lons[-1])], popup="Arrivée", tooltip="Arrivée").add_to(m)

    # Bouée (coordonnées venant de control_bouee.py)
    folium.Marker(
        [BOUEE_LAT, BOUEE_LON],
        popup="Bouée",
        tooltip="Bouée",
        icon=folium.Icon(color="orange", icon="flag")
    ).add_to(m)

    # Fit bounds
    m.fit_bounds(path + [[BOUEE_LAT, BOUEE_LON]])

    m.save(out_html)
    return True

def main():
    parser = argparse.ArgumentParser(
        description="Convertit un log GPS (NPZ/CSV) en KML et génère une carte Folium."
    )
    parser.add_argument("kml_out", help="Chemin de sortie KML (ex: trajet.kml)")
    parser.add_argument("--source", "-s", default="log/log_data.npz",
                        help="Fichier source GPS (par défaut: log_data.npz). Accepte .npz ou .csv/.txt (;).")
    parser.add_argument("--html", help="Chemin de sortie Folium HTML (par défaut: même base que le KML).")
    parser.add_argument("--no-folium", action="store_true", help="Ne pas générer la carte Folium.")
    parser.add_argument("--downsample", type=int, default=0,
                        help="Garde 1 point sur N (0 ou 1 = aucun sous-échantillonnage).")
    args = parser.parse_args()

    src = args.source
    if not os.path.exists(src):
        print(f"Source '{src}' introuvable. Spécifie un autre fichier avec --source.")
        sys.exit(1)

    # Chargement
    try:
        if src.lower().endswith(".npz"):
            lat, lon = load_gps_from_npz(src)
        else:
            # On suppose CSV/TXT délimité par ';'
            lat, lon = load_gps_from_delim(src, delimiter=";")
    except Exception as e:
        print(f"Erreur de lecture de '{src}': {e}")
        sys.exit(1)

    # Nettoyage + éventuel downsample
    try:
        lat, lon = sanitize_series(lat, lon)
    except Exception as e:
        print(f"Données invalides: {e}")
        sys.exit(1)

    if args.downsample and args.downsample > 1:
        lat = lat[::args.downsample]
        lon = lon[::args.downsample]

    # Écriture KML
    try:
        kml_name = f"Trajet {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
        write_kml(lat, lon, args.kml_out, name=kml_name)
        print(f"KML écrit: {args.kml_out}")
    except Exception as e:
        print(f"Échec écriture KML: {e}")
        sys.exit(1)

    # Folium (sauf désactivé)
    if not args.no_folium:
        html_out = args.html
        if not html_out:
            base, _ = os.path.splitext(args.kml_out)
            html_out = base + ".html"
        try:
            ok = write_folium_map(lat, lon, html_out)
            if ok:
                print(f"Carte Folium écrite: {html_out}")
        except Exception as e:
            print(f"Folium: échec de génération: {e}")

if __name__ == "__main__":
    main()
