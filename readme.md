# Guide DDBoat

## 1. Calibrer entièrement l’IMU

### 1.1 Acquisition sur le DDBoat
```bash
python3 imu.py
```
Enregistre les mesures brutes de magnétomètre dans `log/data_imu_mag.npy`.

### 1.2 Traitement initial sur la machine perso
```bash
scp ue32@172.20.25.213:Guerlebang/py/log/data_imu_mag.npy log/<nom1>.npy
```
Adapter ensuite `calibration.py` (ligne 59) :
```python
path = "log/<nom1>.npy"
```
Puis calculer l’ellipse de calibration et rapatrier le résultat sur le DDBoat :
```bash
python3 calibration.py
scp log/mag_calib.npz ue32@172.20.25.213:Guerlebang/py/log
```

### 1.3 Alignement IMU à bord
1. Vérifier que la ligne de correction IMU (ligne 53) est **commentée** dans `euler_angle.py`.
2. Réaliser les poses Nord, Ouest et Haut :
   ```bash
   python3 euler_angle.py
   ```
   Ce script produit `log/ycorr.npy`.

### 1.4 Post-traitement final
```bash
scp ue32@172.20.25.213:Guerlebang/py/log/ycorr.npy log/<nom2>.npy
```
Mettre `R_imu.py` à jour (ligne 16) :
```python
Y = np.load("log/<nom2>.npy").T
```
Générer ensuite la matrice d’alignement puis la renvoyer sur le DDBoat :
```bash
python3 R_imu.py
scp log/R_imu.npy ue32@172.20.25.213:Guerlebang/py/log
```

### 1.5 Réactiver la correction IMU
Décommenter la ligne 53 de `euler_angle.py` puis renvoyer le fichier :
```bash
scp euler_angle.py ue32@172.20.25.213:Guerlebang/py
```
Les angles retournés par `euler_angle.py` sont désormais fiables.

---

## 2. Lancer les missions

### 2.1 Bouée aller-retour
```bash
python3 control_bouee.py
```
Produit `log/log_data_bouee.npz`.

### 2.2 Mission de suivi d’une trajectoire
```bash
python3 control_suivi.py
```
Produit `log/log_data_suivi.npz`.

---

## 3. Consulter et visualiser les logs

### 3.1 Trajets simples (lat/lon)
```bash
python3 read_log.py trajet_bouee.kml --source log/log_data_bouee.npz
```
Génère un fichier KML (`trajet_bouee.kml`) et, par défaut, une carte Folium `trajet_bouee.html`, incluant la position fixe de la bouée issue de `control_bouee.py`.

Options utiles :
- `--html sortie.html` : nom personnalisé pour la carte.
- `--no-folium` : saute la carte HTML.
- `--downsample N` : conserve un point GPS sur `N`.

### 3.2 Trajet + point suivi
```bash
python3 read_log2.py --source log/log_data_suivi.npz --html suivi.html --kml suivi.kml
```
Affiche la trajectoire du bateau (bleu) et celle du point virtuel suivi (rouge), convertit automatiquement `points_suivis` (coordonnées UTM) en lat/lon et peut générer un KML combiné. L’option `--downsample N` reste disponible.

### 3.3 Dépendances
- `read_log.py` nécessite `folium` (HTML).
- `read_log2.py` nécessite `folium` **et** `pyproj`.
Installer si besoin :
```bash
pip install folium pyproj
```
Ouvrir les `.html` dans un navigateur ou les `.kml` dans Google Earth pour visualiser les trajets.

## 4. Scripts complémentaires

### 4.1 Points de suivi
`calcule_points_suivi.py` calcule la liste de points de consigne et génère une carte Folium illustrant la trajectoire cible. On peut y tester les paramètres de l’équation de trajectoire afin de valider leur applicabilité avant mission.

### 4.2 GPS
`gps.py` permet seulement de vérifier que le gps fonctionne bien

### 4.3 Propulseur
`propel.py` permet de vérifier le bon fonctionnement des propulseurs

### 4.4 Les drivers
Les fichiers du dossier `drivers-ddboat-v2` sert à piloter les différents capteurs et actionneurs du ddboat. Ces derniers nous étaient fournis en amont.