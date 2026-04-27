"""
v7.map_widget — Embedded Leaflet.js map widget for the PLFM Radar GUI V7.

Classes:
  - MapBridge        — QObject exposed to JavaScript via QWebChannel
  - RadarMapWidget   — QWidget wrapping QWebEngineView with Leaflet map

The full HTML/CSS/JS for Leaflet is generated inline (no external files).
Supports: OSM, Google, Google Sat, Google Hybrid, ESRI Sat tile servers;
coverage circle, target trails, velocity-based color coding, popups, legend.
"""

import json
import logging

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFrame,
    QComboBox, QCheckBox, QPushButton, QLabel,
)
from PyQt6.QtCore import Qt, QUrl, pyqtSignal, pyqtSlot, QObject
from PyQt6.QtWebEngineCore import QWebEngineSettings
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebChannel import QWebChannel

from .models import (
    GPSData, RadarTarget, TileServer,
    DARK_BG, DARK_FG, DARK_ACCENT, DARK_BORDER,
    DARK_TEXT, DARK_BUTTON, DARK_BUTTON_HOVER,
    DARK_SUCCESS, DARK_INFO,
)

logger = logging.getLogger(__name__)


# =============================================================================
# MapBridge — Python <-> JavaScript
# =============================================================================

class MapBridge(QObject):
    """Bridge object registered with QWebChannel for JS ↔ Python calls."""

    mapClicked = pyqtSignal(float, float)   # lat, lon
    markerClicked = pyqtSignal(int)          # target_id
    mapReady = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._map_ready = False

    @pyqtSlot(float, float)
    def onMapClick(self, lat: float, lon: float):
        logger.debug(f"Map clicked: {lat}, {lon}")
        self.mapClicked.emit(lat, lon)

    @pyqtSlot(int)
    def onMarkerClick(self, target_id: int):
        logger.debug(f"Marker clicked: #{target_id}")
        self.markerClicked.emit(target_id)

    @pyqtSlot()
    def onMapReady(self):
        logger.info("Leaflet map ready")
        self._map_ready = True
        self.mapReady.emit()

    @pyqtSlot(str)
    def logFromJS(self, message: str):
        logger.info(f"[JS] {message}")

    @property
    def is_ready(self) -> bool:
        return self._map_ready


# =============================================================================
# RadarMapWidget
# =============================================================================

class RadarMapWidget(QWidget):
    """
    Embeds a Leaflet.js interactive map inside a QWebEngineView.

    Public methods mirror the V6 map API:
      set_radar_position(gps), set_targets(list), set_coverage_radius(r),
      set_zoom(level)
    """

    targetSelected = pyqtSignal(int)

    def __init__(self, radar_lat: float = 41.9028, radar_lon: float = 12.4964,
                 parent=None):
        super().__init__(parent)

        # State
        self._radar_position = GPSData(
            latitude=radar_lat, longitude=radar_lon,
            altitude=0.0, pitch=0.0, heading=0.0,
        )
        self._targets: list[RadarTarget] = []
        self._pending_targets: list[RadarTarget] | None = None
        self._coverage_radius = 1_536   # metres (64 bins x ~24 m/bin)
        self._tile_server = TileServer.OPENSTREETMAP
        self._show_coverage = True
        self._show_trails = False
        self._show_3d = False

        # Build UI
        self._setup_ui()

        # Bridge + channel
        self._bridge = MapBridge(self)
        self._bridge.mapReady.connect(self._on_map_ready)
        self._bridge.markerClicked.connect(self._on_marker_clicked)

        self._channel = QWebChannel()
        self._channel.registerObject("bridge", self._bridge)
        self._web_view.page().setWebChannel(self._channel)

        # Load the Leaflet map
        self._load_map()

    # ---- UI setup ----------------------------------------------------------

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Control bar
        bar = QFrame()
        bar.setStyleSheet(f"background-color: {DARK_ACCENT}; border-radius: 4px;")
        bar_layout = QHBoxLayout(bar)
        bar_layout.setContentsMargins(8, 4, 8, 4)

        # Tile selector
        self._tile_combo = QComboBox()
        self._tile_combo.addItem("OpenStreetMap", TileServer.OPENSTREETMAP)
        self._tile_combo.addItem("Google Maps", TileServer.GOOGLE_MAPS)
        self._tile_combo.addItem("Google Satellite", TileServer.GOOGLE_SATELLITE)
        self._tile_combo.addItem("Google Hybrid", TileServer.GOOGLE_HYBRID)
        self._tile_combo.addItem("ESRI Satellite", TileServer.ESRI_SATELLITE)
        self._tile_combo.currentIndexChanged.connect(self._on_tile_changed)
        self._tile_combo.setStyleSheet(f"""
            QComboBox {{
                background-color: {DARK_BUTTON}; color: {DARK_FG};
                border: 1px solid {DARK_BORDER}; padding: 4px 8px; border-radius: 4px;
            }}
        """)
        bar_layout.addWidget(QLabel("Tiles:"))
        bar_layout.addWidget(self._tile_combo)

        # Toggles
        self._coverage_check = QCheckBox("Coverage")
        self._coverage_check.setChecked(True)
        self._coverage_check.stateChanged.connect(self._on_coverage_toggled)
        bar_layout.addWidget(self._coverage_check)

        self._trails_check = QCheckBox("Trails")
        self._trails_check.setChecked(False)
        self._trails_check.stateChanged.connect(self._on_trails_toggled)
        bar_layout.addWidget(self._trails_check)

        #added 3d mode creates a checkbox next to the trails box
        self._3d_check = QCheckBox("3D")
        self._3d_check.setChecked(False)
        self._3d_check.stateChanged.connect(self._on_3d_toggled)
        bar_layout.addWidget(self._3d_check)

        btn_style = f"""
            QPushButton {{
                background-color: {DARK_BUTTON}; color: {DARK_FG};
                border: 1px solid {DARK_BORDER}; padding: 4px 12px; border-radius: 4px;
            }}
            QPushButton:hover {{ background-color: {DARK_BUTTON_HOVER}; }}
        """

        center_btn = QPushButton("Center")
        center_btn.clicked.connect(self._center_on_radar)
        center_btn.setStyleSheet(btn_style)
        bar_layout.addWidget(center_btn)

        fit_btn = QPushButton("Fit All")
        fit_btn.clicked.connect(self._fit_all)
        fit_btn.setStyleSheet(btn_style)
        bar_layout.addWidget(fit_btn)

        bar_layout.addStretch()

        self._status_label = QLabel("Loading map...")
        self._status_label.setStyleSheet(f"color: {DARK_INFO};")
        bar_layout.addWidget(self._status_label)

        layout.addWidget(bar)

        # Web view
        self._web_view = QWebEngineView()
        self._web_view.setMinimumSize(400, 300)
        layout.addWidget(self._web_view, stretch=1)

    # ---- HTML generation ---------------------------------------------------

    def _get_map_html(self) -> str:
        lat = self._radar_position.latitude
        lon = self._radar_position.longitude
        cov = self._coverage_radius

        # Using {{ / }} for literal braces inside the f-string
        return f'''<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Radar Map</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY=" crossorigin=""/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
    integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo=" crossorigin=""></script>
<script src="qrc:///qtwebchannel/qwebchannel.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<style>
* {{ margin:0; padding:0; box-sizing:border-box; }}
html, body {{ height:100%; width:100%; font-family:'Segoe UI',Arial,sans-serif; }}
#map {{ height:100%; width:100%; background-color:{DARK_BG}; }}
.leaflet-container {{ background-color:{DARK_BG} !important; }}
.leaflet-popup-content-wrapper {{
    background-color:{DARK_ACCENT}; color:{DARK_FG};
    border-radius:8px; box-shadow:0 4px 12px rgba(0,0,0,0.4);
}}
.leaflet-popup-tip {{ background-color:{DARK_ACCENT}; }}
.leaflet-popup-content {{ margin:12px; }}
.popup-title {{
    font-size:14px; font-weight:bold; color:#4e9eff;
    margin-bottom:8px; border-bottom:1px solid {DARK_BORDER}; padding-bottom:6px;
}}
.popup-row {{ display:flex; justify-content:space-between; margin:4px 0; font-size:12px; }}
.popup-label {{ color:{DARK_TEXT}; }}
.popup-value {{ color:{DARK_FG}; font-weight:500; }}
.status-approaching {{ color:#F44336; }}
.status-receding {{ color:#2196F3; }}
.status-stationary {{ color:#9E9E9E; }}
.legend {{
    background-color:{DARK_ACCENT}; color:{DARK_FG};
    padding:10px 14px; border-radius:6px; font-size:12px;
    box-shadow:0 2px 8px rgba(0,0,0,0.3);
}}
.legend-title {{ font-weight:bold; margin-bottom:8px; color:#4e9eff; }}
.legend-item {{ display:flex; align-items:center; margin:4px 0; }}
.legend-color {{
    width:14px; height:14px; border-radius:50%;
    margin-right:8px; border:1px solid white;
}}
</style>
</head>
<body>

<! This is the block that it generates  >

<div id="map"></div>
<div id="view3d" style="display:none; width:100%; height:100%;
    background:#1a1a2e; position:relative;">
    <div id="popup3d" style="
        display:none; position:absolute; background:#3c3f41;
        color:#e0e0e0; border:1px solid #555; border-radius:8px;
        padding:12px; font-size:12px; font-family:'Segoe UI',Arial,sans-serif;
        pointer-events:none; z-index:100; min-width:160px;
        box-shadow:0 4px 12px rgba(0,0,0,0.5);
    "></div>
</div>

<script>
var map, radarMarker, coverageCircle;
var targetMarkers = {{}};
var targetTrails = {{}};
var targetTrailHistory = {{}};
var bridge = null;
var currentTileLayer = null;
var showCoverage = true;
var showTrails = false;
var maxTrailLength = 30;

var tileServers = {{
    'osm': {{
        url:'https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',
        attribution:'&copy; OpenStreetMap', maxZoom:19
    }},
    'google': {{
        url:'https://mt0.google.com/vt/lyrs=m&hl=en&x={{x}}&y={{y}}&z={{z}}&s=Ga',
        attribution:'&copy; Google Maps', maxZoom:22
    }},
    'google_sat': {{
        url:'https://mt0.google.com/vt/lyrs=s&hl=en&x={{x}}&y={{y}}&z={{z}}&s=Ga',
        attribution:'&copy; Google Maps', maxZoom:22
    }},
    'google_hybrid': {{
        url:'https://mt0.google.com/vt/lyrs=y&hl=en&x={{x}}&y={{y}}&z={{z}}&s=Ga',
        attribution:'&copy; Google Maps', maxZoom:22
    }},
    'esri_sat': {{
        url:'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}',
        attribution:'&copy; Esri', maxZoom:19
    }}
}};

function initMap() {{
    map = L.map('map', {{ preferCanvas:true, zoomControl:true }})
        .setView([{lat}, {lon}], 10);
    setTileServer('osm');

    radarMarker = L.circleMarker([{lat},{lon}], {{
        radius:12, fillColor:'#FF5252', color:'white',
        weight:3, opacity:1, fillOpacity:1
    }}).addTo(map);
    updateRadarPopup();

    coverageCircle = L.circle([{lat},{lon}], {{
        radius:{cov}, color:'#FF5252', fillColor:'#FF5252',
        fillOpacity:0.08, weight:2, dashArray:'8, 8'
    }}).addTo(map);

    addLegend();
    map.on('click', function(e){{ if(bridge) bridge.onMapClick(e.latlng.lat,e.latlng.lng); }});
}}

function setTileServer(id) {{
    var cfg = tileServers[id]; if(!cfg) return;
    if(currentTileLayer) map.removeLayer(currentTileLayer);
    currentTileLayer = L.tileLayer(
        cfg.url,
        {{ attribution:cfg.attribution, maxZoom:cfg.maxZoom }}
    ).addTo(map);
}}

function updateRadarPopup() {{
    if(!radarMarker) return;
    var ll = radarMarker.getLatLng();
    radarMarker.bindPopup(
        '<div class="popup-title">Radar System</div>'+
        (
            '<div class="popup-row"><span class="popup-label">Lat:</span>'+
            '<span class="popup-value">'+ll.lat.toFixed(6)+'</span></div>'
        )+
        (
            '<div class="popup-row"><span class="popup-label">Lon:</span>'+
            '<span class="popup-value">'+ll.lng.toFixed(6)+'</span></div>'
        )+
        (
            '<div class="popup-row"><span class="popup-label">Status:</span>'+
            '<span class="popup-value status-approaching">Active</span></div>'
        )
    );
}}

function addLegend() {{
    var legend = L.control({{ position:'bottomright' }});
    legend.onAdd = function() {{
        var d = L.DomUtil.create('div','legend');
        d.innerHTML =
            '<div class="legend-title">Target Legend</div>'+
            (
                '<div class="legend-item"><div class="legend-color" '+
                'style="background:#F44336"></div>Approaching</div>'
            )+
            (
                '<div class="legend-item"><div class="legend-color" '+
                'style="background:#2196F3"></div>Receding</div>'
            )+
            (
                '<div class="legend-item"><div class="legend-color" '+
                'style="background:#9E9E9E"></div>Stationary</div>'
            )+
            (
                '<div class="legend-item"><div class="legend-color" '+
                'style="background:#FF5252"></div>Radar</div>'
            );
        return d;
    }};
    legend.addTo(map);
}}

function updateRadarPosition(lat,lon,alt,pitch,heading) {{
    if(!radarMarker||!coverageCircle) return;
    radarMarker.setLatLng([lat,lon]);
    coverageCircle.setLatLng([lat,lon]);
    updateRadarPopup();
}}

function updateTargets(targetsJson) {{
    try {{
        if(!map) {{
            if(bridge) bridge.logFromJS('updateTargets: map not ready yet');
            return;
        }}
        var targets = JSON.parse(targetsJson);
        if(bridge) bridge.logFromJS('updateTargets: parsed '+targets.length+' targets');
        var currentIds = {{}};

        targets.forEach(function(t) {{
            currentIds[t.id] = true;
            var lat=t.latitude, lon=t.longitude;
            var color = getTargetColor(t.velocity);
            var radius = Math.max(5, Math.min(12, 5+(t.snr||0)/5));

            if(!targetTrailHistory[t.id]) targetTrailHistory[t.id] = [];
            targetTrailHistory[t.id].push([lat,lon]);
            if(targetTrailHistory[t.id].length > maxTrailLength)
                targetTrailHistory[t.id].shift();

            if(targetMarkers[t.id]) {{
                targetMarkers[t.id].setLatLng([lat,lon]);
                targetMarkers[t.id].setStyle({{
                    fillColor:color, color:'white', radius:radius
                }});
                if(targetTrails[t.id]) {{
                    targetTrails[t.id].setLatLngs(targetTrailHistory[t.id]);
                    targetTrails[t.id].setStyle({{ color:color }});
                }}
            }} else {{
                var marker = L.circleMarker([lat,lon], {{
                    radius:radius, fillColor:color, color:'white',
                    weight:2, opacity:1, fillOpacity:0.9
                }}).addTo(map);
                marker.on(
                    'click',
                    (function(id){{
                        return function(){{ if(bridge) bridge.onMarkerClick(id); }};
                    }})(t.id)
                );
                targetMarkers[t.id] = marker;
                if(showTrails) {{
                    targetTrails[t.id] = L.polyline(targetTrailHistory[t.id], {{
                        color:color, weight:3, opacity:0.7,
                        lineCap:'round', lineJoin:'round'
                    }}).addTo(map);
                }}
            }}
            updateTargetPopup(t);
        }});

        for(var id in targetMarkers) {{
            if(!currentIds[id]) {{
                map.removeLayer(targetMarkers[id]); delete targetMarkers[id];
                if(targetTrails[id]) {{
                    map.removeLayer(targetTrails[id]);
                    delete targetTrails[id];
                }}
                delete targetTrailHistory[id];
            }}
        }}
        if(show3D) render3D(targets); 
    }} catch(e) {{
        if(bridge) bridge.logFromJS('updateTargets ERROR: '+e.message);
    }}
}}

function updateTargetPopup(t) {{
    if(!targetMarkers[t.id]) return;
    var sc = t.velocity>1
        ? 'status-approaching'
        : (t.velocity<-1 ? 'status-receding' : 'status-stationary');
    var st = t.velocity>1?'Approaching':(t.velocity<-1?'Receding':'Stationary');
    var rng = (typeof t.range === 'number') ? t.range.toFixed(1) : '?';
    var vel = (typeof t.velocity === 'number') ? t.velocity.toFixed(1) : '?';
    var az  = (typeof t.azimuth === 'number') ? t.azimuth.toFixed(1) : '?';
    var el  = (typeof t.elevation === 'number') ? t.elevation.toFixed(1) : '?';
    var snr = (typeof t.snr === 'number') ? t.snr.toFixed(1) : '?';
    targetMarkers[t.id].bindPopup(
        '<div class="popup-title">Target #'+t.id+'</div>'+
        '<div class="popup-row"><span class="popup-label">Range:</span>'+
        '<span class="popup-value">'+rng+' m</span></div>'+
        '<div class="popup-row"><span class="popup-label">Velocity:</span>'+
        '<span class="popup-value">'+vel+' m/s</span></div>'+
        '<div class="popup-row"><span class="popup-label">Azimuth:</span>'+
        '<span class="popup-value">'+az+'&deg;</span></div>'+
        '<div class="popup-row"><span class="popup-label">Elevation:</span>'+
        '<span class="popup-value">'+el+'&deg;</span></div>'+
        '<div class="popup-row"><span class="popup-label">SNR:</span>'+
        '<span class="popup-value">'+snr+' dB</span></div>'+
        '<div class="popup-row"><span class="popup-label">Track:</span>'+
        '<span class="popup-value">'+t.track_id+'</span></div>'+
        '<div class="popup-row"><span class="popup-label">Status:</span>'+
        '<span class="popup-value '+sc+'">'+st+'</span></div>'
    );
}}

function getTargetColor(v) {{
    if(v>50)  return '#FF1744';
    if(v>10)  return '#FF5252';
    if(v>1)   return '#FF8A65';
    if(v<-50) return '#1565C0';
    if(v<-10) return '#2196F3';
    if(v<-1)  return '#64B5F6';
    return '#9E9E9E';
}}

function setCoverageVisible(vis) {{
    showCoverage = vis;
    if(coverageCircle) {{
        if(vis) coverageCircle.addTo(map); else map.removeLayer(coverageCircle);
    }}
}}
function setCoverageRadius(r) {{ if(coverageCircle) coverageCircle.setRadius(r); }}

function setTrailsVisible(vis) {{
    showTrails = vis;
    if(vis) {{
        for(var id in targetMarkers) {{
            if(!targetTrails[id] && targetTrailHistory[id] && targetTrailHistory[id].length>1) {{
                targetTrails[id] = L.polyline(targetTrailHistory[id], {{
                    color:'#4CAF50', weight:3, opacity:0.7, lineCap:'round', lineJoin:'round'
                }}).addTo(map);
            }} else if(targetTrails[id]) {{
                targetTrails[id].addTo(map);
            }}
        }}
    }} else {{
        for(var id in targetTrails) {{ map.removeLayer(targetTrails[id]); }}
    }}
}}

function centerOnRadar() {{ if(radarMarker) map.setView(radarMarker.getLatLng(), map.getZoom()); }}

function fitAllTargets() {{
    var b = L.latLngBounds([]);
    if(radarMarker) b.extend(radarMarker.getLatLng());
    for(var id in targetMarkers) b.extend(targetMarkers[id].getLatLng());
    if(b.isValid()) map.fitBounds(b, {{ padding:[50,50] }});
}}

function setZoom(lvl) {{ map.setZoom(lvl); }}

var show3D = false;
var scene, camera, renderer;
var targetMeshes = {{}}, dropLines = {{}};
var trailLines3D = {{}}, trailHistory3D = {{}}, targetData = {{}};

/* new lines for the rings */
var groundRingsGroup = null;
var coverageRingRadius = {cov};

function init3D() {{
    var container = document.getElementById('view3d');
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a2e);

    camera = new THREE.PerspectiveCamera(
        60, container.clientWidth / container.clientHeight, 1, 200000
    );
    camera.position.set(0, 12000, 20000);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({{ antialias: true }});
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    //var grid = new THREE.GridHelper(100000, 20, 0x444466, 0x333355);
    //scene.add(grid);

    drawGroundRings(coverageRingRadius);



    var radarGeo = new THREE.SphereGeometry(100, 16, 16);
    var radarMat = new THREE.MeshBasicMaterial({{ color: 0xFF5252 }});
    scene.add(new THREE.Mesh(radarGeo, radarMat));

    scene.add(new THREE.AmbientLight(0xffffff, 0.8));

    var isDragging = false, prevX = 0, prevY = 0;
    var theta = 0, phi = Math.PI / 4, radius = 10000;

    renderer.domElement.addEventListener('mousedown', function(e) {{
        isDragging = true; prevX = e.clientX; prevY = e.clientY;
    }});
    renderer.domElement.addEventListener('mouseup', function() {{ isDragging = false; }});
    renderer.domElement.addEventListener('mousemove', function(e) {{
        if(!isDragging) return;
        theta -= (e.clientX - prevX) * 0.005;
        phi   -= (e.clientY - prevY) * 0.005;
        phi = Math.max(0.1, Math.min(Math.PI/2, phi));
        prevX = e.clientX; prevY = e.clientY;
        camera.position.set(
            radius * Math.sin(phi) * Math.sin(theta),
            radius * Math.cos(phi),
            radius * Math.sin(phi) * Math.cos(theta)
        );
        camera.lookAt(0, 0, 0);
    }});
    renderer.domElement.addEventListener('wheel', function(e) {{
        radius = Math.max(5000, Math.min(150000, radius + e.deltaY * 10));
        camera.position.set(
            radius * Math.sin(phi) * Math.sin(theta),
            radius * Math.cos(phi),
            radius * Math.sin(phi) * Math.cos(theta)
        );
        camera.lookAt(0, 0, 0);
    }});

    var raycaster = new THREE.Raycaster();
    var mouse = new THREE.Vector2();

    renderer.domElement.addEventListener('click', function(e) {{
        var rect = renderer.domElement.getBoundingClientRect();
        mouse.x =  ((e.clientX - rect.left) / rect.width)  * 2 - 1;
        mouse.y = -((e.clientY - rect.top)  / rect.height) * 2 + 1;

        raycaster.setFromCamera(mouse, camera);
        var meshes = Object.values(targetMeshes);
        var hits = raycaster.intersectObjects(meshes);

        var popup = document.getElementById('popup3d');
        if(hits.length > 0) {{
            var hit = hits[0].object;
            var tid = Object.keys(targetMeshes).find(function(k) {{
                return targetMeshes[k] === hit;
            }});
            var t = targetData[tid];
            if(t) {{
                var status = t.velocity > 1 ? 'Approaching' :
                    (t.velocity < -1 ? 'Receding' : 'Stationary');
                var statusColor = t.velocity > 1 ? '#F44336' :
                    (t.velocity < -1 ? '#2196F3' : '#9E9E9E');
                popup.innerHTML =
                    '<div style="font-weight:bold;color:#4e9eff;'+
                    'margin-bottom:8px;border-bottom:1px solid #555;'+
                    'padding-bottom:6px;">Target #'+t.id+'</div>'+
                    '<div>Range: '+t.range.toFixed(1)+' m</div>'+
                    '<div>Velocity: '+t.velocity.toFixed(1)+' m/s</div>'+
                    '<div>Azimuth: '+t.azimuth.toFixed(1)+'°</div>'+
                    '<div>Elevation: '+t.elevation.toFixed(1)+'°</div>'+
                    '<div>SNR: '+t.snr.toFixed(1)+' dB</div>'+
                    '<div>Track: '+t.track_id+'</div>'+
                    '<div>Status: <span style="color:'+statusColor+'">'+status+'</span></div>';
                popup.style.left = (e.clientX - rect.left + 10) + 'px';
                popup.style.top  = (e.clientY - rect.top  + 10) + 'px';
                popup.style.display = 'block';
            }}
        }} else {{
            popup.style.display = 'none';
        }}
    }});

    animate3D();
}}

function animate3D() {{
    requestAnimationFrame(animate3D);
    if(show3D) renderer.render(scene, camera);
}}

function render3D(targets) {{
    if(!scene) return;
    var seen = {{}};
    targets.forEach(function(t) {{
        seen[t.id] = true;
        var az = t.azimuth  * Math.PI / 180;
        var el = t.elevation * Math.PI / 180;
        var r  = t.range;
        var x  =  r * Math.cos(el) * Math.sin(az);
        var y  =  r * Math.sin(el);
        var z  =  r * Math.cos(el) * Math.cos(az);
        if(!trailHistory3D[t.id]) trailHistory3D[t.id] = [];
        trailHistory3D[t.id].push(new THREE.Vector3(x, y, z));
        if(trailHistory3D[t.id].length > 30) trailHistory3D[t.id].shift();
        var color = getTargetColor3D(t.velocity);
        targetData[t.id] = t;

        if(targetMeshes[t.id]) {{
            targetMeshes[t.id].position.set(x, y, z);
            targetMeshes[t.id].material.color.set(color);
            if(dropLines[t.id]) {{
                dropLines[t.id].geometry.setFromPoints([
                    new THREE.Vector3(x, y, z),
                    new THREE.Vector3(x, 0, z)
                ]);
            }}
            if(trailLines3D[t.id] && trailHistory3D[t.id].length > 1) {{
                trailLines3D[t.id].geometry.setFromPoints(trailHistory3D[t.id]);
            }}
        }} else {{
            var geo = new THREE.SphereGeometry(80, 12, 12);
            var mat = new THREE.MeshBasicMaterial({{ color: color }});
            var mesh = new THREE.Mesh(geo, mat);
            mesh.position.set(x, y, z);
            scene.add(mesh);
            targetMeshes[t.id] = mesh;


            // this block renders the droplines
            var dropGeo = new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(x, y, z),
                new THREE.Vector3(x, 0, z)
            ]);
            var dropMat = new THREE.LineBasicMaterial({{
                color: color, opacity: 0.4, transparent: true
            }});
            var dropLine = new THREE.Line(dropGeo, dropMat);
            scene.add(dropLine);
            dropLines[t.id] = dropLine;
            var trailGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(x,y,z)]);
            var trailMat = new THREE.LineBasicMaterial({{
                color: color, opacity: 0.6, transparent: true
            }});1
            var trailLine = new THREE.Line(trailGeo, trailMat);
            scene.add(trailLine);
            trailLines3D[t.id] = trailLine;
        }}
    }});

    for(var id in targetMeshes) {{
        if(!seen[id]) {{
            scene.remove(targetMeshes[id]);
            delete targetMeshes[id];
            if(dropLines[id]) {{
                scene.remove(dropLines[id]);
                delete dropLines[id];
            }}
            if(trailLines3D[id]) {{
                scene.remove(trailLines3D[id]);
                delete trailLines3D[id];
            }}
            if(trailHistory3D[id]) delete trailHistory3D[id];
        }}
    }}
}}

function getTargetColor3D(v) {{
    if(v > 1)  return 0xFF5252;
    if(v < -1) return 0x2196F3;
    return 0x9E9E9E;
}}

function toggle3D(enable) {{
    show3D = enable;
    document.getElementById('map').style.display    = enable ? 'none'  : 'block';
    document.getElementById('view3d').style.display = enable ? 'block' : 'none';
    if(enable && !renderer) init3D();
}}

// Fuunction to draw the ground rings //
function drawGroundRings(coverageRadius) {{
    if(groundRingsGroup) {{
        scene.remove(groundRingsGroup);
        groundRingsGroup = null;
    }}

    groundRingsGroup = new THREE.Group();

    var renderDistance = coverageRadius * 2.5;
    var ringCount = 8;
    var spacing = renderDistance / ringCount;

    for(var i = 1; i <= ringCount; i++) {{
        var r = spacing * i;
        var isCoverage = Math.abs(r - coverageRadius) < spacing * 0.15;

        var segments = 128;
        var points = [];
        for(var j = 0; j <= segments; j++) {{
            var angle = (j / segments) * Math.PI * 2;
            points.push(new THREE.Vector3(
                Math.sin(angle) * r, 0, Math.cos(angle) * r
            ));
        }}

        var geo = new THREE.BufferGeometry().setFromPoints(points);
        var mat = new THREE.LineBasicMaterial({{
            color: isCoverage ? 0x00ff88 : 0x334455,
            linewidth: isCoverage ? 3 : 1,
            opacity: isCoverage ? 1.0 : 0.5,
            transparent: !isCoverage
        }});
        var ring = new THREE.Line(geo, mat);
        groundRingsGroup.add(ring);

        // Distance label on east side
        var labelText = r >= 1000
            ? (r/1000).toFixed(1) + ' km'
            : r.toFixed(0) + ' m';
        if(isCoverage) labelText += ' ◀ coverage';
        var ringColor = isCoverage ? '#00ff88' : '#445566';
        addGroundLabel(labelText, r + 200, 0, 0, ringColor, groundRingsGroup);
    }}

    // NSEW labels at outermost ring
    var outer = spacing * ringCount;
    addGroundLabel('N', 0, 0, -(outer + 1500), '#aaaacc', groundRingsGroup);
    addGroundLabel('S', 0, 0,  (outer + 1500), '#aaaacc', groundRingsGroup);
    addGroundLabel('E', (outer + 1500), 0, 0,  '#aaaacc', groundRingsGroup);
    addGroundLabel('W', -(outer + 1500), 0, 0, '#aaaacc', groundRingsGroup);

    // Bearing lines (N-S and E-W cross)
    var lineColor = 0x334455;
    [[0,0,-(outer), 0,0,(outer)], [-(outer),0,0, (outer),0,0]].forEach(function(pts) {{
        var lpts = [
            new THREE.Vector3(pts[0],pts[1],pts[2]),
            new THREE.Vector3(pts[3],pts[4],pts[5])
        ];
        var lg = new THREE.BufferGeometry().setFromPoints(lpts);
        var lm = new THREE.LineBasicMaterial({{ color:lineColor, opacity:0.4, transparent:true }});
        groundRingsGroup.add(new THREE.Line(lg, lm));
    }});

    scene.add(groundRingsGroup);
}}

function addGroundLabel(text, x, y, z, color, group) {{
    var canvas = document.createElement('canvas');
    canvas.width = 230; canvas.height = 64;
    var ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, 230, 64);
    ctx.font = 'bold 17px Arial';
    ctx.fillStyle = color;
    ctx.textAlign = 'center';
    ctx.fillText(text, 128, 40);

    var tex = new THREE.CanvasTexture(canvas);
    var mat = new THREE.SpriteMaterial({{ map: tex, transparent: true }});
    var sprite = new THREE.Sprite(mat);
    sprite.position.set(x, y, z);
    sprite.scale.set(2000, 500, 1);
    group.add(sprite);
}}

function updateCoverageRing(radius) {{
    coverageRingRadius = radius;
    if(show3D) drawGroundRings(radius);
}}

document.addEventListener('DOMContentLoaded', function() {{
    new QWebChannel(qt.webChannelTransport, function(ch) {{
        bridge = ch.objects.bridge;
        initMap();
        if(bridge) bridge.onMapReady();
    }});
}});
</script>
</body>
</html>'''

    # ---- load / helpers ----------------------------------------------------

    def _load_map(self):
        # Enable remote resource access so Leaflet CDN scripts/tiles can load.
        settings = self._web_view.page().settings()
        settings.setAttribute(
            QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls,
            True,
        )
        # Provide an HTTP base URL so the page has a proper origin;
        # without this, setHtml() defaults to about:blank which blocks
        # external resource loading in modern Chromium.
        self._web_view.setHtml(
            self._get_map_html(),
            QUrl("http://localhost/radar_map"),
        )
        logger.info("Leaflet map HTML loaded (with HTTP base URL)")

    def _on_map_ready(self):
        self._status_label.setText(f"Map ready - {len(self._targets)} targets")
        self._status_label.setStyleSheet(f"color: {DARK_SUCCESS};")
        # Flush any targets that arrived before the map was ready
        if self._pending_targets is not None:
            self.set_targets(self._pending_targets)
            self._pending_targets = None

    def _on_marker_clicked(self, tid: int):
        self.targetSelected.emit(tid)

    def _run_js(self, script: str):
        def _js_callback(result):
            if result is not None:
                logger.info("JS result: %s", result)
        self._web_view.page().runJavaScript(script, 0, _js_callback)

    # ---- control bar callbacks ---------------------------------------------

    def _on_tile_changed(self, _index: int):
        server = self._tile_combo.currentData()
        if server:
            self._tile_server = server
            self._run_js(f"setTileServer('{server.value}')")

    def _on_coverage_toggled(self, state: int):
        vis = state == Qt.CheckState.Checked.value
        self._show_coverage = vis
        self._run_js(f"setCoverageVisible({str(vis).lower()})")

    def _on_trails_toggled(self, state: int):
        vis = state == Qt.CheckState.Checked.value
        self._show_trails = vis
        self._run_js(f"setTrailsVisible({str(vis).lower()})")


    # Added a new function to toggle the 3d state.
    def _on_3d_toggled(self, state: int):
        vis = state == Qt.CheckState.Checked.value
        self._show_3d = vis
        self._run_js(f"toggle3D({str(vis).lower()})")

    def _center_on_radar(self):
        self._run_js("centerOnRadar()")

    def _fit_all(self):
        self._run_js("fitAllTargets()")

    # ---- public API --------------------------------------------------------

    def set_radar_position(self, gps: GPSData):
        self._radar_position = gps
        self._run_js(
            f"updateRadarPosition({gps.latitude},{gps.longitude},"
            f"{gps.altitude},{gps.pitch},{gps.heading})"
        )

    def set_targets(self, targets: list[RadarTarget]):
        self._targets = targets
        if not self._bridge.is_ready:
            logger.info("Map not ready yet — queuing %d targets", len(targets))
            self._pending_targets = targets
            return
        data = [t.to_dict() for t in targets]
        js_payload = json.dumps(data).replace("\\", "\\\\").replace("'", "\\'")
        logger.info(
            "set_targets: %d targets, JSON len=%d, first 200 chars: %s",
            len(targets), len(js_payload), js_payload[:200],
        )
        self._status_label.setText(f"{len(targets)} targets tracked")
        self._run_js(f"updateTargets('{js_payload}')")



    def set_coverage_radius(self, radius_m: float):
       self._coverage_radius = radius_m
       self._run_js(f"setCoverageRadius({radius_m})")
       self._run_js(f"updateCoverageRing({radius_m})")

    def set_zoom(self, level: int):
        level = max(0, min(22, level))
        self._run_js(f"setZoom({level})")
