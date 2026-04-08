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
from typing import List

from PyQt6.QtCore import QObject, Qt, pyqtSignal, pyqtSlot
from PyQt6.QtWebChannel import QWebChannel
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from .models import (
    DARK_ACCENT,
    DARK_BG,
    DARK_BORDER,
    DARK_BUTTON,
    DARK_BUTTON_HOVER,
    DARK_FG,
    DARK_INFO,
    DARK_SUCCESS,
    DARK_TEXT,
    GPSData,
    RadarTarget,
    TileServer,
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
        logger.debug(f"[JS] {message}")

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
        self._targets: List[RadarTarget] = []
        self._coverage_radius = 50_000   # metres
        self._tile_server = TileServer.OPENSTREETMAP
        self._show_coverage = True
        self._show_trails = False

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
<div id="map"></div>
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

    var radarIcon = L.divIcon({{
        className:'radar-icon',
        html:'<div style="background:radial-gradient(circle,#FF5252 0%,#D32F2F 100%);'+
             'width:24px;height:24px;border-radius:50%;border:3px solid white;'+
             'box-shadow:0 2px 8px rgba(0,0,0,0.5);"></div>',
        iconSize:[24,24], iconAnchor:[12,12]
    }});

    radarMarker = L.marker([{lat},{lon}], {{ icon:radarIcon, zIndexOffset:1000 }}).addTo(map);
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
    currentTileLayer = L.tileLayer(cfg.url, {{ attribution:cfg.attribution, maxZoom:cfg.maxZoom }}).addTo(map);
}}

function updateRadarPopup() {{
    if(!radarMarker) return;
    var ll = radarMarker.getLatLng();
    radarMarker.bindPopup(
        '<div class="popup-title">Radar System</div>'+
        '<div class="popup-row"><span class="popup-label">Lat:</span><span class="popup-value">'+ll.lat.toFixed(6)+'</span></div>'+
        '<div class="popup-row"><span class="popup-label">Lon:</span><span class="popup-value">'+ll.lng.toFixed(6)+'</span></div>'+
        '<div class="popup-row"><span class="popup-label">Status:</span><span class="popup-value status-approaching">Active</span></div>'
    );
}}

function addLegend() {{
    var legend = L.control({{ position:'bottomright' }});
    legend.onAdd = function() {{
        var d = L.DomUtil.create('div','legend');
        d.innerHTML =
            '<div class="legend-title">Target Legend</div>'+
            '<div class="legend-item"><div class="legend-color" style="background:#F44336"></div>Approaching</div>'+
            '<div class="legend-item"><div class="legend-color" style="background:#2196F3"></div>Receding</div>'+
            '<div class="legend-item"><div class="legend-color" style="background:#9E9E9E"></div>Stationary</div>'+
            '<div class="legend-item"><div class="legend-color" style="background:#FF5252"></div>Radar</div>';
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
    var targets = JSON.parse(targetsJson);
    var currentIds = {{}};

    targets.forEach(function(t) {{
        currentIds[t.id] = true;
        var lat=t.latitude, lon=t.longitude;
        var color = getTargetColor(t.velocity);
        var sz = Math.max(10, Math.min(20, 10+t.snr/3));

        if(!targetTrailHistory[t.id]) targetTrailHistory[t.id] = [];
        targetTrailHistory[t.id].push([lat,lon]);
        if(targetTrailHistory[t.id].length > maxTrailLength)
            targetTrailHistory[t.id].shift();

        if(targetMarkers[t.id]) {{
            targetMarkers[t.id].setLatLng([lat,lon]);
            targetMarkers[t.id].setIcon(makeIcon(color,sz));
            if(targetTrails[t.id]) {{
                targetTrails[t.id].setLatLngs(targetTrailHistory[t.id]);
                targetTrails[t.id].setStyle({{ color:color }});
            }}
        }} else {{
            var marker = L.marker([lat,lon], {{ icon:makeIcon(color,sz) }}).addTo(map);
            marker.on('click', (function(id){{ return function(){{ if(bridge) bridge.onMarkerClick(id); }}; }})(t.id));
            targetMarkers[t.id] = marker;
            if(showTrails) {{
                targetTrails[t.id] = L.polyline(targetTrailHistory[t.id], {{
                    color:color, weight:3, opacity:0.7, lineCap:'round', lineJoin:'round'
                }}).addTo(map);
            }}
        }}
        updateTargetPopup(t);
    }});

    for(var id in targetMarkers) {{
        if(!currentIds[id]) {{
            map.removeLayer(targetMarkers[id]); delete targetMarkers[id];
            if(targetTrails[id]) {{ map.removeLayer(targetTrails[id]); delete targetTrails[id]; }}
            delete targetTrailHistory[id];
        }}
    }}
}}

function makeIcon(color,sz) {{
    return L.divIcon({{
        className:'target-icon',
        html:'<div style="background-color:'+color+';width:'+sz+'px;height:'+sz+'px;'+
             'border-radius:50%;border:2px solid white;box-shadow:0 2px 6px rgba(0,0,0,0.4);"></div>',
        iconSize:[sz,sz], iconAnchor:[sz/2,sz/2]
    }});
}}

function updateTargetPopup(t) {{
    if(!targetMarkers[t.id]) return;
    var sc = t.velocity>1?'status-approaching':(t.velocity<-1?'status-receding':'status-stationary');
    var st = t.velocity>1?'Approaching':(t.velocity<-1?'Receding':'Stationary');
    targetMarkers[t.id].bindPopup(
        '<div class="popup-title">Target #'+t.id+'</div>'+
        '<div class="popup-row"><span class="popup-label">Range:</span><span class="popup-value">'+t.range.toFixed(1)+' m</span></div>'+
        '<div class="popup-row"><span class="popup-label">Velocity:</span><span class="popup-value">'+t.velocity.toFixed(1)+' m/s</span></div>'+
        '<div class="popup-row"><span class="popup-label">Azimuth:</span><span class="popup-value">'+t.azimuth.toFixed(1)+'&deg;</span></div>'+
        '<div class="popup-row"><span class="popup-label">Elevation:</span><span class="popup-value">'+t.elevation.toFixed(1)+'&deg;</span></div>'+
        '<div class="popup-row"><span class="popup-label">SNR:</span><span class="popup-value">'+t.snr.toFixed(1)+' dB</span></div>'+
        '<div class="popup-row"><span class="popup-label">Track:</span><span class="popup-value">'+t.track_id+'</span></div>'+
        '<div class="popup-row"><span class="popup-label">Status:</span><span class="popup-value '+sc+'">'+st+'</span></div>'
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
        self._web_view.setHtml(self._get_map_html())
        logger.info("Leaflet map HTML loaded")

    def _on_map_ready(self):
        self._status_label.setText(f"Map ready - {len(self._targets)} targets")
        self._status_label.setStyleSheet(f"color: {DARK_SUCCESS};")

    def _on_marker_clicked(self, tid: int):
        self.targetSelected.emit(tid)

    def _run_js(self, script: str):
        self._web_view.page().runJavaScript(script)

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

    def set_targets(self, targets: List[RadarTarget]):
        self._targets = targets
        data = [t.to_dict() for t in targets]
        js = json.dumps(data).replace("'", "\\'")
        self._status_label.setText(f"{len(targets)} targets tracked")
        self._run_js(f"updateTargets('{js}')")

    def set_coverage_radius(self, radius_m: float):
        self._coverage_radius = radius_m
        self._run_js(f"setCoverageRadius({radius_m})")

    def set_zoom(self, level: int):
        level = max(0, min(22, level))
        self._run_js(f"setZoom({level})")
