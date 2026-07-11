# Farm UI — Developer Log

## What This Is

A desktop GIS-style farm management interface built with Python and PyQt6. It displays a satellite or aerial image of a farm as a pannable, zoomable map, with farm assets (tanks, bores, irrigation taps, vehicles, etc.) placed on the map at their real GPS coordinates. Clicking an asset shows its live sensor data. A status panel shows system telemetry and a trends panel shows historical averages across 1, 7, 30, and 90 day windows.

This is the GUI layer only. It is designed to sit on top of a separate `core/` system that handles sensor ingestion, a rules engine, and automated output actions. The GUI reads from config YAMLs and will eventually read live data from a database.

---

## Project Structure

```
farm/
├── core/               # rules engine, sensor ingestion, actions (separate system)
├── config/
│   └── registry/
│       ├── water.yaml
│       ├── vehicles.yaml
│       ├── power.yaml
│       └── crops.yaml
├── gui/                # everything below lives here
│   ├── main.py
│   ├── view.py
│   ├── scene.py
│   ├── assets.py
│   ├── app_state.py
│   ├── registry_loader.py
│   ├── status_panel.py
│   ├── trends_panel.py
│   └── icons/
│       ├── tank.png
│       ├── bore.png
│       ├── tap.png
│       ├── vehicle.png
│       ├── solar.png
│       ├── battery.png
│       └── crop.png
```

---

## Running the App

```bash
cd farm/gui
python main.py
```

Requires Python 3.11+ and PyQt6:

```bash
pip install PyQt6 pyyaml
```

The app expects `farm.png` (the aerial/satellite background image) to be present in `gui/`. It will raise a `RuntimeError` if missing.

---

## Architecture

The system enforces one authority per domain. This is intentional and should be maintained.

| Domain | Authority | File |
|---|---|---|
| Camera / zoom / pan | `FarmView` only | `view.py` |
| UI layout | `MainWindow._resize_panels()` only | `main.py` |
| Application state | `AppState` only | `app_state.py` |
| Asset data | YAML files via `registry_loader.py` | `registry_loader.py` |
| GPS ↔ scene coordinates | `SimpleMapProjector` only | `scene.py` |

Mixing these authorities (e.g. calling `fitInView` from outside `view.py`, or sizing panels from inside a panel widget) causes layout and zoom instability. Don't do it.

---

## Key Files

### `main.py`
Entry point. Creates the window, wires all components together, handles F5 scene rebuild and the 60-second vehicle GPS auto-refresh timer. Panel geometry is set here and only here via `_resize_panels()`.

### `view.py`
Handles the map viewport. Zoom is implemented as discrete steps via `ZOOM_STEPS`. The transform is applied as relative `scale(new/old)` increments — never as absolute resets mid-session except on an explicit scene rebuild. Background is dark `#1a1a1a` so empty space beside the map is not white.

### `scene.py`
Builds the `QGraphicsScene`. Places the background image, then places `AssetItem` objects from the loaded asset list. GPS coordinates are converted to scene pixel coordinates via `SimpleMapProjector`. Asset click handlers are wired here.

**Important:** `SimpleMapProjector` has hardcoded GPS bounds. These must be updated to match the actual farm boundary:
```python
self.min_lat = -31.9510
self.max_lat = -31.9500
self.min_lon = 115.8600
self.max_lon = 115.8610
```

### `assets.py`
Defines `AssetItem` (a `QGraphicsItem` subclass). Icons are selected by asset `type` via `ICON_MAP`. If an icon file is missing, the asset renders as a red square so the problem is immediately visible. Icons are fixed screen size (64px) and do not scale with zoom — they stay the same visual size regardless of zoom level.

### `app_state.py`
Central state hub. Holds the currently selected asset, system telemetry dict, and a registry of all `AssetState` objects keyed by `asset_id`. UI components register callbacks here rather than holding references to each other. Supports `trigger_refresh()` for external systems to push updates.

### `registry_loader.py`
Loads all four YAML files and returns a flat list of `Asset` objects. Skips missing files gracefully so the app runs even if some YAMLs don't exist yet.

### `status_panel.py`
Bottom-right overlay panel. Three sections top to bottom: selected asset info (7 lines reserved, fixed height so layout doesn't shift when an asset is selected/deselected), system telemetry (battery, voltage, tank level, temperature), GPS coordinates of the mouse cursor. Updated via callbacks registered in `AppState`.

### `trends_panel.py`
Bottom panel to the left of the status panel. Shows 1d / 7d / 30d / 90d averages for: rain, temperature, wind, battery SOC, battery voltage, tank 1 output, human days, tractor use. Currently uses stub data. Replace `STUB_DATA` with real DB queries and call `update_trends(data)` to push live values.

---

## Asset YAML Format

```yaml
water:
  tank_1:
    id: tank_1
    name: Main Storage Tank 1
    type: tank          # drives icon selection — see ICON_MAP in assets.py
    capacity_l: 27000
    sensors:
      - tank_1_level
      - tank_1_flow_in
      - tank_1_flow_out
    metadata:
      gps: [-31.950331, 115.86021]    # [lat, lon]
      display:                         # optional visual nudge in scene pixels
        offset_x: 0
        offset_y: -20
```

Supported `type` values and their icons:

| type | icon file |
|---|---|
| tank | icons/tank.png |
| bore | icons/bore.png |
| tap | icons/tap.png |
| vehicle | icons/vehicle.png |
| solar | icons/solar.png |
| battery | icons/battery.png |
| crop | icons/crop.png |

Any unrecognised type falls back to `icons/default.png` and renders red if that file is also missing.

---

## Keyboard Shortcuts

| Key | Action |
|---|---|
| F5 | Full scene rebuild — reloads all YAMLs, repositions all assets |
| Scroll wheel | Zoom in / out |
| Click + drag | Pan the map |

---

## Auto-Refresh

Vehicles are expected to have changing GPS coordinates (they move). `main.py` runs a `QTimer` that triggers a full scene rebuild every 60 seconds automatically. This interval can be changed in `MainWindow.__init__`:

```python
self._vehicle_timer.setInterval(60_000)  # milliseconds
```

---

## What Still Needs Doing

- **Database integration** — `trends_panel.py` uses stub data. Real queries need to replace `STUB_DATA` and feed into `update_trends()`. Similarly, `update_system()` on the status panel needs to be called with live telemetry.
- **GPS bounds** — `SimpleMapProjector` in `scene.py` has hardcoded bounds. These should be moved to a config file or derived from the farm's YAML data.
- **Icon assets** — icon PNG files need to be created or sourced for all asset types and placed in `gui/icons/`.
- **Vehicle tracking** — currently vehicles reload from YAML on a timer. If vehicles report GPS live (e.g. via MQTT or a DB), `app_state.update_asset_gps()` can be called directly without a full scene rebuild.
- **Asset panel depth** — the status panel shows up to 7 sensor readings. Assets with more sensors than that are silently truncated. A scrollable view or expandable panel would fix this for complex assets.
- **core/ integration** — the `core/` rules engine outputs actions. Those actions are not yet surfaced in the GUI. An alerts or actions feed would be a natural next addition.

---

## Design Decisions Worth Knowing

**Why no Qt layout manager for panels?**
Qt's layout manager fights with `QGraphicsView`'s transform system on resize. Panels are sized and positioned exclusively via `setGeometry()` in `_resize_panels()`. This is intentional. Do not add panels to a layout.

**Why discrete zoom steps instead of continuous zoom?**
Continuous zoom via `wheelEvent` accumulates floating point error in the transform matrix over time, causing drift. Discrete steps with relative `scale(new/old)` increments keep the transform stable indefinitely.

**Why rebuild the whole scene on F5 instead of updating in place?**
Qt's `QGraphicsScene` does not have a clean partial-update path that handles add/remove/reposition atomically. A full rebuild is cheaper than managing the delta and avoids stale item state. The zoom level is preserved across rebuilds.

---

## Contact / Handover Notes

This project was built iteratively. If you are picking this up fresh:

1. Read this file top to bottom before touching anything.
2. Run `python main.py` from inside `gui/` — not from the project root.
3. The three non-negotiables: one camera authority, one layout authority, one data authority. Violating any of these will reproduce the original bugs (zoom fighting, panel collapse, clustering).
4. The `core/` system is entirely separate — do not import from `gui/` into `core/` or vice versa. The boundary is intentional.
