# ConstellationSim

C++ satellite constellation propagation and mission-design trade-study framework. Runs single simulations or parallel Monte Carlo / Pareto sweeps over constellation design parameters. Optional OpenGL visualizer with real-time playback control.

**Scope.** This project answers orbit-level and mission-design questions — constellation architecture trades, coverage/revisit/capacity, deorbit compliance, station-keeping ΔV budgets, conjunction screening — for many-satellite constellations. It deliberately does **not** simulate individual-satellite attitude dynamics or flight software; that's a different fidelity/scale problem (one high-fidelity vehicle vs. thousands of low-fidelity trajectories) solved by a separate project, [spacecraft-dynamics-sim](https://github.com/vcampos4545/spacecraft-dynamics-sim), a general rigid-body physics engine with reaction wheels, sensor models, and a full closed-loop ADCS example. The two hand off data (e.g. an orbital state / sun vector) rather than sharing an engine.

## Build

```bash
cmake -B build
cmake --build build -j$(nproc)
```

Tests are built by default (see [Testing](#testing) below). Disable with `-DBUILD_TESTS=OFF` if you don't want the Catch2 dependency fetched.

With visualization (requires VGL):
```bash
cmake -B build -DBUILD_VIZ=ON
cmake --build build -j$(nproc)
```

## Testing

```bash
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

Covers: orbital-elements round-tripping and Kepler-equation solving, RK4 energy/angular-momentum conservation, J2 secular RAAN drift against the analytic rate, Walker constellation geometry, SGP4 against the published Spacetrack Report #3 verification vector (and against this project's own RK4+J2 propagator), conjunction screening, station-keeping ΔV accounting, Pareto-frontier extraction, and the link-budget model. CI (`.github/workflows/ci.yml`) builds and runs this suite on every push.

## Running a Simulation

```bash
./build/ConstellationSim config/earth_observation.json
./build/ConstellationSim config/earth_observation.json --visualize
```

**Visualizer controls:**
| Key / Mouse | Action |
|---|---|
| `Space` | Pause / resume |
| `1` `2` `3` `4` | Speed: 1×, 10×, 100×, 1000× |
| `+` / `-` | Step speed up / down |
| Left-drag | Orbit camera |
| Right-drag | Pan |
| Scroll | Zoom |

## Running a Monte Carlo / Pareto Sweep

```bash
./build/ConstellationSim montecarlo/starlink_sweep.json
```

Results are written to `output/<experiment_name>/` as CSV.

---

## Creating a Scenario

### Single simulation — `config/my_scenario.json`

```json
{
  "simulation": {
    "name": "My Constellation",
    "duration_days": 3,
    "timestep_s": 30,
    "epoch_jd": 2451545.0
  },
  "constellation": {
    "type": "walker",
    "altitude_km": 550,
    "inclination_deg": 53,
    "total_satellites": 1584,
    "planes": 72,
    "phasing_factor": 13
  },
  "satellite": {
    "mass_kg": 260,
    "drag_coefficient": 2.2,
    "drag_area_m2": 2.0,
    "reflectivity": 1.3,
    "srp_area_m2": 2.0
  },
  "forces": {
    "gravity": true,
    "j2": true,
    "j3": false,
    "j4": false,
    "drag": true,
    "srp": true,
    "moon": false,
    "sun_gravity": false
  },
  "metrics": {
    "coverage": {
      "enabled": true,
      "grid_resolution_deg": 2,
      "min_elevation_deg": 25,
      "sample_interval_s": 300
    },
    "sunlight": true,
    "drag": true,
    "delta_v": true,
    "revisit": true,
    "conjunction": {
      "enabled": false,
      "threshold_km": 5.0,
      "sample_interval_s": 60.0
    },
    "link_budget": {
      "enabled": false,
      "frequency_ghz": 12.0,
      "eirp_dbw": 45.0,
      "gt_dbk": 15.0,
      "bandwidth_mhz": 50.0,
      "other_losses_db": 3.0
    }
  },
  "station_keeping": {
    "enabled": false,
    "altitude_deadband_km": 5.0,
    "check_interval_s": 3600.0
  },
  "output": {
    "directory": "output",
    "run_name": "my_constellation",
    "trajectory_sample_interval_s": 0,
    "export_oem": false,
    "oem_satellite_ids": []
  }
}
```

### Monte Carlo / Pareto sweep — `montecarlo/my_sweep.json`

```json
{
  "experiment": {
    "name": "altitude_sweep",
    "sampling": "grid",
    "threads": 0
  },
  "base_config": { ... },
  "sweep": {
    "altitude_km":     [400, 500, 600],
    "inclination_deg": [53, 70, 97],
    "planes":          [24, 48, 72]
  },
  "output": {
    "directory": "output",
    "experiment_name": "altitude_sweep"
  }
}
```

`"sampling"`:
- `"grid"` — every combination of the swept parameters.
- `"random"` — samples `runs` points at random (with replacement) from each parameter's list.
- `"pareto"` — runs the full grid (like `"grid"`), then extracts the non-dominated subset by the objectives in `pareto_objectives` and writes it to `pareto_frontier.csv`. This answers questions like *"what's the minimum satellite count for ≥95% coverage under a ΔV budget?"* directly — filter the frontier for your constraint, then read off the extreme. Objectives are `"maximize:<field>"` / `"minimize:<field>"` over any `ConstellationResult` field (e.g. `total_satellites`, `coverage_pct`, `avg_annual_sk_dv_ms_per_year`, `avg_datarate_mbps`):
  ```json
  "pareto_objectives": ["minimize:total_satellites", "maximize:coverage_pct"]
  ```

Set `"threads": 0` to use all CPU cores.

**Sweepable parameters:** `altitude_km`, `inclination_deg`, `planes`, `sats_per_plane`, `total_satellites`, `phasing_factor`, `mass_kg`, `drag_coefficient`, `drag_area_m2`, `timestep_s`, `duration_days`, `min_elevation_deg`

---

## Importing Real Constellations (TLE / SGP4)

[`src/orbit/TLE.h`](src/orbit/TLE.h) parses standard two-line element sets, and [`src/orbit/SGP4.h`](src/orbit/SGP4.h) is a from-scratch reimplementation of the public near-Earth SGP4 algorithm (Vallado/Crawford/Hujsak/Kelso, "Revisiting Spacetrack Report #3"), validated against the paper's published reference vector in `tests/test_sgp4.cpp`. It exists so:

- Real TLE-derived orbital elements can seed a scenario instead of only synthetic Walker/custom constellations.
- This project's own RK4 propagator has an independent analytic cross-check (see [Validation](#validation) below).

Scope: near-Earth objects only (LEO — this project's whole focus); deep-space resonance terms (SDP4, for GEO/Molniya-class orbits) and the low-perigee (<220 km) simplified-drag branch are not implemented. See the caveats documented at the top of `SGP4.h`.

## Ephemeris Export (CCSDS OEM)

Set `output.export_oem: true` (with `output.trajectory_sample_interval_s > 0`) to write one [CCSDS OEM](https://public.ccsds.org/Pubs/502x0b3e1.pdf) file per satellite to `<run_dir>/oem/sat_<id>.oem` — the standard ephemeris interchange format readable by STK, GMAT, FreeFlyer, and most other mission-design tools. Use `output.oem_satellite_ids` to restrict export to specific satellites (recommended for large constellations — exporting all 1500+ satellites in a mega-constellation run is possible but produces a lot of files).

## Conjunction Screening

`metrics.conjunction.enabled: true` runs pairwise closest-approach screening within the constellation, reporting any pair that ever comes within `threshold_km` to `<run_dir>/conjunctions.csv` (sorted closest-first). This is a **discrete** screen — it checks separation at `sample_interval_s`, not continuously, so true closest approach between samples can be missed; treat it as "which pairs/regions are worth a closer look," not operational conjunction assessment. Cost is O(N²) per sample after a one-time radius-band pre-filter that discards pairs whose orbits can never come close (different altitude shells) — within a single Walker shell the pre-filter doesn't help, so keep `sample_interval_s` coarse for large constellations. See [`src/metrics/ConjunctionScreener.h`](src/metrics/ConjunctionScreener.h).

## Station-Keeping ΔV Budgeting

`station_keeping.enabled: true` actively simulates maintaining the constellation's altitude: when a satellite's semi-major axis decays past `altitude_deadband_km` below nominal (checked every `check_interval_s`), a tangential reboost burn restores it, and the ΔV actually spent is accumulated per satellite and reported (`stationkeeping_dv_ms`, annualized as `annual_sk_dv_ms_per_year`, plus `sk_maneuver_count`). This is a real closed control loop over the existing drag model — what a real ops team would burn to hold the shell — not a passive decay estimate.

RAAN/plane-keeping is deliberately not modeled: within a single Walker shell every satellite shares the same SMA and inclination, so J2 nodal drift is common-mode and relative plane spacing is preserved without any ΔV expenditure — a basic property of Walker constellation design, not an omission. See [`src/orbit/StationKeeping.h`](src/orbit/StationKeeping.h).

## Link-Budget-Aware Coverage

`metrics.link_budget.enabled: true` extends ground-target access metrics beyond geometric elevation-mask visibility with a simple free-space link budget (EIRP + ground G/T, Shannon-capacity data rate) — see [`src/environment/LinkBudget.h`](src/environment/LinkBudget.h). Ground target results gain `avg_datarate_mbps`, `min_datarate_mbps`, `peak_datarate_mbps`, computed from the best (highest-elevation) visible satellite's slant range at each sample. This is a trade-study capacity estimate, not an RF engineering design tool — no rain fade, pointing loss, modulation/coding efficiency, or real antenna pattern.

---

## Output

Each run produces CSV files inside `output/<experiment_name>/run_NNNN/`:

| File | Contents |
|---|---|
| `summary.csv` | Coverage %, avg revisit time, drag ΔV, station-keeping ΔV, deorbit lifetime, achievable data rate |
| `satellites.csv` | Per-satellite drag ΔV, station-keeping ΔV/maneuver count, altitude, eclipse fraction, orbital lifetime |
| `ground_targets.csv` | Per-target visibility %, pass count/duration, elevation, achievable data rate (if link budget enabled) |
| `pass_events.csv` | Per-pass AOS/LOS table (if ground targets configured) |
| `trajectory.csv` | Orbital-element time history (if `trajectory_sample_interval_s > 0`) |
| `conjunctions.csv` | Closest-approach pairs under threshold (if conjunction screening enabled) |
| `oem/sat_*.oem` | CCSDS ephemeris per satellite (if OEM export enabled) |

Monte Carlo runs also produce `experiment_summary.csv` (all runs) and, in `"pareto"` sampling mode, `pareto_frontier.csv` (non-dominated runs only).

## Physics

| Model | Notes |
|---|---|
| Two-body gravity | Central force, GM = 3.986004418×10¹⁴ m³/s² |
| J2 oblateness | First zonal harmonic, J2 = 1.08262668×10⁻³ |
| J3 / J4 | Higher-order zonal harmonics (pear-shape, north-south asymmetry) |
| Atmospheric drag | USSA76, 23 layers to 700 km |
| Solar radiation pressure | Cannonball model, Vallado low-precision Sun position |
| Third-body gravity | Sun and Moon, optional |
| SGP4 | Independent analytic propagator for TLE import and cross-validation (see above) |

Integrator: RK4 with configurable timestep.

## Validation

- `tests/test_propagator.cpp`: RK4 conserves specific energy and angular momentum over a full orbit to ~1e-8 relative tolerance, and the J2-only case matches the analytic secular RAAN drift rate to within 5% over 20 orbits.
- `tests/test_sgp4.cpp`: this project's SGP4 implementation matches the published Spacetrack Report #3 / Vallado (AIAA 2006-6753) reference vector at epoch to within a few km / mm·s⁻¹, and agrees with the RK4+J2 propagator to well under a kilometer over a 1-hour span for a real ISS TLE.
- [`analysis/sgp4_validation.ipynb`](analysis/sgp4_validation.ipynb): plots RK4-vs-SGP4 position separation and altitude over 72 hours (regenerate the underlying data with `./build/sgp4_validation`).
- [`analysis/sso_j2_analysis.ipynb`](analysis/sso_j2_analysis.ipynb), [`analysis/reflect_orbital_analysis.ipynb`](analysis/reflect_orbital_analysis.ipynb): longer-horizon orbital-mechanics studies (J2 precession, reflector geometry) from real simulation output.
