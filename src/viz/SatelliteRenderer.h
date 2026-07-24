#pragma once

#ifdef CONSTELLATION_VIZ_ENABLED

#include "viz/FrameQueue.h"
#include "core/ConfigLoader.h"
#include "core/SimulationEngine.h"
#include "core/math/Constants.h"
#include "core/math/Vec3.h"
#include <vgl/vgl.h>
#include <vgl/OrbitalCamera.h>
#include <memory>
#include <vector>
#include <deque>
#include <utility>

// Streams and renders live simulation frames from a FrameQueue.
//
// Features:
//   - Left-click a satellite  : select it; camera tracks it automatically
//   - Escape / click on space : deselect
//   - ImGui telemetry panel   : shows orbital state for the selected satellite
//   - ImGui HUD               : always-visible mission time / speed / sim status
//
// Controls:
//   Space           — pause / resume
//   1 / 2 / 3 / 4  — speed presets: 1×, 10×, 100×, 1000×
//   + / =           — step up to next speed preset
//   -               — step down to previous speed preset
//   Left drag       — orbit camera
//   Right drag      — pan target (breaks satellite tracking)
//   Scroll          — zoom
//   Escape          — deselect satellite
class SatelliteRenderer {
public:
    static constexpr float  SCENE_SCALE     = 1.0f / 6'378'137.0f;
    static constexpr float  EARTH_DISPLAY_R = 1.0f;
    static constexpr float  SAT_DOT_R       = 0.012f;
    static constexpr float  SAT_PICK_R      = 0.035f;  // ray-cast hit radius (larger than visual)

    static constexpr int   TRAIL_FRAMES   = 90;
    static constexpr int   TRAIL_MAX_SATS = 200;

    // Fraction of window width occupied by the data panel in satellite-selected mode
    static constexpr float SPLIT_FRAC = 0.40f;

    SatelliteRenderer(std::shared_ptr<FrameQueue>                    queue,
                      std::vector<GroundTarget>                      ground_targets = {},
                      double                                         min_elevation_deg = 10.0,
                      std::vector<SimulationEngine::SatelliteInfo>   sat_info = {},
                      double                                         epoch_jd = Constants::J2000_JD,
                      int window_w = 1280, int window_h = 720);

    ~SatelliteRenderer();

    // Blocks until the window is closed.
    void run();

private:
    std::shared_ptr<FrameQueue>                   queue_;
    std::vector<SimulationEngine::SatelliteInfo>  sat_info_;
    GUI           gui_;
    OrbitalCamera orbital_cam_;      // planet-scale view (no satellite selected)
    OrbitalCamera orbital_cam_sat_;  // satellite close-up view (satellite selected)

    // ---------------------------------------------------------------------------
    // Playback state
    // ---------------------------------------------------------------------------
    struct Playback {
        double sim_time_s{0.0};
        double speed{1.0};
        bool   paused{false};
        double wall_prev{-1.0};
    } pb_;

    static constexpr double SPEED_PRESETS[4] = {1.0, 10.0, 100.0, 1000.0};
    int speed_preset_idx_{0};

    // ---------------------------------------------------------------------------
    // Input / selection state
    // ---------------------------------------------------------------------------
    glm::vec2 prev_mouse_pos_{0.0f};

    int   selected_sat_idx_{-1};              // -1 = none
    int   prev_selected_idx_{-2};            // used to detect new-selection camera reset
    int   scene_x_win_{0};                   // left edge of 3D viewport (window coords)
    glm::vec2 mouse_at_press_{0.0f};          // position when left-button was pressed
    bool      drag_started_{false};
    static constexpr float CLICK_DRAG_THRESHOLD = 5.0f;  // pixels

    // ---------------------------------------------------------------------------
    // Textures
    // ---------------------------------------------------------------------------
    Texture earth_tex_;
    Texture star_tex_;

    // ---------------------------------------------------------------------------
    // ---------------------------------------------------------------------------
    // Per-render-tick interpolated state
    // ---------------------------------------------------------------------------
    std::vector<glm::vec3> interp_pos_;       // scene-space satellite positions
    std::vector<Vec3>      interp_pos_eci_;   // ECI positions [m]  — for telemetry
    std::vector<Vec3>      interp_vel_eci_;   // ECI velocities [m/s] — for telemetry
    std::vector<bool>      interp_ecl_;
    glm::vec3              interp_sun_{1.0f, 0.0f, 0.0f};
    glm::vec3              interp_moon_{0.0f, 1.0f, 0.0f};
    int                    lo_frame_idx_{-1};
    float                  sim_dt_s_{0.0f};      // derived from consecutive frame timestamps

    // Trail ring-buffer per satellite
    std::vector<std::deque<glm::vec3>> trail_buf_;

    // ---------------------------------------------------------------------------
    // 2D Mercator ground track
    // ---------------------------------------------------------------------------
    // lat/lon history (degrees) for the selected satellite
    std::deque<std::pair<float, float>> ground_track_;
    static constexpr int GROUND_TRACK_MAX = 2000;

    void drawMercatorWindow();

    // ---------------------------------------------------------------------------
    // Ground targets
    // ---------------------------------------------------------------------------
    struct GroundTargetViz {
        std::string name;
        Vec3        pos_ecef;
    };
    std::vector<GroundTargetViz> ground_targets_;
    float                        min_elev_sin_{0.0f};
    double                       min_elevation_rad_{0.0};
    float                        min_elev_deg_ui_{10.0f};  // editable copy for CONFIG slider
    double                       epoch_jd_{Constants::J2000_JD};
    std::vector<glm::vec3>       gt_scene_pos_;

    static constexpr float GT_MARKER_R = 0.018f;

    // ---------------------------------------------------------------------------
    // Panel tab state
    // ---------------------------------------------------------------------------
    enum class PanelTab { Data, Power, Thermal, Faults, Viz };
    PanelTab active_tab_{PanelTab::Data};

    // ---------------------------------------------------------------------------
    // Internal helpers
    // ---------------------------------------------------------------------------
    void handleInput();
    void advancePlayback();
    void buildInterpState();
    void applyTracking();          // override OrbitalCamera target to track selected sat
    void updateWindowTitle();

    void drawStarBackground();
    void drawEarth();
    void drawSunIndicator();
    void drawMoonIndicator();
    void drawSatellites();
    void drawBodyAxes();             // body-frame XYZ arrows for selected satellite
    void drawTrails();
    void drawGroundTargets();
    void drawGroundLinks();
    void drawCoverageFootprint();   // ring on Earth surface for selected satellite

    // ImGui overlays
    void drawHUD();
    void drawSatellitePanel();       // combined left-panel: telemetry + IMU + wheels
    void drawAxesOverlay();          // ECI frame indicator (repositioned in split mode)

    // Returns the index of the satellite under the given screen-space position,
    // or -1 if no satellite was hit.
    int pickSatellite(glm::vec2 mouse_pos) const;

    glm::vec3 eciToScene(const Vec3& eci_m) const;
};

#endif // CONSTELLATION_VIZ_ENABLED
