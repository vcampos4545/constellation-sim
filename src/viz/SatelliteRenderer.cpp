#ifdef CONSTELLATION_VIZ_ENABLED

#include "viz/SatelliteRenderer.h"
#include "core/math/Constants.h"
#include "environment/EarthModel.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <limits>

constexpr double SatelliteRenderer::SPEED_PRESETS[4];

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

SatelliteRenderer::SatelliteRenderer(std::shared_ptr<FrameQueue> queue,
                                     std::vector<GroundTarget> ground_targets,
                                     double min_elevation_deg,
                                     std::vector<SimulationEngine::SatelliteInfo> sat_info,
                                     double epoch_jd,
                                     int window_w, int window_h)
    : queue_(std::move(queue)),
      sat_info_(std::move(sat_info)),
      gui_(window_w, window_h, "ConstellationSim"),
      orbital_cam_(3.5f, 25.0f, 20.0f, glm::vec3(0.0f)),
      orbital_cam_sat_(0.25f, 25.0f, 20.0f, glm::vec3(0.0f)),
      min_elev_sin_(static_cast<float>(std::sin(min_elevation_deg * Constants::DEG2RAD))),
      min_elevation_rad_(min_elevation_deg * Constants::DEG2RAD),
      min_elev_deg_ui_(static_cast<float>(min_elevation_deg)),
      epoch_jd_(epoch_jd)
{
    gui_.camera
        .setFOV(45.0f)
        .setClipPlanes(0.001f, 500.0f)
        .setUp({0.0f, 0.0f, 1.0f});

    gui_.setLogDepth(500.0f);
    gui_.setLighting(true);

    orbital_cam_
        .setMinDistance(1.05f)
        .setMaxDistance(20.0f)
        .setZoomSensitivity(0.3f)
        .setPanSensitivity(0.25f);

    // Satellite close-up camera — reset per-selection in applyTracking()
    orbital_cam_sat_
        .setMinDistance(0.03f)
        .setMaxDistance(0.8f)
        .setZoomSensitivity(0.015f)
        .setPanSensitivity(0.10f);

    earth_tex_.loadFromFile("resources/textures/earth.jpg");
    star_tex_.loadFromFile("resources/textures/stars.jpg");

    for (const auto &gt : ground_targets)
    {
        GroundTargetViz v;
        v.name = gt.name;
        v.pos_ecef = EarthModel::geodeticToECEF(gt.lat_deg, gt.lon_deg, 0.0);
        ground_targets_.push_back(std::move(v));
    }

    pb_.wall_prev = glfwGetTime();

    // --- Dear ImGui init (after GL context is live via GUI constructor) ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    // Tune alpha so panels are readable but not opaque
    ImGui::GetStyle().Alpha = 0.92f;
    ImGui::GetStyle().WindowRounding = 6.0f;

    ImGui_ImplGlfw_InitForOpenGL(gui_.getWindow(), true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

SatelliteRenderer::~SatelliteRenderer()
{
    // Shut down ImGui before the GL context (owned by GUI) is destroyed.
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

// ---------------------------------------------------------------------------
// Coordinate conversion
// ---------------------------------------------------------------------------
glm::vec3 SatelliteRenderer::eciToScene(const Vec3 &eci_m) const
{
    return {
        static_cast<float>(eci_m.x) * SCENE_SCALE,
        static_cast<float>(eci_m.y) * SCENE_SCALE,
        static_cast<float>(eci_m.z) * SCENE_SCALE};
}

// ---------------------------------------------------------------------------
// Playback control
// ---------------------------------------------------------------------------
void SatelliteRenderer::advancePlayback()
{
    const double wall_now = glfwGetTime();
    const double real_dt = wall_now - pb_.wall_prev;
    pb_.wall_prev = wall_now;

    if (pb_.paused)
        return;

    pb_.sim_time_s += real_dt * pb_.speed;

    const double max_t = queue_->maxSimTime();
    if (max_t <= 0.0)
    {
        pb_.sim_time_s = 0.0;
        return;
    }

    if (pb_.sim_time_s > max_t)
    {
        if (queue_->isSimDone())
        {
            pb_.sim_time_s = std::fmod(pb_.sim_time_s, max_t);
            trail_buf_.clear();
            ground_track_.clear();
            lo_frame_idx_ = -1;
        }
        else
        {
            pb_.sim_time_s = max_t;
        }
    }
}

// ---------------------------------------------------------------------------
// Interpolated state
// ---------------------------------------------------------------------------
void SatelliteRenderer::buildInterpState()
{
    const int new_lo = queue_->findIdx(pb_.sim_time_s);
    if (new_lo < 0)
        return;

    SimulationEngine::FrameData lo_frame, hi_frame;
    const bool has_hi = queue_->copyFramePair(new_lo, new_lo + 1, lo_frame, hi_frame);

    float alpha = 0.0f;
    if (has_hi)
    {
        const double dt = hi_frame.time_s - lo_frame.time_s;
        if (dt > 0.0)
        {
            alpha = std::clamp(
                static_cast<float>((pb_.sim_time_s - lo_frame.time_s) / dt),
                0.0f, 1.0f);
            if (sim_dt_s_ == 0.0f)
                sim_dt_s_ = static_cast<float>(dt);
        }
    }

    const int n = static_cast<int>(lo_frame.positions.size());

    if (new_lo != lo_frame_idx_)
    {
        if (static_cast<int>(trail_buf_.size()) != n)
            trail_buf_.assign(n, std::deque<glm::vec3>{});

        for (int i = 0; i < n; ++i)
        {
            trail_buf_[i].push_back(eciToScene(lo_frame.positions[i]));
            while (static_cast<int>(trail_buf_[i].size()) > TRAIL_FRAMES + 1)
                trail_buf_[i].pop_front();
        }

        lo_frame_idx_ = new_lo;
    }

    interp_pos_.resize(n);
    interp_pos_eci_.resize(n);
    interp_vel_eci_.resize(n);

    for (int i = 0; i < n; ++i)
    {
        const glm::vec3 a = eciToScene(lo_frame.positions[i]);
        const glm::vec3 b = has_hi ? eciToScene(hi_frame.positions[i]) : a;
        interp_pos_[i] = glm::mix(a, b, alpha);

        // ECI position (unscaled) for telemetry
        const Vec3 &p0 = lo_frame.positions[i];
        if (has_hi)
        {
            const Vec3 &p1 = hi_frame.positions[i];
            interp_pos_eci_[i] = {
                p0.x + alpha * (p1.x - p0.x),
                p0.y + alpha * (p1.y - p0.y),
                p0.z + alpha * (p1.z - p0.z)};
        }
        else
        {
            interp_pos_eci_[i] = p0;
        }

        // ECI velocity (unscaled) for telemetry
        const bool has_vel = (i < (int)lo_frame.velocities.size());
        if (has_vel)
        {
            const Vec3 &v0 = lo_frame.velocities[i];
            if (has_hi && i < (int)hi_frame.velocities.size())
            {
                const Vec3 &v1 = hi_frame.velocities[i];
                interp_vel_eci_[i] = {
                    v0.x + alpha * (v1.x - v0.x),
                    v0.y + alpha * (v1.y - v0.y),
                    v0.z + alpha * (v1.z - v0.z)};
            }
            else
            {
                interp_vel_eci_[i] = v0;
            }
        }
        else
        {
            interp_vel_eci_[i] = {};
        }
    }

    interp_ecl_ = lo_frame.in_eclipse;
    interp_sun_ = {
        static_cast<float>(lo_frame.sun_dir_eci.x),
        static_cast<float>(lo_frame.sun_dir_eci.y),
        static_cast<float>(lo_frame.sun_dir_eci.z)};
    interp_moon_ = {
        static_cast<float>(lo_frame.moon_dir_eci.x),
        static_cast<float>(lo_frame.moon_dir_eci.y),
        static_cast<float>(lo_frame.moon_dir_eci.z)};

    {
        const double gmst = EarthModel::gmst_rad(epoch_jd_ + pb_.sim_time_s / Constants::SEC_PER_DAY);
        gt_scene_pos_.resize(ground_targets_.size());
        for (size_t i = 0; i < ground_targets_.size(); ++i)
        {
            const Vec3 eci = EarthModel::ecefToECI(ground_targets_[i].pos_ecef, gmst);
            gt_scene_pos_[i] = eciToScene(eci);
        }
    }
}

// ---------------------------------------------------------------------------
// Camera tracking
// ---------------------------------------------------------------------------
void SatelliteRenderer::applyTracking()
{
    if (selected_sat_idx_ < 0 || selected_sat_idx_ >= (int)interp_pos_.size())
    {
        prev_selected_idx_ = -2; // ensure a reset on next selection
        orbital_cam_.applyToCamera(gui_.camera);
        return;
    }

    // On a new satellite selection, reset the satellite camera to a clean close-up view.
    // Assignment reconstructs with a fixed initial distance (0.25 scene units ≈ 1600 km).
    if (selected_sat_idx_ != prev_selected_idx_)
    {
        orbital_cam_sat_ = OrbitalCamera(0.25f, 25.0f, 20.0f,
                                         interp_pos_[selected_sat_idx_]);
        orbital_cam_sat_
            .setMinDistance(0.03f)
            .setMaxDistance(0.8f)
            .setZoomSensitivity(0.015f)
            .setPanSensitivity(0.10f);
        prev_selected_idx_ = selected_sat_idx_;
    }

    orbital_cam_sat_.setTarget(interp_pos_[selected_sat_idx_]);
    orbital_cam_sat_.applyToCamera(gui_.camera);
}

void SatelliteRenderer::updateWindowTitle()
{
    const double t = pb_.sim_time_s;
    const int days = static_cast<int>(t / 86400.0);
    const int hrs = static_cast<int>(std::fmod(t, 86400.0) / 3600.0);
    const int mins = static_cast<int>(std::fmod(t, 3600.0) / 60.0);
    const int n_sats = static_cast<int>(interp_pos_.size());

    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "ConstellationSim | T+%dd %02dh %02dm | %d sats | Speed: %.0f×%s%s",
                  days, hrs, mins, n_sats,
                  pb_.speed,
                  pb_.paused ? " [PAUSED]" : "",
                  !queue_->isSimDone() ? " [SIMULATING...]" : "");

    glfwSetWindowTitle(gui_.getWindow(), buf);
}

// ---------------------------------------------------------------------------
// Satellite picking — ray-sphere intersection
// ---------------------------------------------------------------------------
int SatelliteRenderer::pickSatellite(glm::vec2 mouse_pos) const
{
    if (interp_pos_.empty())
        return -1;

    const glm::vec3 ray_dir = glm::normalize(gui_.getMouseRay(mouse_pos));
    const glm::vec3 ray_orig = gui_.camera.position;

    int best_idx = -1;
    float best_t = std::numeric_limits<float>::max();

    for (int i = 0; i < (int)interp_pos_.size(); ++i)
    {
        const glm::vec3 oc = ray_orig - interp_pos_[i];
        const float b = glm::dot(oc, ray_dir); // half-b form
        const float c = glm::dot(oc, oc) - SAT_PICK_R * SAT_PICK_R;
        const float disc = b * b - c;
        if (disc < 0.0f)
            continue;
        const float t = -b - std::sqrt(disc);
        if (t > 0.0f && t < best_t)
        {
            best_t = t;
            best_idx = i;
        }
    }
    return best_idx;
}

// ---------------------------------------------------------------------------
// Input handling
// ---------------------------------------------------------------------------
void SatelliteRenderer::handleInput()
{
    const bool imgui_wants_kb = ImGui::GetIO().WantCaptureKeyboard;
    const bool imgui_wants_mouse = ImGui::GetIO().WantCaptureMouse;

    if (!imgui_wants_kb)
    {
        if (gui_.isKeyJustPressed(GLFW_KEY_SPACE))
            pb_.paused = !pb_.paused;

        if (gui_.isKeyJustPressed(GLFW_KEY_ESCAPE))
            selected_sat_idx_ = -1;

        const int preset_keys[4] = {GLFW_KEY_1, GLFW_KEY_2, GLFW_KEY_3, GLFW_KEY_4};
        for (int i = 0; i < 4; ++i)
        {
            if (gui_.isKeyJustPressed(preset_keys[i]))
            {
                speed_preset_idx_ = i;
                pb_.speed = SPEED_PRESETS[i];
            }
        }

        const bool inc = gui_.isKeyJustPressed(GLFW_KEY_EQUAL) || gui_.isKeyJustPressed(GLFW_KEY_KP_ADD);
        const bool dec = gui_.isKeyJustPressed(GLFW_KEY_MINUS) || gui_.isKeyJustPressed(GLFW_KEY_KP_SUBTRACT);
        if (inc)
        {
            speed_preset_idx_ = std::min(speed_preset_idx_ + 1, 3);
            pb_.speed = SPEED_PRESETS[speed_preset_idx_];
        }
        if (dec)
        {
            speed_preset_idx_ = std::max(speed_preset_idx_ - 1, 0);
            pb_.speed = SPEED_PRESETS[speed_preset_idx_];
        }
    }

    const glm::vec2 mouse_pos = gui_.getMousePosition();
    const bool left_just_pressed = gui_.isMouseButtonJustPressed(GLFW_MOUSE_BUTTON_LEFT);
    const bool left_just_released = gui_.isMouseButtonJustReleased(GLFW_MOUSE_BUTTON_LEFT);
    const bool left_held = gui_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT);
    const bool right_held = gui_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT);
    const bool just_pressed_any = left_just_pressed || gui_.isMouseButtonJustPressed(GLFW_MOUSE_BUTTON_RIGHT);

    // --- Click detection: track drag distance from press point ---
    if (left_just_pressed && !imgui_wants_mouse)
    {
        mouse_at_press_ = mouse_pos;
        drag_started_ = false;
    }
    if (left_held && !drag_started_)
    {
        if (glm::length(mouse_pos - mouse_at_press_) > CLICK_DRAG_THRESHOLD)
            drag_started_ = true;
    }

    // On clean click-release, pick a satellite (or deselect on miss)
    if (left_just_released && !drag_started_ && !imgui_wants_mouse)
        selected_sat_idx_ = pickSatellite(mouse_pos);

    if (imgui_wants_mouse)
    {
        prev_mouse_pos_ = mouse_pos;
        return;
    }

    glm::vec2 mouse_delta{0.0f};
    if ((left_held || right_held) && !just_pressed_any)
        mouse_delta = mouse_pos - prev_mouse_pos_;
    prev_mouse_pos_ = mouse_pos;

    // Route orbit control to the active camera
    if (selected_sat_idx_ >= 0)
    {
        orbital_cam_sat_.handleInput(gui_, mouse_delta, gui_.getScrollDelta());
        orbital_cam_sat_.applyToCamera(gui_.camera);
    }
    else
    {
        orbital_cam_.handleInput(gui_, mouse_delta, gui_.getScrollDelta());
        orbital_cam_.applyToCamera(gui_.camera);
    }
}

// ---------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------
void SatelliteRenderer::drawStarBackground()
{
    // Equirectangular celestial texture in equatorial coordinates (RA/Dec).
    // VGL sphere mesh + stbi flip places the north celestial pole content at
    // mesh -Y and RA=0h (vernal equinox) at mesh +X.
    // R_x(-90°) maps -Y → +Z and keeps +X → +X, aligning NCP with ECI +Z
    // and the vernal equinox with ECI +X.
    static const glm::quat STAR_BASE_ROT =
        glm::angleAxis(-glm::half_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));

    // Center the sphere on the camera so stars track the view with no parallax.
    // Disable depth writes so the background never occludes scene objects.
    glDepthMask(GL_FALSE);
    gui_.setLighting(false);
    gui_.drawTexturedSphere(gui_.camera.position, 100.0f, STAR_BASE_ROT, star_tex_);
    gui_.setLighting(true);
    glDepthMask(GL_TRUE);
}

void SatelliteRenderer::drawEarth()
{
    // The VGL sphere mesh places the north pole at +Y and the prime meridian
    // at -X (u=0.5 maps to theta=π). Combined with stbi's vertical flip,
    // Earth's north pole renders at -Y and prime meridian at -X.
    // Fix: 180° rotation around (0,1,-1)/√2 sends -Y→+Z and -X→+X.
    // EARTH_BASE_ROT: fixes mesh orientation so north pole→+Z, prime meridian→+X at GMST=0.
    // Sidereal rotation: spin around +Z by current GMST so Greenwich tracks the real Earth.
    static const glm::quat EARTH_BASE_ROT =
        glm::angleAxis(glm::pi<float>(), glm::normalize(glm::vec3(0.0f, 1.0f, -1.0f)));
    const float gmst_f = static_cast<float>(
        EarthModel::gmst_rad(epoch_jd_ + pb_.sim_time_s / Constants::SEC_PER_DAY));
    const glm::quat sidereal_rot = glm::angleAxis(gmst_f, glm::vec3(0.0f, 0.0f, 1.0f));

    gui_.setLightDirection(interp_sun_);
    gui_.drawTexturedSphere({0.0f, 0.0f, 0.0f}, EARTH_DISPLAY_R,
                            sidereal_rot * EARTH_BASE_ROT, earth_tex_);
}

void SatelliteRenderer::drawSunIndicator()
{
    gui_.setLighting(false);
    const float len = EARTH_DISPLAY_R * 2.2f;
    gui_.drawArrow({0.0f, 0.0f, 0.0f}, interp_sun_ * len, {1.0f, 0.95f, 0.3f}, 1.5f);
    gui_.setLighting(true);
}

void SatelliteRenderer::drawMoonIndicator()
{
    gui_.setLighting(false);
    const float len = EARTH_DISPLAY_R * 1.8f; // slightly shorter than sun arrow
    gui_.drawArrow({0.0f, 0.0f, 0.0f}, interp_moon_ * len, {0.75f, 0.80f, 0.90f}, 1.5f);
    gui_.setLighting(true);
}

void SatelliteRenderer::drawSatellites()
{
    if (interp_pos_.empty())
        return;
    const int n = static_cast<int>(interp_pos_.size());

    // --- Unselected satellites: plain spheres ---
    gui_.setLighting(false);
    for (int i = 0; i < n; ++i)
    {
        if (i == selected_sat_idx_)
            continue;

        const bool in_ecl = (i < (int)interp_ecl_.size() && interp_ecl_[i]);
        const glm::vec3 col = in_ecl
                                  ? glm::vec3{0.30f, 0.30f, 0.55f}
                                  : glm::vec3{1.00f, 0.95f, 0.65f};
        gui_.drawSphere(interp_pos_[i], SAT_DOT_R, col);
    }

    // --- Selected satellite: selection halo + lit cube ---
    if (selected_sat_idx_ >= 0 && selected_sat_idx_ < n)
    {
        const int idx = selected_sat_idx_;
        static constexpr glm::vec3 SAT_BOX{0.010f, 0.010f, 0.005f};
        gui_.setLighting(true);
        gui_.drawBox(interp_pos_[idx], SAT_BOX, glm::quat(1.0f, 0.0f, 0.0f, 0.0f), {0.85f, 0.92f, 1.0f});
    }

    gui_.setLighting(true);
}

void SatelliteRenderer::drawTrails()
{
    const int n = static_cast<int>(trail_buf_.size());
    if (n == 0 || n > TRAIL_MAX_SATS)
        return;

    gui_.setLighting(false);
    for (int s = 0; s < n; ++s)
    {
        const auto &buf = trail_buf_[s];
        const int sz = static_cast<int>(buf.size());
        if (sz == 0)
            continue;

        // Highlight the selected satellite's trail
        const bool is_selected = (s == selected_sat_idx_);

        for (int f = 1; f < sz; ++f)
        {
            const float t = static_cast<float>(f) / sz;
            glm::vec3 c;
            if (is_selected)
                c = glm::mix(glm::vec3{0.0f, 0.3f, 0.5f}, glm::vec3{0.0f, 0.85f, 1.0f}, t);
            else
                c = glm::mix(glm::vec3{0.05f, 0.2f, 0.05f}, glm::vec3{0.2f, 0.8f, 0.3f}, t);
            gui_.drawLine(buf[f - 1], buf[f], c, is_selected ? 1.5f : 1.0f);
        }

        if (s < (int)interp_pos_.size())
            gui_.drawLine(buf.back(), interp_pos_[s],
                          is_selected ? glm::vec3{0.0f, 0.85f, 1.0f} : glm::vec3{0.2f, 0.8f, 0.3f},
                          is_selected ? 1.5f : 1.0f);
    }
    gui_.setLighting(true);
}

void SatelliteRenderer::drawGroundTargets()
{
    if (gt_scene_pos_.empty())
        return;
    gui_.setLighting(false);
    for (const auto &pos : gt_scene_pos_)
        gui_.drawSphere(pos, GT_MARKER_R, {1.0f, 0.55f, 0.05f});
    gui_.setLighting(true);
}

void SatelliteRenderer::drawGroundLinks()
{
    if (gt_scene_pos_.empty() || interp_pos_.empty())
        return;
    const int n_sats = static_cast<int>(interp_pos_.size());
    const int n_targets = static_cast<int>(gt_scene_pos_.size());

    gui_.setLighting(false);
    for (int t = 0; t < n_targets; ++t)
    {
        const glm::vec3 &gnd = gt_scene_pos_[t];
        const glm::vec3 gnd_hat = glm::normalize(gnd);

        for (int s = 0; s < n_sats; ++s)
        {
            if (s < (int)interp_ecl_.size() && interp_ecl_[s])
                continue;

            const glm::vec3 slant = interp_pos_[s] - gnd;
            const float sin_elev = glm::dot(gnd_hat, glm::normalize(slant));
            if (sin_elev >= min_elev_sin_)
            {
                const float brightness = 0.4f + 0.6f * (sin_elev - min_elev_sin_) / (1.0f - min_elev_sin_);
                gui_.drawLine(gnd, interp_pos_[s],
                              {brightness, brightness * 0.85f, 0.1f}, 0.8f);
            }
        }
    }
    gui_.setLighting(true);
}

// ---------------------------------------------------------------------------
// Coverage footprint
// ---------------------------------------------------------------------------

void SatelliteRenderer::drawCoverageFootprint()
{
    if (selected_sat_idx_ < 0 || selected_sat_idx_ >= (int)interp_pos_.size())
        return;

    const int idx = selected_sat_idx_;
    const double r = interp_pos_eci_[idx].norm(); // satellite radius [m]
    const double R = Constants::EARTH_RADIUS_M;

    // Earth central half-angle for coverage at configured min-elevation:
    //   sin(η) = R * cos(ε) / r   (nadir angle)
    //   ρ      = π/2 − ε − η
    const double eps = min_elevation_rad_;
    const double cos_eps = std::cos(eps);
    const double eta = std::asin(std::clamp(R * cos_eps / r, 0.0, 1.0));
    const double rho = Constants::PI / 2.0 - eps - eta;
    if (rho <= 0.0)
        return;

    const float cos_rho = static_cast<float>(std::cos(rho));
    const float sin_rho = static_cast<float>(std::sin(rho));

    // Sub-satellite direction in scene space
    const glm::vec3 sub = glm::normalize(interp_pos_[idx]);

    // Orthonormal basis perpendicular to sub
    const glm::vec3 ref = (std::abs(glm::dot(sub, {0.0f, 0.0f, 1.0f})) > 0.99f)
                              ? glm::vec3{1.0f, 0.0f, 0.0f}
                              : glm::vec3{0.0f, 0.0f, 1.0f};
    const glm::vec3 t1 = glm::normalize(glm::cross(sub, ref));
    const glm::vec3 t2 = glm::cross(sub, t1); // already unit (sub, t1 orthonormal)

    // Draw N line-segments tracing the small circle on the sphere surface.
    // Each point: dir = sub*cos_rho + (t1*cos θ + t2*sin θ)*sin_rho   (unit vector)
    // Scaled to 1.003× Earth radius to sit just above the surface.
    constexpr int N = 96;
    constexpr float OFFSET = EARTH_DISPLAY_R * 1.003f;
    constexpr float PI2 = static_cast<float>(Constants::TWO_PI);

    gui_.setLighting(false);

    glm::vec3 prev;
    for (int i = 0; i <= N; ++i)
    {
        const float angle = PI2 * i / N;
        const glm::vec3 dir = sub * cos_rho + t1 * std::cos(angle) * sin_rho + t2 * std::sin(angle) * sin_rho;
        // dir is already unit length (sub,t1,t2 orthonormal)
        const glm::vec3 pt = dir * OFFSET;
        if (i > 0)
            gui_.drawLine(prev, pt, {0.0f, 0.85f, 1.0f}, 1.8f);
        prev = pt;
    }

    // Dashed nadir line: Earth surface → satellite (every other segment)
    {
        const glm::vec3 surface_pt = sub * EARTH_DISPLAY_R;
        const glm::vec3 sat_pt = interp_pos_[idx];
        constexpr int SEGS = 20;
        for (int i = 0; i < SEGS; i += 2)
        { // skip every other → dashed look
            const glm::vec3 a = glm::mix(surface_pt, sat_pt, float(i) / SEGS);
            const glm::vec3 b = glm::mix(surface_pt, sat_pt, float(i + 1) / SEGS);
            gui_.drawLine(a, b, {0.0f, 0.65f, 0.9f}, 0.8f);
        }
    }

    gui_.setLighting(true);
}

// drawMercatorWindow() — logic moved into drawSatellitePanel() GROUND TRACK section.
void SatelliteRenderer::drawMercatorWindow() {}

// ---------------------------------------------------------------------------
// ImGui overlays
// ---------------------------------------------------------------------------

void SatelliteRenderer::drawHUD()
{
    const double t = pb_.sim_time_s;
    const int days = static_cast<int>(t / 86400.0);
    const int hrs = static_cast<int>(std::fmod(t, 86400.0) / 3600.0);
    const int mins = static_cast<int>(std::fmod(t, 3600.0) / 60.0);
    const int secs = static_cast<int>(std::fmod(t, 60.0));
    const int n_sats = static_cast<int>(interp_pos_.size());

    ImGui::SetNextWindowPos({4.0f, 4.0f});
    ImGui::SetNextWindowBgAlpha(0.55f);
    ImGui::Begin("##hud", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);

    // Mission time
    ImGui::TextColored({0.7f, 0.9f, 1.0f, 1.0f}, "T+");
    ImGui::SameLine(0, 4);
    ImGui::Text("%02dd %02dh %02dm %02ds", days, hrs, mins, secs);

    ImGui::SameLine(0, 16);
    ImGui::TextColored({0.9f, 0.9f, 0.5f, 1.0f}, "%.0fx", pb_.speed);

    ImGui::SameLine(0, 8);
    if (pb_.paused)
        ImGui::TextColored({1.0f, 0.8f, 0.1f, 1.0f}, "PAUSED");
    else
        ImGui::TextColored({0.5f, 0.5f, 0.5f, 1.0f}, "RUNNING");

    if (!queue_->isSimDone())
    {
        ImGui::SameLine(0, 8);
        ImGui::TextColored({0.3f, 1.0f, 0.4f, 1.0f}, "[SIMULATING...]");
    }

    ImGui::SameLine(0, 16);
    ImGui::TextColored({0.75f, 0.75f, 0.75f, 1.0f}, "%d sats", n_sats);

    if (selected_sat_idx_ < 0 && !interp_pos_.empty())
    {
        ImGui::SameLine(0, 16);
        ImGui::TextDisabled("Click a satellite to select");
    }

    ImGui::End();
}

// ---------------------------------------------------------------------------
// Combined satellite data panel — full-height left panel in split-screen mode
// ---------------------------------------------------------------------------
void SatelliteRenderer::drawSatellitePanel()
{
    if (selected_sat_idx_ < 0 || selected_sat_idx_ >= (int)interp_pos_.size())
    {
        ground_track_.clear();
        return;
    }

    const int idx = selected_sat_idx_;
    const ImGuiIO &io = ImGui::GetIO();
    const float panel_w = io.DisplaySize.x * SPLIT_FRAC;
    const float panel_h = io.DisplaySize.y;

    // ── Orbital quantities ────────────────────────────────────────────────
    const Vec3 &pos = interp_pos_eci_[idx];
    const Vec3 &vel = interp_vel_eci_[idx];
    const double r = pos.norm();
    const double R_m = Constants::EARTH_RADIUS_M;
    const double R_km = R_m / 1000.0;
    const double alt_km = (r - R_m) / 1000.0;

    const double eta = std::asin(std::clamp(R_m * std::cos(min_elevation_rad_) / r, 0.0, 1.0));
    const double rho = Constants::PI / 2.0 - min_elevation_rad_ - eta;
    const double cov_radius_km = R_km * rho;
    const double cov_area_km2 = Constants::TWO_PI * R_km * R_km * (1.0 - std::cos(rho));

    const double gst = EarthModel::gmst_rad(epoch_jd_ + pb_.sim_time_s / Constants::SEC_PER_DAY);
    const double cx = std::cos(gst), sx = std::sin(gst);
    const double ecef_x = pos.x * cx + pos.y * sx;
    const double ecef_y = -pos.x * sx + pos.y * cx;
    const double ecef_z = pos.z;
    const float lat_deg = static_cast<float>(
        std::atan2(ecef_z, std::sqrt(ecef_x * ecef_x + ecef_y * ecef_y)) * Constants::RAD2DEG);
    const float lon_deg = static_cast<float>(
        std::atan2(ecef_y, ecef_x) * Constants::RAD2DEG);

    const double speed_kms = vel.norm() / 1000.0;
    const double period_min = (Constants::TWO_PI * std::sqrt(r * r * r / Constants::GM_EARTH)) / 60.0;
    const Vec3 h_orb = pos.cross(vel);
    const double h_norm_val = h_orb.norm();
    const double inc_deg = (h_norm_val > 0.0)
                               ? std::acos(std::clamp(h_orb.z / h_norm_val, -1.0, 1.0)) * Constants::RAD2DEG
                               : 0.0;

    const bool in_ecl = (idx < (int)interp_ecl_.size()) && interp_ecl_[idx];

    // ── Ground track update ───────────────────────────────────────────────
    ground_track_.push_back({lat_deg, lon_deg});
    if ((int)ground_track_.size() > GROUND_TRACK_MAX)
        ground_track_.pop_front();

    // ── Style ─────────────────────────────────────────────────────────────
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f, 0.07f, 0.11f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TitleBg, ImVec4(0.04f, 0.12f, 0.25f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.0f, 0.22f, 0.45f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.15f, 0.25f, 0.45f, 0.9f));
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f, 0.2f, 0.4f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.15f, 0.35f, 0.65f, 1.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14.0f, 10.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 5.0f));

    // ── Window ────────────────────────────────────────────────────────────
    char win_title[64];
    if (idx < (int)sat_info_.size())
        std::snprintf(win_title, sizeof(win_title), "SAT-%03d", sat_info_[idx].id);
    else
        std::snprintf(win_title, sizeof(win_title), "SAT-%03d", idx);

    ImGui::SetNextWindowPos({0.0f, 0.0f}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({panel_w, panel_h}, ImGuiCond_Always);
    ImGui::Begin(win_title, nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse);

    // ── Header row ───────────────────────────────────────────────────────
    ImGui::TextColored({0.0f, 0.85f, 1.0f, 1.0f}, "%s", win_title);
    if (idx < (int)sat_info_.size())
    {
        ImGui::SameLine(0, 10);
        ImGui::TextDisabled("Plane %d  Seat %d",
                            sat_info_[idx].plane_id, sat_info_[idx].seat_id);
    }
    {
        const char *badge_txt = in_ecl ? "[ECL] ECLIPSE" : "[SUN] SUNLIT";
        const ImVec4 badge_col = in_ecl
                                     ? ImVec4{0.35f, 0.40f, 0.90f, 1.0f}
                                     : ImVec4{1.00f, 0.88f, 0.25f, 1.0f};
        const float badge_w = ImGui::CalcTextSize(badge_txt).x + 4.0f;
        ImGui::SameLine(panel_w - badge_w - 28.0f);
        ImGui::TextColored(badge_col, "%s", badge_txt);
    }

    // ── Mission time (mini-HUD) ───────────────────────────────────────────
    const double t = pb_.sim_time_s;
    ImGui::TextColored({0.40f, 0.50f, 0.75f, 1.0f},
                       "T+ %02dd %02dh %02dm %02ds",
                       (int)(t / 86400), (int)(fmod(t, 86400) / 3600),
                       (int)(fmod(t, 3600) / 60), (int)fmod(t, 60));
    ImGui::SameLine(0, 16);
    ImGui::TextColored({0.85f, 0.85f, 0.45f, 1.0f}, "%.0fx", pb_.speed);
    if (pb_.paused)
    {
        ImGui::SameLine(0, 8);
        ImGui::TextColored({1.0f, 0.6f, 0.1f, 1.0f}, "PAUSED");
    }
    if (!queue_->isSimDone())
    {
        ImGui::SameLine(0, 8);
        ImGui::TextColored({0.25f, 1.0f, 0.4f, 1.0f}, "[SIM]");
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // ── Tab bar ───────────────────────────────────────────────────────────
    {
        struct TabDef { PanelTab tab; const char* label; };
        static constexpr int NUM_TABS = 5;
        const TabDef tabs[NUM_TABS] = {
            {PanelTab::Data,    "Data"},
            {PanelTab::Power,   "Power"},
            {PanelTab::Thermal, "Thermal"},
            {PanelTab::Faults,  "Faults"},
            {PanelTab::Viz,     "Viz"},
        };

        const float avail_w = ImGui::GetContentRegionAvail().x;
        const float btn_spacing = 4.0f;
        const float btn_w = (avail_w - btn_spacing * (NUM_TABS - 1)) / NUM_TABS;
        const float btn_h = 26.0f;

        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 4.0f);
        for (int i = 0; i < NUM_TABS; ++i)
        {
            if (i > 0) ImGui::SameLine(0, btn_spacing);

            const bool is_active = (active_tab_ == tabs[i].tab);
            if (is_active)
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.35f, 0.7f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.0f, 0.40f, 0.8f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            }
            else
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.14f, 0.22f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.18f, 0.22f, 0.35f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.55f, 0.60f, 0.70f, 1.0f));
            }

            if (ImGui::Button(tabs[i].label, {btn_w, btn_h}))
                active_tab_ = tabs[i].tab;

            ImGui::PopStyleColor(3);
        }
        ImGui::PopStyleVar();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // ── Tab content ───────────────────────────────────────────────────────
    const float col0 = panel_w * 0.42f;

    switch (active_tab_)
    {
    // ══════════════════════════════════════════════════════════════════════
    case PanelTab::Power:
    {
        ImGui::TextColored({0.5f, 0.7f, 1.0f, 1.0f}, "ELECTRICAL POWER SYSTEM");
        ImGui::Spacing();
        ImGui::TextDisabled("No power telemetry available.");
        break;
    }

    // ══════════════════════════════════════════════════════════════════════
    case PanelTab::Thermal:
    {
        ImGui::TextColored({0.5f, 0.7f, 1.0f, 1.0f}, "THERMAL CONTROL");
        ImGui::Spacing();
        ImGui::TextDisabled("No thermal telemetry available.");
        break;
    }

    // ══════════════════════════════════════════════════════════════════════
    case PanelTab::Data:
    {
        ImGui::TextColored({0.5f, 0.7f, 1.0f, 1.0f}, "ORBIT & POSITION DATA");
        ImGui::Spacing();

        // Position
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::CollapsingHeader("POSITION"))
        {
            ImGui::Columns(2, "pos_cols", false);
            ImGui::SetColumnWidth(0, col0);

            ImGui::TextDisabled("Altitude");
            ImGui::NextColumn();
            ImGui::Text("%.1f km", alt_km);
            ImGui::NextColumn();

            ImGui::TextDisabled("Lat / Lon");
            ImGui::NextColumn();
            ImGui::Text("%+.3f deg  %+.3f deg", lat_deg, lon_deg);
            ImGui::NextColumn();

            ImGui::TextDisabled("Cov. radius");
            ImGui::NextColumn();
            ImGui::Text("%.0f km", cov_radius_km);
            ImGui::NextColumn();
            ImGui::TextDisabled("Cov. area");
            ImGui::NextColumn();
            ImGui::Text("%.0f km^2", cov_area_km2);
            ImGui::NextColumn();

            ImGui::Columns(1);
        }

        // Orbit
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::CollapsingHeader("ORBIT"))
        {
            ImGui::Columns(2, "orb_cols", false);
            ImGui::SetColumnWidth(0, col0);

            ImGui::TextDisabled("Speed");
            ImGui::NextColumn();
            ImGui::Text("%.3f km/s", speed_kms);
            ImGui::NextColumn();
            ImGui::TextDisabled("Period");
            ImGui::NextColumn();
            ImGui::Text("%.1f min", period_min);
            ImGui::NextColumn();
            ImGui::TextDisabled("Inclination");
            ImGui::NextColumn();
            ImGui::Text("%.2f deg", inc_deg);
            ImGui::NextColumn();

            ImGui::Columns(1);
        }
        break;
    }

    // ══════════════════════════════════════════════════════════════════════
    case PanelTab::Faults:
    {
        ImGui::TextColored({0.5f, 0.7f, 1.0f, 1.0f}, "FAULT MANAGEMENT");
        ImGui::Spacing();
        ImGui::TextDisabled("No active faults.");
        break;
    }

    // ══════════════════════════════════════════════════════════════════════
    case PanelTab::Viz:
    {
        ImGui::TextColored({0.5f, 0.7f, 1.0f, 1.0f}, "VISUALIZATION & CONFIG");
        ImGui::Spacing();

        // Ground track
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::CollapsingHeader("GROUND TRACK"))
        {
            if (ImGui::BeginChild("##gt_child", {0.0f, 180.0f}, false, ImGuiWindowFlags_NoScrollbar))
            {
                const ImVec2 cp = ImGui::GetCursorScreenPos();
                const ImVec2 cs = ImGui::GetContentRegionAvail();

                ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(earth_tex_.id())),
                             cs, ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));

                ImDrawList *dl = ImGui::GetWindowDrawList();

                auto toScreen = [&](float lat, float lon) -> ImVec2
                {
                    return {cp.x + (lon + 180.0f) / 360.0f * cs.x,
                            cp.y + (90.0f - lat) / 180.0f * cs.y};
                };

                const int n_trail = static_cast<int>(ground_track_.size());
                for (int i = 1; i < n_trail; ++i)
                {
                    const float lon0 = ground_track_[i - 1].second;
                    const float lon1 = ground_track_[i].second;
                    if (std::abs(lon1 - lon0) > 90.0f)
                        continue;
                    const float a = static_cast<float>(i) / n_trail;
                    const ImU32 col = IM_COL32(
                        static_cast<int>(20 + 20 * a),
                        static_cast<int>(120 + 135 * a),
                        static_cast<int>(220 - 20 * a), 220);
                    dl->AddLine(toScreen(ground_track_[i - 1].first, lon0),
                                toScreen(ground_track_[i].first, lon1), col, 1.5f);
                }

                if (rho > 0.0)
                {
                    const Vec3 sub_eci = pos * (1.0 / r);
                    const Vec3 ref_ = (std::abs(sub_eci.z) > 0.99) ? Vec3{1, 0, 0} : Vec3{0, 0, 1};
                    const Vec3 t1 = sub_eci.cross(ref_).normalized();
                    const Vec3 t2 = sub_eci.cross(t1);
                    const double cr = std::cos(rho), sr = std::sin(rho);
                    constexpr int N = 72;
                    ImVec2 prev_sp{};
                    float prev_slon = 0.0f;
                    bool has_prev = false;
                    for (int i = 0; i <= N; ++i)
                    {
                        const double theta = Constants::TWO_PI * i / N;
                        const Vec3 pt = sub_eci * cr + t1 * (std::cos(theta) * sr) + t2 * (std::sin(theta) * sr);
                        const double px_e = pt.x * cx + pt.y * sx;
                        const double py_e = -pt.x * sx + pt.y * cx;
                        const float slat = static_cast<float>(
                            std::asin(std::clamp(pt.z, -1.0, 1.0)) * Constants::RAD2DEG);
                        const float slon = static_cast<float>(
                            std::atan2(py_e, px_e) * Constants::RAD2DEG);
                        const ImVec2 sp = toScreen(slat, slon);
                        if (has_prev && std::abs(slon - prev_slon) < 90.0f)
                            dl->AddLine(prev_sp, sp, IM_COL32(0, 210, 255, 160), 1.2f);
                        prev_sp = sp;
                        prev_slon = slon;
                        has_prev = true;
                    }
                }

                const ImVec2 sat_px = toScreen(lat_deg, lon_deg);
                dl->AddCircleFilled(sat_px, 5.5f, IM_COL32(0, 220, 255, 255));
                dl->AddCircle(sat_px, 7.5f, IM_COL32(255, 255, 255, 200), 16, 1.2f);

                char coord_buf[64];
                std::snprintf(coord_buf, sizeof(coord_buf), "%.2f %s  %.2f %s",
                              std::abs(lat_deg), lat_deg >= 0 ? "N" : "S",
                              std::abs(lon_deg), lon_deg >= 0 ? "E" : "W");
                dl->AddText({cp.x + 4.0f, cp.y + cs.y - 16.0f},
                            IM_COL32(220, 220, 220, 220), coord_buf);
            }
            ImGui::EndChild();
        }

        // Simulation config
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::CollapsingHeader("SIMULATION CONFIG"))
        {
            float speed_f = static_cast<float>(pb_.speed);
            ImGui::SetNextItemWidth(-1.0f);
            if (ImGui::SliderFloat("Speed##cfg", &speed_f, 1.0f, 1000.0f, "%.0fx"))
                pb_.speed = speed_f;

            ImGui::SetNextItemWidth(-1.0f);
            if (ImGui::SliderFloat("Min Elev##cfg", &min_elev_deg_ui_, 0.0f, 45.0f, "%.1f deg"))
            {
                min_elevation_rad_ = static_cast<double>(min_elev_deg_ui_) * Constants::DEG2RAD;
                min_elev_sin_ = std::sin(static_cast<float>(min_elevation_rad_));
            }

            ImGui::Spacing();

            ImGui::Columns(2, "cfg_cols", false);
            ImGui::SetColumnWidth(0, col0);

            ImGui::TextDisabled("Frame dt");
            ImGui::NextColumn();
            if (sim_dt_s_ > 0.0f)
                ImGui::Text("%.4f s  (%.1f Hz)", sim_dt_s_, 1.0f / sim_dt_s_);
            else
                ImGui::TextDisabled("--");
            ImGui::NextColumn();

            ImGui::TextDisabled("Config");
            ImGui::NextColumn();
            ImGui::TextDisabled("see JSON");
            ImGui::NextColumn();

            ImGui::Columns(1);
        }
        break;
    }
    } // end switch

    // ── Deselect ──────────────────────────────────────────────────────────
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    if (ImGui::Button("Deselect  [Esc]", {-1.0f, 0.0f}))
        selected_sat_idx_ = -1;

    ImGui::End();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(6);
}

// ---------------------------------------------------------------------------
// ECI axes overlay — bottom-left corner
// ---------------------------------------------------------------------------
void SatelliteRenderer::drawAxesOverlay()
{
    const ImGuiIO &io = ImGui::GetIO();
    ImDrawList *dl = ImGui::GetForegroundDrawList();

    // Origin: 80 px from a corner.
    // In split mode, move to the bottom-right of the 3D viewport so it isn't
    // hidden behind the data panel.
    const float margin = 80.0f;
    const float arm = 50.0f;
    const float ox = (scene_x_win_ > 0)
                         ? io.DisplaySize.x - margin // bottom-right in split mode
                         : margin;                   // bottom-left normally
    const ImVec2 origin(ox, io.DisplaySize.y - margin);

    // Background disc for contrast
    dl->AddCircleFilled(origin, arm * 0.65f, IM_COL32(0, 0, 0, 120), 32);

    // Project an ECI unit vector to 2D screen offsets using the camera basis.
    // camera right  → screen +X
    // camera up     → screen -Y  (screen Y grows downward)
    const glm::vec3 cam_right = gui_.camera.getRight();
    const glm::vec3 cam_up = gui_.camera.getUpVector();

    auto project = [&](glm::vec3 eci_dir) -> ImVec2
    {
        float sx = glm::dot(cam_right, eci_dir) * arm;
        float sy = -glm::dot(cam_up, eci_dir) * arm;
        return ImVec2(origin.x + sx, origin.y + sy);
    };

    // ECI axes: +X = vernal equinox, +Y = completes right-hand frame, +Z = north pole
    const ImVec2 tip_x = project({1.0f, 0.0f, 0.0f});
    const ImVec2 tip_y = project({0.0f, 1.0f, 0.0f});
    const ImVec2 tip_z = project({0.0f, 0.0f, 1.0f});

    const ImU32 col_x = IM_COL32(255, 80, 80, 230);
    const ImU32 col_y = IM_COL32(80, 220, 80, 230);
    const ImU32 col_z = IM_COL32(80, 140, 255, 230);

    // Draw shafts (2 px wide)
    dl->AddLine(origin, tip_x, col_x, 2.0f);
    dl->AddLine(origin, tip_y, col_y, 2.0f);
    dl->AddLine(origin, tip_z, col_z, 2.0f);

    // Arrowhead triangles
    auto arrowhead = [&](ImVec2 base, ImVec2 tip, ImU32 col)
    {
        float dx = tip.x - base.x, dy = tip.y - base.y;
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1.0f)
            return;
        float ux = dx / len, uy = dy / len;
        float px = -uy * 4.0f, py = ux * 4.0f;
        dl->AddTriangleFilled(
            ImVec2(tip.x, tip.y),
            ImVec2(tip.x - ux * 9.0f + px, tip.y - uy * 9.0f + py),
            ImVec2(tip.x - ux * 9.0f - px, tip.y - uy * 9.0f - py),
            col);
    };
    arrowhead(origin, tip_x, col_x);
    arrowhead(origin, tip_y, col_y);
    arrowhead(origin, tip_z, col_z);

    // Labels (offset slightly past the tip)
    auto labelOffset = [&](ImVec2 orig_pt, ImVec2 tip_pt, float offset) -> ImVec2
    {
        float dx = tip_pt.x - orig_pt.x, dy = tip_pt.y - orig_pt.y;
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1.0f)
            return tip_pt;
        return ImVec2(tip_pt.x + dx / len * offset, tip_pt.y + dy / len * offset);
    };

    const ImVec2 lx = labelOffset(origin, tip_x, 12.0f);
    const ImVec2 ly = labelOffset(origin, tip_y, 12.0f);
    const ImVec2 lz = labelOffset(origin, tip_z, 12.0f);

    dl->AddText(ImVec2(lx.x - 4, lx.y - 6), col_x, "X");
    dl->AddText(ImVec2(ly.x - 4, ly.y - 6), col_y, "Y");
    dl->AddText(ImVec2(lz.x - 4, lz.y - 6), col_z, "Z");

    // Small "ECI" label below the disc
    dl->AddText(ImVec2(origin.x - 10, origin.y + arm * 0.65f + 4), IM_COL32(200, 200, 200, 180), "ECI");
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void SatelliteRenderer::run()
{
    while (!gui_.shouldClose())
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        advancePlayback();
        handleInput();
        buildInterpState();
        applyTracking();

        // ── Split-screen geometry ─────────────────────────────────────────
        // When a satellite is selected: left SPLIT_FRAC = data panel (ImGui),
        // right (1-SPLIT_FRAC) = 3D scene rendered into a restricted viewport.
        const bool sat_view = (selected_sat_idx_ >= 0 &&
                               selected_sat_idx_ < (int)interp_pos_.size());

        const int fw = gui_.getFramebufferWidth();
        const int fh = gui_.getFramebufferHeight();
        const int ww = gui_.getWindowWidth();

        int scene_x_fb = 0, scene_w_fb = fw;
        if (sat_view)
        {
            const float dpi = (float)fw / ww;
            const int pw_win = static_cast<int>(ww * SPLIT_FRAC);
            scene_x_win_ = pw_win;
            scene_x_fb = static_cast<int>(pw_win * dpi);
            scene_w_fb = fw - scene_x_fb;
            // Use right-half aspect so the projection is correct for that viewport
            gui_.setAspectOverride((float)scene_w_fb / fh);
        }
        else
        {
            scene_x_win_ = 0;
            gui_.clearAspectOverride();
        }

        // ── 3D scene ──────────────────────────────────────────────────────
        // beginFrame clears the full buffer and uploads the (possibly overridden)
        // projection matrix. We then restrict the viewport to the right half.
        gui_.beginFrame();

        if (sat_view)
            glViewport(scene_x_fb, 0, scene_w_fb, fh);

        drawStarBackground();
        drawEarth();
        drawCoverageFootprint();
        drawGroundTargets();

        if (!interp_pos_.empty())
        {
            drawSunIndicator();
            drawMoonIndicator();
            drawTrails();
            drawSatellites();
            drawGroundLinks();
        }

        // Restore full viewport before ImGui renders
        if (sat_view)
            glViewport(0, 0, fw, fh);

        // ── ImGui overlays ────────────────────────────────────────────────
        if (!sat_view)
            drawHUD();        // HUD only shown when no satellite selected
        drawSatellitePanel(); // combined left panel (no-op when none selected)
        drawAxesOverlay();    // ECI axes widget, repositioned in split mode

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        updateWindowTitle();
        gui_.endFrame();
    }
}

#endif // CONSTELLATION_VIZ_ENABLED
