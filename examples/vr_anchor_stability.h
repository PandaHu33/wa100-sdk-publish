#pragma once
#include <algorithm>
#include <cmath>

// Master-side geometric fusion state. Never filters actuator/canonical targets.
struct VrAnchorStability {
    double enter_sec = 0.08, release_sec = 0.18, switch_sec = 0.16;
    double release_scale = 1.8, still_cutoff_hz = 0.65;
    double anchor_radius_scale = 0.40;
    double contact_margin_m = 0.012;
    // XR positions are meters; Hi5 FK anchor coordinates have a different
    // native scale and must not define XR contact distances.
    double xr_contact_enter_m = 0.025;
    double fast_release_scale = 3.0, fast_release_sec = 0.06;
    double hold_weight = 1.0;
    double feature_time = -1.0, thumb_features[2] = {0.0, 0.0};
    int active = -1, pending = -2;
    double pending_since = 0, fast_release_since = -1, last_time = -1, ratio = 0, weight = 0;
    void reset() { active = -1; pending = -2; fast_release_since = -1; last_time = -1; feature_time = -1; }
    static double alpha(double hz, double dt) {
        return dt / (dt + 1.0 / (6.283185307179586 * hz));
    }
    void filter_thumb_features(double& pitch, double& yaw, double now) {
        // Dimensionless geometric flexion/opposition BEFORE RatioToTarget and
        // calibrated anchor construction, never canonical/actuator smoothing.
        double input[2] = {pitch, yaw};
        if (feature_time < 0 || now < feature_time || now - feature_time > 0.25) {
            thumb_features[0] = pitch; thumb_features[1] = yaw; feature_time = now;
            return;
        }
        if (now > feature_time) {
            const double dt = std::min(0.05, now - feature_time);
            for (int i = 0; i < 2; ++i) {
                const double error = input[i] - thumb_features[i];
                const double magnitude = std::abs(error);
                const double deadband = i == 0 ? 0.025 : 0.035;
                const double remaining = std::max(0.0, magnitude - deadband);
                const double hz = std::min(3.0, 0.45 + 8.0 * remaining);
                // Strong rest rejection; sustained deliberate movement can
                // traverse the deadband without a permanently latched thumb.
                const double step = alpha(hz, dt) * remaining;
                thumb_features[i] += std::copysign(std::min(step, 1.5 * dt), error);
            }
            feature_time = now;
        }
        pitch = thumb_features[0]; yaw = thumb_features[1];
    }
    void update(double raw_ratio, double raw_weight, double index_distance,
                double middle_distance, double now,
                double index_contact = -1.0, double middle_contact = -1.0) {
        if (last_time < 0 || now < last_time || now - last_time > 0.25) {
            reset(); ratio = raw_ratio; weight = raw_weight; last_time = now;
            return;
        }
        if (now <= last_time) return;
        const double dt = std::min(0.05, now - last_time);
        last_time = now;
        const double distances[2] = {index_distance, middle_distance};
        // Actual separation of the pinching tips, relative to the calibrated
        // contact gap. Moving the other finger must not release this grip.
        const double contacts[2] = {
            index_contact < 0.0 ? index_distance : index_contact,
            middle_contact < 0.0 ? middle_distance : middle_contact};
        const int nearest = index_distance <= middle_distance ? 0 : 1;
        int candidate = active;
        if (active < 0) candidate = distances[nearest] <= 1.0 && contacts[nearest] <= 1.0 ? nearest : -1;
        else if (nearest != active && contacts[active] > 1.0 &&
                 distances[nearest] <= 0.75 && contacts[nearest] <= 1.0 &&
                 distances[nearest] + 0.35 < distances[active]) candidate = nearest;
        else if (contacts[active] > release_scale) candidate = -1;
        const bool clearly_open = candidate < 0 && active >= 0 && contacts[active] > fast_release_scale;
        if (clearly_open) {
            if (fast_release_since < 0) fast_release_since = now;
        } else fast_release_since = -1;
        if (candidate == active) pending = -2;
        else {
            if (pending != candidate) { pending = candidate; pending_since = now; }
            const double duration = candidate < 0 ? release_sec : (active < 0 ? enter_sec : switch_sec);
            const bool fast_ready = clearly_open && now - fast_release_since + 1e-9 >= fast_release_sec;
            if (now - pending_since + 1e-9 >= duration || fast_ready) {
                active = candidate; pending = -2; fast_release_since = -1;
            }
        }
        const double target_ratio = active < 0 ? raw_ratio : static_cast<double>(active);
        // Tight contact holds the calibrated endpoint; all transitions stay
        // continuous in feature space before the calibrated target is formed.
        const double error = std::abs(target_ratio - ratio);
        const double hz = active >= 0 ? 4.0 : std::max(still_cutoff_hz, std::min(10.0, error * 20.0));
        ratio += alpha(hz, dt) * (target_ratio - ratio);
        // While contact remains latched, do not fade back to noisy geometric
        // thumb estimates. Release confirmation is the only way to remove the
        // anchor constraint. This blending still precedes target construction.
        const double target_weight = active >= 0 ? hold_weight : raw_weight;
        weight += alpha(target_weight < weight ? 10.0 : hz, dt) * (target_weight - weight);
    }
};
