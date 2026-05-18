#include <chrono>
#include <cmath>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <ctime>

#include "dji_flight_controller.h"
#include "dji_fc_subscription.h"
#include "dji_widget.h"
#include "dji_error.h"
#include "utils/util_misc.h"
#include "application.hpp"
#include <dji_logger.h>

// ---- Tunable constants -------------------------------------------------------
static constexpr double EARTH_RADIUS_M               = 6371000.0;
static constexpr double LANDING_DISTANCE_THRESHOLD_M = 2.0;   // meters
static constexpr int    MAX_RETRY                    = 3;
static constexpr int    MAX_ROUND                    = 3;
static constexpr int    LANDING_WAIT_TIMEOUT_S       = 600;   // 10 min safety cap
static constexpr int    LOOP_PERIOD_MS               = 200;   // 5 Hz main loop
static constexpr int    WIDGET_DIR_PATH_LEN_MAX      = 256;

// ---- Shared flags (SDK callback thread ↔ main thread) -----------------------
static std::atomic<bool> s_rth_widget_trigger{false};
static std::atomic<bool> s_shutdown{false};

static void OnShutdownSignal(int) { s_shutdown.store(true); }

// ---- Widget callbacks --------------------------------------------------------
static T_DjiReturnCode RthWidget_SetValue(
        E_DjiWidgetType widgetType, uint32_t index, int32_t value, void *userData) {
    (void)widgetType; (void)userData;
    if (index == 0 && value == DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
        USER_LOG_INFO("RTH widget triggered.");
        s_rth_widget_trigger.store(true);
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode RthWidget_GetValue(
        E_DjiWidgetType widgetType, uint32_t index, int32_t *value, void *userData) {
    (void)widgetType; (void)index; (void)userData;
    *value = 0;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static const T_DjiWidgetHandlerListItem s_rthWidgetList[] = {
    {0, DJI_WIDGET_TYPE_BUTTON, RthWidget_SetValue, RthWidget_GetValue, nullptr},
    {1, DJI_WIDGET_TYPE_BUTTON, RthWidget_SetValue, RthWidget_GetValue, nullptr},
};
static const uint32_t s_widgetListCount =
    sizeof(s_rthWidgetList) / sizeof(T_DjiWidgetHandlerListItem);

// ---- Init / DeInit -----------------------------------------------------------
static T_DjiReturnCode DjiTest_FlightControlInit() {
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    if (!osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    T_DjiFlightControllerRidInfo ridInfo = {0};
    ridInfo.latitude  = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude  = 10;

    T_DjiReturnCode rc = DjiFlightController_Init(ridInfo);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight controller failed, error code:0x%08llX", rc);
        return rc;
    }

    rc = DjiFcSubscription_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init data subscription failed, error code:0x%08llX", rc);
        return rc;
    }

    static const E_DjiFcSubscriptionTopic kTopics[] = {
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
        DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
        DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
        DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
    };
    for (auto topic : kTopics) {
        rc = DjiFcSubscription_SubscribeTopic(topic, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, nullptr);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic 0x%X failed, error code:0x%08llX", topic, rc);
            return rc;
        }
    }

    osalHandler->TaskSleepMs(2000);

    rc = DjiWidget_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Widget init failed, error code:0x%08llX", rc);
        return rc;
    }

    char curFileDirPath[WIDGET_DIR_PATH_LEN_MAX];
    rc = DjiUserUtil_GetCurrentFileDirPath(__FILE__, WIDGET_DIR_PATH_LEN_MAX, curFileDirPath);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get file current path error, stat = 0x%08llX", rc);
        return rc;
    }

    char widgetPath[WIDGET_DIR_PATH_LEN_MAX];
    snprintf(widgetPath, sizeof(widgetPath),
             "%swidget/widget_file/en_big_screen", curFileDirPath);
    USER_LOG_INFO("widget file: %s", widgetPath);

    rc = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH,
                                        DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                        widgetPath);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Widget RegUiConfig failed, error code:0x%08llX", rc);
        return rc;
    }

    rc = DjiWidget_RegHandlerList(s_rthWidgetList, s_widgetListCount);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Widget RegHandlerList failed, error code:0x%08llX", rc);
        return rc;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTest_FlightControlDeInit() {
    static const E_DjiFcSubscriptionTopic kTopics[] = {
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
        DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
        DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
        DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
    };

    // Continue on error so every resource is released before returning.
    T_DjiReturnCode result = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    for (auto topic : kTopics) {
        T_DjiReturnCode rc = DjiFcSubscription_UnSubscribeTopic(topic);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Unsubscribe topic 0x%X failed, error code:0x%08llX", topic, rc);
            result = rc;
        }
    }

    T_DjiReturnCode rc = DjiFlightController_DeInit();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit flight controller failed, error code:0x%08llX", rc);
        result = rc;
    }

    rc = DjiFcSubscription_DeInit();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit data subscription failed, error code:0x%08llX", rc);
        result = rc;
    }

    return result;
}

// ---- Geometry ----------------------------------------------------------------
// Haversine great-circle distance (meters). All inputs in radians.
// PSDK mixes units: HOME_POINT_INFO is already rad, GPS_POSITION is deg×1e-7.
static double haversineDistanceRad(double lat1, double lon1, double lat2, double lon2) {
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = std::pow(std::sin(dLat / 2.0), 2) +
               std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dLon / 2.0), 2);
    return EARTH_RADIUS_M * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

// ---- Target location source --------------------------------------------------
// Reads the latest target from a shared file written by target_location_publisher.py.
// File format (one line): lat_deg,lon_deg,unix_timestamp
// Returns false if the file is missing, malformed, or older than MAX_STALE_SEC.
static constexpr const char *TARGET_FILE    = "/tmp/rth_target.txt";
static constexpr double      MAX_STALE_SEC  = 5.0;

static bool GetTargetLocation(T_DjiFlightControllerHomeLocation &out) {
    FILE *f = fopen(TARGET_FILE, "r");
    if (!f) {
        USER_LOG_ERROR("Target file not found: %s", TARGET_FILE);
        return false;
    }

    double lat_deg, lon_deg, ts;
    int n = fscanf(f, "%lf,%lf,%lf", &lat_deg, &lon_deg, &ts);
    fclose(f);

    if (n != 3) {
        USER_LOG_ERROR("Target file parse error (expected 3 fields, got %d).", n);
        return false;
    }

    double age_sec = difftime(time(nullptr), static_cast<time_t>(ts));
    if (age_sec > MAX_STALE_SEC) {
        USER_LOG_WARN("Target data stale (%.1f s old, max %.0f s). Is publisher running?",
                      age_sec, MAX_STALE_SEC);
        return false;
    }

    out.latitude  = lat_deg * DJI_PI / 180.0;
    out.longitude = lon_deg * DJI_PI / 180.0;
    return true;
}

// ---- State machine -----------------------------------------------------------
enum RthState {
    STATE_WAIT_TRIGGER,    // idle — waiting for operator button press
    STATE_GET_LOCATION,    // fetch target landing coordinates
    STATE_SET_HOME,        // push new home GPS coords to FC
    STATE_SET_ALTITUDE,    // push go-home altitude to FC
    STATE_START_GO_HOME,   // command FC to begin RTH
    STATE_MONITOR_RTH,     // watch RTH progress; on AUTO_LANDING, check distance
    STATE_WAIT_LANDING,    // wait for motors to stop after touchdown
    STATE_DONE,            // success — resets to WAIT_TRIGGER
    STATE_ABORT_PILOT,     // pilot took control or drone already in RTH — resets
    STATE_ABORT_ERROR,     // unrecoverable SDK error — exits the process
};

static const char* stateName(RthState s) {
    switch (s) {
        case STATE_WAIT_TRIGGER:  return "WAIT_TRIGGER";
        case STATE_GET_LOCATION:  return "GET_LOCATION";
        case STATE_SET_HOME:      return "SET_HOME";
        case STATE_SET_ALTITUDE:  return "SET_ALTITUDE";
        case STATE_START_GO_HOME: return "START_GO_HOME";
        case STATE_MONITOR_RTH:   return "MONITOR_RTH";
        case STATE_WAIT_LANDING:  return "WAIT_LANDING";
        case STATE_DONE:          return "DONE";
        case STATE_ABORT_PILOT:   return "ABORT_PILOT";
        case STATE_ABORT_ERROR:   return "ABORT_ERROR";
    }
    return "?";
}

// True when display_mode means a human pilot is flying the drone (P/A/M).
static bool isPilotControlMode(uint8_t mode) {
    return mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_MANUAL_CTRL ||
           mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE    ||
           mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS;
}

// True in states where reverting to P/A/M mode means the pilot took over.
// Guard is inactive during setup states (SET_HOME, SET_ALTITUDE, START_GO_HOME)
// because FC stays in P_GPS until the RTH command is acknowledged — checking
// there would produce false aborts due to FC command latency (~0.5–2 s).
static bool isActiveRthState(RthState s) {
    return s == STATE_MONITOR_RTH || s == STATE_WAIT_LANDING;
}

// ---- Entry point -------------------------------------------------------------
int main(int argc, char** argv) {
    signal(SIGINT,  OnShutdownSignal);
    signal(SIGTERM, OnShutdownSignal);

    USER_LOG_INFO("Starting RTH state machine.");

    Application application(argc, argv);

    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    if (!osalHandler) {
        USER_LOG_ERROR("Failed to get OSAL handler.");
        return -1;
    }

    USER_LOG_INFO("Init flight control and data subscription.");
    if (DjiTest_FlightControlInit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to init flight controller.");
        return -1;
    }
    osalHandler->TaskSleepMs(100);

    // ---- Telemetry buffers ----
    T_DjiFcSubscriptionHomePointInfo home_point_info = {0};
    T_DjiFcSubscriptionGpsPosition   gpsPosition     = {0};
    T_DjiFcSubscriptionDisplaymode   display_mode    = 0;
    T_DjiFcSubscriptionFlightStatus  flight_status   = 0;
    T_DjiDataTimestamp                timestamp       = {0};

    // ---- Mission parameters ----
    T_DjiFlightControllerHomeLocation   location        = {0};
    E_DjiFlightControllerGoHomeAltitude return_altitude =
        static_cast<E_DjiFlightControllerGoHomeAltitude>(30);

    // ---- State machine variables ----
    RthState state      = STATE_WAIT_TRIGGER;
    RthState prev_state = STATE_ABORT_ERROR;  // sentinel: forces first-entry log

    int  round             = 0;
    int  retry_set_home    = 0;
    int  retry_set_alt     = 0;
    bool confirmed_go_home = false;  // true once FC reports NAVI_GO_HOME; pilot guard only fires after this

    using Clock = std::chrono::steady_clock;
    auto next_set_home_attempt = Clock::time_point::min();
    auto next_set_alt_attempt  = Clock::time_point::min();
    auto landing_deadline      = Clock::time_point::max();

    // Reset all per-round state and loop back to WAIT_TRIGGER.
    auto resetForNextRound = [&]() {
        round             = 0;
        retry_set_home    = 0;
        retry_set_alt     = 0;
        confirmed_go_home = false;
        next_set_home_attempt = Clock::time_point::min();
        next_set_alt_attempt  = Clock::time_point::min();
        landing_deadline      = Clock::time_point::max();
        s_rth_widget_trigger.store(false);  // discard presses that arrived during execution
        state = STATE_WAIT_TRIGGER;
    };

    while (state != STATE_ABORT_ERROR && !s_shutdown.load()) {
        if (state != prev_state) {
            USER_LOG_INFO("State: %s → %s", stateName(prev_state), stateName(state));
            prev_state = state;
        }

        // Read telemetry every tick.
        DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
            reinterpret_cast<uint8_t*>(&display_mode), sizeof(display_mode), &timestamp);
        DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
            reinterpret_cast<uint8_t*>(&home_point_info), sizeof(home_point_info), &timestamp);
        DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
            reinterpret_cast<uint8_t*>(&gpsPosition), sizeof(gpsPosition), &timestamp);
        DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
            reinterpret_cast<uint8_t*>(&flight_status), sizeof(flight_status), &timestamp);

        // Global pilot-takeover guard (highest priority).
        // The confirmed_go_home flag prevents false positives while FC is still
        // transitioning to NAVI_GO_HOME after StartGoHome() returns SUCCESS.
        if (isActiveRthState(state) && confirmed_go_home && isPilotControlMode(display_mode)) {
            USER_LOG_WARN("Pilot took control mid-RTH (mode=%d). Aborting.", display_mode);
            state = STATE_ABORT_PILOT;
            continue;
        }

        switch (state) {
            case STATE_WAIT_TRIGGER: {
                if (s_rth_widget_trigger.exchange(false)) {
                    USER_LOG_INFO("Widget button pressed. Starting RTH workflow.");
                    state = STATE_GET_LOCATION;
                } else if (display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME ||
                           display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING) {
                    USER_LOG_WARN("Drone already in RTH/AUTO_LANDING (mode=%d). Waiting.", display_mode);
                    state = STATE_ABORT_PILOT;
                } else {
                    USER_LOG_INFO("Waiting for widget trigger...");
                }
                break;
            }

            case STATE_GET_LOCATION: {
                if (!GetTargetLocation(location)) {
                    USER_LOG_ERROR("Failed to get target location. Aborting.");
                    state = STATE_ABORT_ERROR;
                    break;
                }
                USER_LOG_INFO("Target: lat=%.6f rad, lon=%.6f rad",
                              location.latitude, location.longitude);
                state = STATE_SET_HOME;
                break;
            }

            case STATE_SET_HOME: {
                if (round >= MAX_ROUND) {
                    USER_LOG_ERROR("Exceeded MAX_ROUND=%d. Giving up.", MAX_ROUND);
                    state = STATE_DONE;
                    break;
                }
                if (Clock::now() < next_set_home_attempt) break;

                USER_LOG_INFO("Round %d/%d: setting home location...", round + 1, MAX_ROUND);
                T_DjiReturnCode rc = DjiFlightController_SetHomeLocationUsingGPSCoordinates(location);
                if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    retry_set_home = 0;
                    state = STATE_SET_ALTITUDE;
                } else {
                    retry_set_home++;
                    USER_LOG_ERROR("SetHome failed (0x%08llX). Retry %d/%d",
                                   rc, retry_set_home, MAX_RETRY);
                    if (retry_set_home > MAX_RETRY) {
                        state = STATE_ABORT_ERROR;
                    } else {
                        next_set_home_attempt = Clock::now() + std::chrono::seconds(1);
                    }
                }
                break;
            }

            case STATE_SET_ALTITUDE: {
                if (Clock::now() < next_set_alt_attempt) break;

                USER_LOG_INFO("Setting go-home altitude...");
                T_DjiReturnCode rc = DjiFlightController_SetGoHomeAltitude(return_altitude);
                if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    retry_set_alt = 0;
                    state = STATE_START_GO_HOME;
                } else {
                    retry_set_alt++;
                    USER_LOG_ERROR("SetAltitude failed (0x%08llX). Retry %d/%d",
                                   rc, retry_set_alt, MAX_RETRY);
                    if (retry_set_alt > MAX_RETRY) {
                        state = STATE_ABORT_ERROR;
                    } else {
                        next_set_alt_attempt = Clock::now() + std::chrono::seconds(1);
                    }
                }
                break;
            }

            case STATE_START_GO_HOME: {
                USER_LOG_INFO("Commanding drone to go home...");
                T_DjiReturnCode rc = DjiFlightController_StartGoHome();
                if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    round++;
                    state = STATE_MONITOR_RTH;
                } else {
                    USER_LOG_ERROR("StartGoHome failed (0x%08llX).", rc);
                    state = STATE_ABORT_ERROR;
                }
                break;
            }

            case STATE_MONITOR_RTH: {
                if (display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME)
                    confirmed_go_home = true;

                // Re-fetch current target every tick: landing platform may have moved.
                T_DjiFlightControllerHomeLocation current_target;
                if (!GetTargetLocation(current_target)) {
                    USER_LOG_WARN("Failed to get current target location. Skipping distance check.");
                    break;
                }

                // GPS_POSITION is deg×1e-7; target location is already rad.
                static constexpr double DEG_TO_RAD = M_PI / 180.0;
                double cur_lat = gpsPosition.y * 1e-7 * DEG_TO_RAD;
                double cur_lon = gpsPosition.x * 1e-7 * DEG_TO_RAD;
                double distance = haversineDistanceRad(cur_lat, cur_lon,
                                                       current_target.latitude,
                                                       current_target.longitude);

                if (display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING) {
                    if (distance <= LANDING_DISTANCE_THRESHOLD_M) {
                        USER_LOG_INFO("On target (%.2f m). Letting it land.", distance);
                        landing_deadline = Clock::now() + std::chrono::seconds(LANDING_WAIT_TIMEOUT_S);
                        state = STATE_WAIT_LANDING;
                    } else {
                        USER_LOG_WARN("Off target (%.2f m). Cancelling and retrying.", distance);
                        T_DjiReturnCode rc = DjiFlightController_CancelGoHome();
                        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                            USER_LOG_ERROR("CancelGoHome failed (0x%08llX).", rc);
                            state = STATE_ABORT_ERROR;
                        } else {
                            confirmed_go_home = false;  // FC returns to P_GPS; wait for NAVI_GO_HOME on retry
                            state = STATE_GET_LOCATION;
                        }
                    }
                }
                // else: NAVI_GO_HOME — still en route, keep monitoring.
                break;
            }

            case STATE_WAIT_LANDING: {
                if (flight_status == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED) {
                    USER_LOG_INFO("Landing complete. Motors stopped.");
                    state = STATE_DONE;
                } else if (Clock::now() > landing_deadline) {
                    USER_LOG_WARN("Landing timeout after %d s (status=%d). Exiting.",
                                  LANDING_WAIT_TIMEOUT_S, flight_status);
                    state = STATE_DONE;
                }
                break;
            }

            case STATE_DONE:
                USER_LOG_INFO("RTH complete. Ready for next trigger.");
                resetForNextRound();
                break;

            case STATE_ABORT_PILOT:
                USER_LOG_WARN("RTH aborted (pilot control). Ready for next trigger.");
                resetForNextRound();
                break;

            case STATE_ABORT_ERROR:
                break;  // exits the while loop
        }

        osalHandler->TaskSleepMs(LOOP_PERIOD_MS);
    }

    if (s_shutdown.load()) {
        USER_LOG_INFO("Shutdown signal received. Cleaning up.");
    } else {
        USER_LOG_ERROR("RTH aborted: unrecoverable error.");
    }

    DjiTest_FlightControlDeInit();
    return s_shutdown.load() ? 0 : -1;
}
