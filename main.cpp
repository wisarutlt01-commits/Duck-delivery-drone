#include <iostream>
#include <chrono>
#include <memory>
#include <thread>
#include <cmath>

#include "dji_flight_controller.h"
#include "dji_fc_subscription.h"
#include "dji_error.h"

#include <liveview/test_liveview_entry.hpp>
#include <perception/test_perception_entry.hpp>
#include <perception/test_lidar_entry.hpp>
#include <perception/test_radar_entry.hpp>
#include <flight_control/test_flight_control.h>
#include <gimbal/test_gimbal_entry.hpp>
#include "application.hpp"
#include "fc_subscription/test_fc_subscription.h"
#include <gimbal_emu/test_payload_gimbal_emu.h>
#include <camera_emu/test_payload_cam_emu_media.h>
#include <camera_emu/test_payload_cam_emu_base.h>
#include <dji_logger.h>
#include "widget/test_widget.h"
#include "widget/test_widget_speaker.h"
#include <power_management/test_power_management.h>
#include "data_transmission/test_data_transmission.h"
#include <flight_controller/test_flight_controller_entry.h>
#include <positioning/test_positioning.h>
#include <hms_manager/hms_manager_entry.h>
#include "camera_manager/test_camera_manager_entry.h"
#include <widget_manager/test_widget_manager.hpp>
#include <flight_control/test_flight_control.h>

const double EARTH_RADIUS_M = 6371000.0;
const double LANDING_DISTANCE_THRESHOLD_M = 2.0;
const int MAX_RETRY = 3;
const int MAX_ROUND = 3;

// RC mode threshold: Flycart 30 RC reports `mode` as a discrete switch position
// (e.g. P/A/S/F mapped to large integer values). <= -8000 corresponds to the
// upper switch position used as the "start RTH-to-new-home" trigger.
const int16_t RC_TRIGGER_MODE_THRESHOLD = -8000;

// Safety bound: stop monitoring landing after this duration so the script
// never hangs in the field if telemetry never reports motors-stopped.
const int LANDING_WAIT_TIMEOUT_S = 600;   // 10 min wait for engine-off after touchdown

T_DjiReturnCode DjiTest_FlightControlInit(void) {
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *s_osalHandler = NULL;
    T_DjiFlightControllerRidInfo ridInfo = {0};

    s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    //Edit this for home location of drone 
    ridInfo.latitude = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude = 10;

    returnCode = DjiFlightController_Init(ridInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight controller module failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init data subscription module failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    /*! subscribe fc data */
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic flight status failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic display mode failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic avoid data failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps position failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude fused failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic RC failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic home point info failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FlightControlDeInit(void)
{
    T_DjiReturnCode returnCode;

    returnCode = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Unsubscribe topic flight status failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Unsubscribe topic display mode failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Unsubscribe topic height fusion failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Unsubscribe topic gps position failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Unsubscribe topic altitude fused failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Unsubscribe topic home point info failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Unsubscribe topic rc failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFlightController_DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                       returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit data subscription module failed, error code:0x%08llX",
                       returnCode);
        return returnCode;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

// Haversine great-circle distance, returns meters.
// All four inputs MUST be in RADIANS — the natural unit of the formula.
// Caller is responsible for normalizing inputs (PSDK telemetry is mixed:
// HOME_POINT_INFO is rad, GPS_POSITION is deg*1e-7).
double haversineDistanceRad(double lat1_rad, double lon1_rad,
                            double lat2_rad, double lon2_rad) {
    double dLat = lat2_rad - lat1_rad;
    double dLon = lon2_rad - lon1_rad;

    // a = sin²(dLat/2) + cos(lat1) * cos(lat2) * sin²(dLon/2)
    double a = std::pow(std::sin(dLat / 2.0), 2) +
               std::cos(lat1_rad) * std::cos(lat2_rad) *
               std::pow(std::sin(dLon / 2.0), 2);

    // c = 2 * atan2(sqrt(a), sqrt(1-a))
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    // d = R * c
    return EARTH_RADIUS_M * c;
}

// State machine states for the auto-RTH workflow.
enum RthState {
    STATE_WAIT_TRIGGER,    // wait for operator RC switch trigger
    STATE_SET_HOME,        // push new home GPS coords to FC
    STATE_SET_ALTITUDE,    // push go-home altitude to FC
    STATE_START_GO_HOME,   // command FC to begin RTH
    STATE_MONITOR_RTH,     // watch RTH progress; on landing, check distance
    STATE_WAIT_LANDING,    // wait for motors to stop after touchdown
    STATE_DONE,            // success terminal state
    STATE_ABORT_PILOT,     // pilot took control (P/A/M mode) — exit
    STATE_ABORT_ERROR,     // unrecoverable error — exit
};

const char* stateName(RthState s) {
    switch (s) {
        case STATE_WAIT_TRIGGER:  return "WAIT_TRIGGER";
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
// If we see this DURING RTH execution, FC has dropped our control → bail out.
bool isPilotControlMode(uint8_t mode) {
    return mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_MANUAL_CTRL ||
           mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE ||
           mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS;
}

// States in which a switch to P/A/M mode means "pilot took over → abort".
// WAIT_TRIGGER is excluded because we expect P-mode there (pre-trigger normal).
bool isActiveRthState(RthState s) {
    return s == STATE_SET_HOME || s == STATE_SET_ALTITUDE ||
           s == STATE_START_GO_HOME || s == STATE_MONITOR_RTH ||
           s == STATE_WAIT_LANDING;
}

int main(int argc, char** argv) {
    USER_LOG_INFO("Initial Dynamic landing logic.");

    //Init PSDK and set up environment for flight controller sample
    Application application(argc, argv);
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFcSubscriptionRC rc_status = {0};
    T_DjiFcSubscriptionHomePointInfo home_point_info = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionDisplaymode display_mode = 0;
    T_DjiFcSubscriptionFlightStatus flight_status = 0;
    T_DjiDataTimestamp timestamp = {0};

    //param
    T_DjiFlightControllerHomeLocation location = {0};
    E_DjiFlightControllerGoHomeAltitude return_altitude =
        static_cast<E_DjiFlightControllerGoHomeAltitude>(30);

    //init PSDK flight controller and subscription module
    USER_LOG_INFO("Init flight control and data subscription.");
    T_DjiReturnCode init_fc = DjiTest_FlightControlInit();
    if (init_fc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to init flight controller, error code: 0x%08X", init_fc);
        std::cerr << "Failed to init flight controller.\n";
        return -1;
    }
    osalHandler->TaskSleepMs(100);

    // NOTE: Joystick authority is NOT required here. All commands used
    // (SetHomeLocation, SetGoHomeAltitude, StartGoHome, CancelGoHome) are
    // high-level FC commands — they do NOT need joystick authority.
    auto cleanup = [&]() {
        USER_LOG_DEBUG("Deinit Flight Control / FC subscription.");
        DjiTest_FlightControlDeInit();
    };

    //TODO: How to get position from other hardware?
    location.latitude  = 12.994319 * DJI_PI / 180;
    location.longitude = 101.443273 * DJI_PI / 180;

    // ===== State machine =====
    RthState state = STATE_WAIT_TRIGGER;
    RthState prev_state = STATE_DONE;  // sentinel != STATE_WAIT_TRIGGER, forces first-entry log
    int round = 0;
    int retry_set_home = 0;
    int retry_set_altitude = 0;

    // Cooldown timestamps so retries don't hammer FC at 5Hz.
    auto next_set_home_attempt = std::chrono::steady_clock::time_point::min();
    auto next_set_altitude_attempt = std::chrono::steady_clock::time_point::min();

    auto landing_deadline = std::chrono::steady_clock::time_point::max();

    while (state != STATE_DONE && state != STATE_ABORT_PILOT && state != STATE_ABORT_ERROR) {
        // Log on every state transition.
        if (state != prev_state) {
            USER_LOG_INFO("State transition: %s → %s", stateName(prev_state), stateName(state));
            prev_state = state;
        }

        // ---- Read telemetry every tick ----
        DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC,
            (uint8_t *) &rc_status, sizeof(rc_status), &timestamp);
        DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
            (uint8_t *) &display_mode, sizeof(display_mode), &timestamp);
        DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
            (uint8_t *) &home_point_info, sizeof(home_point_info), &timestamp);
        DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
            (uint8_t *) &gpsPosition, sizeof(gpsPosition), &timestamp);
        DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
            (uint8_t *) &flight_status, sizeof(flight_status), &timestamp);

        // ---- Global pilot-takeover guard (highest priority) ----
        // If the FC reports pilot control while we're in any active RTH state,
        // exit immediately so we never fight the human on the sticks.
        if (isActiveRthState(state) && isPilotControlMode(display_mode)) {
            USER_LOG_WARN("Pilot took control mid-RTH (display_mode=%d). Aborting.", display_mode);
            state = STATE_ABORT_PILOT;
            continue;
        }

        // ---- Per-state action ----
        switch (state) {
            case STATE_WAIT_TRIGGER: {
                if (rc_status.mode <= RC_TRIGGER_MODE_THRESHOLD) {
                    USER_LOG_INFO("Trigger received from RC. Starting RTH workflow.");
                    state = STATE_SET_HOME;
                } else if (display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME ||
                           display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING) {
                    USER_LOG_WARN("Drone already in RTH/AUTO_LANDING (display_mode=%d) before trigger. Aborting.",
                                  display_mode);
                    state = STATE_ABORT_PILOT;
                } else {
                    USER_LOG_INFO("Waiting for trigger, current rc mode=%d", rc_status.mode);
                }
                break;
            }

            case STATE_SET_HOME: {
                if (round >= MAX_ROUND) {
                    USER_LOG_ERROR("Exceeded MAX_ROUND=%d. Aborting.", MAX_ROUND);
                    state = STATE_ABORT_ERROR;
                    break;
                }
                if (std::chrono::steady_clock::now() < next_set_home_attempt) break;

                USER_LOG_INFO("Round %d: setting new home location...", round + 1);
                T_DjiReturnCode rc = DjiFlightController_SetHomeLocationUsingGPSCoordinates(location);
                if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    retry_set_home = 0;
                    state = STATE_SET_ALTITUDE;
                } else {
                    retry_set_home++;
                    USER_LOG_ERROR("SetHome failed (0x%08llX). Retry %d/%d", rc, retry_set_home, MAX_RETRY);
                    if (retry_set_home > MAX_RETRY) {
                        state = STATE_ABORT_ERROR;
                    } else {
                        next_set_home_attempt = std::chrono::steady_clock::now() + std::chrono::seconds(1);
                    }
                }
                break;
            }

            case STATE_SET_ALTITUDE: {
                if (std::chrono::steady_clock::now() < next_set_altitude_attempt) break;

                USER_LOG_INFO("Setting go-home altitude...");
                T_DjiReturnCode rc = DjiFlightController_SetGoHomeAltitude(return_altitude);
                if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    retry_set_altitude = 0;
                    state = STATE_START_GO_HOME;
                } else {
                    retry_set_altitude++;
                    USER_LOG_ERROR("SetAltitude failed (0x%08llX). Retry %d/%d",
                                   rc, retry_set_altitude, MAX_RETRY);
                    if (retry_set_altitude > MAX_RETRY) {
                        state = STATE_ABORT_ERROR;
                    } else {
                        next_set_altitude_attempt = std::chrono::steady_clock::now() + std::chrono::seconds(1);
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
                // Compute distance: GPS (deg*1e-7) and home (rad) → both to rad.
                const double DEG_TO_RAD = M_PI / 180.0;
                double cur_lat_rad = gpsPosition.y * 1e-7 * DEG_TO_RAD;
                double cur_lon_rad = gpsPosition.x * 1e-7 * DEG_TO_RAD;
                double distance = haversineDistanceRad(cur_lat_rad, cur_lon_rad,
                                                       home_point_info.latitude,
                                                       home_point_info.longitude);

                if (display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING) {
                    if (distance <= LANDING_DISTANCE_THRESHOLD_M) {
                        USER_LOG_INFO("Reached home (distance=%.2f m). Letting it land.", distance);
                        landing_deadline = std::chrono::steady_clock::now() +
                                           std::chrono::seconds(LANDING_WAIT_TIMEOUT_S);
                        state = STATE_WAIT_LANDING;
                    } else {
                        USER_LOG_WARN("Landing far from home (distance=%.2f m). Cancel + retry.", distance);
                        T_DjiReturnCode rc = DjiFlightController_CancelGoHome();
                        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                            USER_LOG_ERROR("CancelGoHome failed (0x%08llX).", rc);
                            state = STATE_ABORT_ERROR;
                        } else {
                            state = STATE_SET_HOME;  // round-bound check happens in SET_HOME
                        }
                    }
                }
                // else: still climbing/cruising in NAVI_GO_HOME — keep monitoring.
                break;
            }

            case STATE_WAIT_LANDING: {
                if (flight_status == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED) {
                    USER_LOG_INFO("Landing complete, motors stopped.");
                    state = STATE_DONE;
                } else if (std::chrono::steady_clock::now() > landing_deadline) {
                    USER_LOG_WARN("Landing wait timed out after %d s (flight_status=%d). Exiting.",
                                  LANDING_WAIT_TIMEOUT_S, flight_status);
                    state = STATE_DONE;
                }
                break;
            }

            case STATE_DONE:
            case STATE_ABORT_PILOT:
            case STATE_ABORT_ERROR:
                break; // unreachable inside loop
        }

        osalHandler->TaskSleepMs(1000/5); //5 hz
    }

    // ---- Terminal handling ----
    int exit_code = 0;
    switch (state) {
        case STATE_DONE:
            USER_LOG_INFO("RTH workflow completed successfully.");
            break;
        case STATE_ABORT_PILOT:
            USER_LOG_WARN("RTH aborted: pilot/FC took control.");
            exit_code = -1;
            break;
        case STATE_ABORT_ERROR:
            USER_LOG_ERROR("RTH aborted: unrecoverable error.");
            exit_code = -1;
            break;
        default:
            exit_code = -1;
            break;
    }

    cleanup();
    return exit_code;
}
