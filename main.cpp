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
// (e.g. P/A/S/F mapped to large integer values). >= 8000 corresponds to the
// upper switch position used as the "start RTH-to-new-home" trigger.
const uint16_t RC_TRIGGER_MODE_THRESHOLD = 8000;

// Safety bound: stop monitoring landing after this duration so the script
// never hangs in the field if telemetry never reports motors-stopped.
const int LANDING_WAIT_TIMEOUT_S = 600;   // 3 min wait for engine-off after touchdown

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

double toRadians(double degree) {
    return degree * (M_PI / 180.0);
}

double calculateHaversineDistance(double lat1, double lon1, double lat2, double lon2) {
    //Find difference and convert to radians in lat and long
    double dLat = toRadians(lat2 - lat1);
    double dLon = toRadians(lon2 - lon1);
    
    //Convert lat to radians for both points
    double rLat1 = toRadians(lat1);
    double rLat2 = toRadians(lat2);

    //Calculate haversine formula
    // a = sin²(dlat/2) + cos(rLat1) ⋅ cos(rLat2) ⋅ sin²(dLon/2)
    double a = std::pow(std::sin(dLat / 2.0), 2) + 
               std::cos(rLat1) * std::cos(rLat2) * std::pow(std::sin(dLon / 2.0), 2);
               
    // c = 2 ⋅ atan2(√a, √(1−a))
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    
    // real_distance (d = R ⋅ c)
    return EARTH_RADIUS_M * c;
}

int main(int argc, char** argv) {
    std::cout << "Initial DJI PSDK...\n";  

    //Init PSDK and set up environment for flight controller sample
    Application application(argc, argv);
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionRC rc_status = {0};
    T_DjiFcSubscriptionHomePointInfo home_point_info = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionDisplaymode display_mode = 0;
    T_DjiDataTimestamp timestamp = {0};

    //param
    T_DjiFlightControllerHomeLocation location = {0};
    E_DjiFlightControllerGoHomeAltitude return_altitude =
        static_cast<E_DjiFlightControllerGoHomeAltitude>(90);
    
    //init PSDK flight controller and subscription module
    USER_LOG_INFO("Init flight control and data subscription.");
    T_DjiReturnCode init_fc = DjiTest_FlightControlInit();
    if (init_fc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to init flight controller, error code: 0x%08X", init_fc);
        std::cerr << "Failed to init flight controller.\n";
        return -1;
    }
    osalHandler->TaskSleepMs(100);

    //obtain joystick/control authority before sending flight commands
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick control authority failed, error code:0x%08llX", returnCode);
        DjiTest_FlightControlDeInit();
        return -1;
    }

    // Cleanup helper: release joystick authority + deinit subscriptions.
    // Call before EVERY exit path after Obtain so authority never stays held
    // by the PSDK app (would block pilot/FC from regaining control).
    auto cleanup = [&]() {
        T_DjiReturnCode rc = DjiFlightController_ReleaseJoystickCtrlAuthority();
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Release joystick control authority failed, error code:0x%08llX", rc);
        }
        DjiTest_FlightControlDeInit();
    };

    //TODO: How to get position from other hardware?
    //get new home location param from beacon and set new home location param
    location.latitude = 10.05213215;
    location.longitude = 102.1351658418;

    // Wait for operator trigger from RC. This script is meant to run AFTER the
    // drone finishes its mission, so we wait indefinitely.
    // Abort early if the drone is already in RTH or AUTO_LANDING — in that case
    // some other system already took control and we must not override it.
    while (true) {
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC,
                                                        (uint8_t *) &rc_status,
                                                        sizeof(T_DjiFcSubscriptionRC),
                                                        &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic rc error.");
        } else if (rc_status.mode >= RC_TRIGGER_MODE_THRESHOLD) {
            USER_LOG_INFO("Get trigger from remote, start to set home location and command drone go home");
            break;
        } else {
            USER_LOG_INFO("Waiting for trigger from remote, current rc mode is %d", rc_status.mode);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                          (uint8_t *) &display_mode,
                                                          sizeof(T_DjiFcSubscriptionDisplaymode),
                                                          &timestamp);
        if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
            (display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME ||
             display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)) {
            USER_LOG_WARN("Drone already in RTH/AUTO_LANDING (display_mode=%d) before trigger received. "
                          "Aborting custom RTH script — pilot/FC has control.", display_mode);
            cleanup();
            return -1;
        }

        osalHandler->TaskSleepMs(1000/5); //5 hz
    }
        
    int round = 0;
    while (round < MAX_ROUND) {
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
                                                            (uint8_t *) &home_point_info,
                                                            sizeof(T_DjiFcSubscriptionHomePointInfo),
                                                            &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic home point info error.");
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                            (uint8_t *) &display_mode,
                                                            sizeof(T_DjiFcSubscriptionDisplaymode),
                                                            &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic display mode error.");
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                        (uint8_t *) &gpsPosition,
                                                        sizeof(T_DjiFcSubscriptionGpsPosition),
                                                        &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps position error.");
        }

        // GPS_POSITION: x = longitude, y = latitude in 1e-7 degrees (T_DjiVector3d int32)
        double cur_lat_deg = gpsPosition.y * 1e-7;
        double cur_lon_deg = gpsPosition.x * 1e-7;
        double distance = calculateHaversineDistance(cur_lat_deg, cur_lon_deg,
                                                     home_point_info.latitude,
                                                     home_point_info.longitude);

        if (display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING) {
            if (distance <= LANDING_DISTANCE_THRESHOLD_M) {
                USER_LOG_INFO("Drone has reached home point, distance to home point is %.2f meters", distance);
                break;
            } else {
                USER_LOG_INFO("Drone is landing, but has not reached home point, distance to home point is %.2f meters", distance);
                T_DjiReturnCode result_command = DjiFlightController_CancelGoHome();
                if (result_command != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Cannot cancel go home command. Pilot should take control!");
                    cleanup();
                    return -1;
                }
            }
        }

        if (display_mode != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME) {
            USER_LOG_INFO("Round %d to set home location and command drone go home", round + 1);

            //reset retry counters at the start of each round
            int retry_set_home = 1;
            int retry_set_home_altitude = 1;

            USER_LOG_INFO("Setting new home location to FC");
            T_DjiReturnCode set_home_result = DjiFlightController_SetHomeLocationUsingGPSCoordinates(location);
            while (set_home_result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Cannot set new home location. Retry {%d}/%d", retry_set_home, MAX_RETRY);
                set_home_result = DjiFlightController_SetHomeLocationUsingGPSCoordinates(location);
                retry_set_home++;

                if (retry_set_home > MAX_RETRY) {
                    USER_LOG_ERROR("Maximum retry set home location. Pilot should take control!");
                    cleanup();
                    return -1;
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));

            //setting go home altitude
            USER_LOG_INFO("Setting go home altitude to FC");
            T_DjiReturnCode set_home_altitude_result = DjiFlightController_SetGoHomeAltitude(return_altitude);
            while (set_home_altitude_result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Cannot set new home altitude. Retry {%d}/%d", retry_set_home_altitude, MAX_RETRY);
                set_home_altitude_result = DjiFlightController_SetGoHomeAltitude(return_altitude);
                retry_set_home_altitude++;

                if (retry_set_home_altitude > MAX_RETRY) {
                    USER_LOG_ERROR("Maximum retry set home altitude. Pilot should take control!");
                    cleanup();
                    return -1;
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));

            //command drone go home
            USER_LOG_INFO("Command drone go new home location");
            T_DjiReturnCode result_command = DjiFlightController_StartGoHome();
            if (result_command != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Cannot command drone to go home");
                cleanup();
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            round++;
        }

        osalHandler->TaskSleepMs(1000/5); //5 hz
    }

    //wait for landing to complete (flight status == STOPED) before tearing down,
    //so we don't unsubscribe topics while the FC still needs them mid-landing.
    USER_LOG_INFO("Waiting for landing to complete before deinit...");
    auto landing_deadline = std::chrono::steady_clock::now() +
                            std::chrono::seconds(LANDING_WAIT_TIMEOUT_S);
    T_DjiFcSubscriptionFlightStatus flight_status = 0;
    while (std::chrono::steady_clock::now() < landing_deadline) {
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                          (uint8_t *) &flight_status,
                                                          sizeof(T_DjiFcSubscriptionFlightStatus),
                                                          &timestamp);
        if (djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
            flight_status == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED) {
            USER_LOG_INFO("Landing complete, motors stopped.");
            break;
        }
        osalHandler->TaskSleepMs(1000/5); //5 hz
    }
    if (flight_status != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED) {
        USER_LOG_WARN("Landing wait timed out after %d seconds (flight_status=%d). Continuing to deinit anyway.",
                      LANDING_WAIT_TIMEOUT_S, flight_status);
    }

    USER_LOG_DEBUG("Release authority and deinit Flight Control / FC subscription.");
    cleanup();

    return 0;
}
