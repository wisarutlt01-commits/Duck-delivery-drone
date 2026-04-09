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

const double EARTH_RADIUS_KM = 6371.0;

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
    return EARTH_RADIUS_KM * c;
}

int main(int argc, char** argv) {
    std::cout << "Initial DJI PSDK...\n";  

    //Init PSDK and set up environment for flight controller sample
    Application application(argc, argv);
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionRC rc_status; 
    T_DjiFcSubscriptionHomePointInfo home_point_info = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionDisplaymode display_mode;
    T_DjiDataTimestamp timestamp = {0};
                                                                                                                    
    //param 
    int retry_set_home = 1;
    int retry_set_home_altitude = 1;
    T_DjiFlightControllerHomeLocation location;
    E_DjiFlightControllerGoHomeAltitude return_altitude = 90;
    
    //init PSDK flight controller and subscription module
    USER_LOG_INFO("Init flight control and data subscription.");
    T_DjiReturnCode init_fc = DjiTest_FlightControlInit();
    if (init_fc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to init flight controller, error code: 0x%08X", init_fc);
        std::cerr << "Failed to init flight controller.\n";
        return -1;
    }
    osalHandler->TaskSleepMs(100);
    
    //TODO: How to get position from other hardware?
    //get new home location param from beacon and set new home location param                                       
    location.latitude = 10.05213215;
    location.longitude = 102.1351658418;

    //waiting for trigger from remote 
    while (1) {
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC, 
                                                        (int16_t *) &rc_status,
                                                        sizeof(T_DjiFcSubscriptionRC), 
                                                        &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic rc error.");
        } else {
            if (rc_status.mode >= 8000) {
                USER_LOG_INFO("Get trigger from remote, start to set home location and command drone go home");
                break;
            }        
        }
        osalHandler->TaskSleepMs(1000/5); //5 hz
    }
        
    //TODO: add 3 times retry for set/go home location and check if gps location and home location is less than 2 meters then land.
    int round = 0; 
    while (round < 3) {
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
                                                            (int16_t *) &home_point_info,
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
                                                        (int16_t *) &gpsPosition,
                                                        sizeof(T_DjiFcSubscriptionGpsPosition),
                                                        &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps position error.");
        }

        double distance = calculateHaversineDistance(gpsPosition.x, gpsPosition.y, home_point_info.latitude, home_point_info.longitude);

        if (display_mode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING ) {
            if (distance <= 2) {
                USER_LOG_INFO("Drone has reached home point, distance to home point is %.2f meters", distance);
                break; 
            } else {
                USER_LOG_INFO("Drone is landing, but has not reached home point, distance to home point is %.2f meters", distance);
                T_DjiReturnCode result_command = DjiFlightController_CancelGoHome();
                if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Cannot cancel go home command. Pilot should take control!");
                    return -1;
                }
            }
        }

        if (display_mode != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME) {
            USER_LOG_INFO("Round %d to set home location and command drone go home", round + 1);
            USER_LOG_INFO("Setting new home location to FC");
            T_DjiReturnCode set_home_result = DjiFlightController_SetHomeLocationUsingGPSCoordinates(location);
            while (set_home_result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Cannot set new home location. Retry {%d}/3" , retry_set_home);
                set_home_result = DjiFlightController_SetHomeLocationUsingGPSCoordinates(location);
                retry_set_home++;

                if (retry_set_home == 4) {
                    USER_LOG_ERROR("Maximum retry set home location. Pilot should take control!");
                    return -1;
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));

            //setting go home altitude 
            USER_LOG_INFO("Setting go home altitude to FC");
            T_DjiReturnCode set_home_altitude_result = DjiFlightController_SetGoHomeAltitude(return_altitude);
            while (set_home_altitude_result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Cannot set new home altitude. Retry {%d}/3",retry_set_home_altitude);
                set_home_result = DjiFlightController_SetGoHomeAltitude(return_altitude);
                retry_set_home_altitude++;

                if (retry_set_home_altitude == 4) {
                USER_LOG_ERROR("Maximum retry set home location. Pilot should take control!");
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
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            round++;
        }
        
        osalHandler->TaskSleepMs(1000/5); //5 hz
    }

    USER_LOG_DEBUG("Deinit Flight Control and FC subscription.");
    T_DjiReturnCode deinit_fc = DjiTest_FlightControlDeInit();
    if (deinit_fc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to init flight controller, error code: 0x%08X", deinit_fc);
        std::cerr << "Failed to init flight controller.\n";
        return -1;
    }

    return 0;
}
