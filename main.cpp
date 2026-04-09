#include <iostream>
#include <chrono>
#include <memory>
#include <thread>

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
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic flight status failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic display mode failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic avoid data failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic position fused failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude fused failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude of home point failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic RC failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FlightControlDeInit(void)
{
    DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE
    DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION
    DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED
    DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED

    DJI_FC_SUBSCRIPTION_TOPIC_RC
    
    
    T_DjiReturnCode returnCode;

    returnCode = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Unsubscribe topic flight status failed, error code:0x%08llX", returnCode);
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

int main(int argc, char** argv) {
    std::cout << "Initial DJI PSDK...\n";  

    //Init PSDK and set up environment for flight controller sample
    Application application(argc, argv);
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionRC rc_status; 
    T_DjiDataTimestamp timestamp = {0};
                                                                                                                    
    //param 
    int retry_set_home = 1;
    int retry_set_home_altitude = 1;
    T_DjiFlightControllerHomeLocation location;
    E_DjiFlightControllerGoHomeAltitude return_altitude = 90;
    
    //init PSDK flight controller and subscription module
    USER_LOG_DEBUG("Init flight control and data subscription.");
    T_DjiReturnCode init_fc = DjiTest_FlightControlInit();
    if (init_fc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to init flight controller, error code: 0x%08X", init_fc);
        std::cerr << "Failed to init flight controller.\n";
        return -1;
    }
    osalHandler->TaskSleepMs(100);
    
    //TODO: refactor code for loop for repeated set home position -> but behavior of drone will be weird
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
        if (rc_status.mode >= 8000) {
            std::cout << "Get trigger from remote, start to set home location and command drone go home\n";
            break;
        }
        osalHandler->TaskSleepMs(100);
    }
    
    //setting new home location 
    std::cout << "Setting new home location to FC\n";
    
    T_DjiReturnCode set_home_result = DjiFlightController_SetHomeLocationUsingGPSCoordinates(location);
    while (set_home_result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cout << "Cannot set new home location. Retry {%d}/3" << retry_set_home << std::endl;
        set_home_result = DjiFlightController_SetHomeLocationUsingGPSCoordinates(location);
        retry_set_home++;

        if (retry_set_home == 4) {
            std::cout << "Maximum retry set home location. Pilot should take control!\n";
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    time.sleep_for(std::chrono::seconds(1));

    //setting go home altitude 
    std::cout << "Setting go home altitude to FC\n";
    T_DjiReturnCode set_home_altitude_result = DjiFlightController_SetGoHomeAltitude(return_altitude);
    while (set_home_altitude_result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cout << "Cannot set new home altitude. Retry {%d}/3" << retry_set_home_altitude << std::endl;
        set_home_result = DjiFlightController_SetGoHomeAltitude(return_altitude);
        retry_set_home_altitude++;

        if (retry_set_home_altitude == 4) {
            std::cout << "Maximum retry set home location. Pilot should take control!\n";
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    time.sleep_for(std::chrono::seconds(1));

    //command drone go home
    std::cout << "Command drone go new home location\n";
    T_DjiReturnCode result_command = DjiFlightController_StartGoHome();
    if (result_command != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        std::cout << "Cannot command drone to go home\n";
        return -1;
    }
    time.sleep_for(std::chrono::seconds(1));

    USER_LOG_DEBUG("Deinit Flight Control.");
    T_DjiReturnCode deinit_fc = DjiTest_FlightControlDeInit();
    if (deinit_fc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Failed to init flight controller, error code: 0x%08X", deinit_fc);
        std::cerr << "Failed to init flight controller.\n";
        return -1;
    }

    return 0;

}

