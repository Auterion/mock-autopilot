#include <chrono>
#include <iostream>
#include <map>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <thread>
#include <memory>
#include <future>

using namespace mavsdk;

static const std::string COM_RC_IN_MODE("COM_RC_IN_MODE");
static const std::string RC_MAP_ROLL("RC_MAP_ROLL");
static const std::string CAL_MAG2_ID("CAL_MAG2_ID");
static const std::string CAL_MAG1_ID("CAL_MAG1_ID");
static const std::string RC_MAP_AUX2("RC_MAP_AUX2");
static const std::string RC_MAP_AUX1("RC_MAP_AUX1");
static const std::string RC_MAP_FLAPS("RC_MAP_FLAPS");
static const std::string RC_MAP_THROTTLE("RC_MAP_THROTTLE");
static const std::string RC_MAP_YAW("RC_MAP_YAW");
static const std::string RC_MAP_PITCH("RC_MAP_PITCH");
static const std::string RTL_LAND_DELAY("RTL_LAND_DELAY");
static const std::string RTL_DESCEND_ALT("RTL_DESCEND_ALT");
static const std::string RTL_RETURN_ALT("RTL_RETURN_ALT");
static const std::string NAV_DLL_ACT("NAV_DLL_ACT");
static const std::string COM_RC_LOSS_T("COM_RC_LOSS_T");
static const std::string NAV_RCL_ACT("NAV_RCL_ACT");
static const std::string COM_LOW_BAT_ACT("COM_LOW_BAT_ACT");
static const std::string SYS_AUTOSTART("SYS_AUTOSTART");
static const std::string CAL_GYRO0_ID("CAL_GYRO0_ID");
static const std::string RC_MAP_FLTM_BTN("RC_MAP_FLTM_BTN");
static const std::string RC_MAP_MODE_SW("RC_MAP_MODE_SW");
static const std::string SYS_AUTOCONFIG("SYS_AUTOCONFIG");
static const std::string MAV_SYS_ID("MAV_SYS_ID");
static const std::string CAL_ACC0_ID("CAL_ACC0_ID");
static const std::string CAL_MAG0_ID("CAL_MAG0_ID");
static const std::string RC_MAP_RETURN_SW("RC_MAP_RETURN_SW");
static const std::string RC_MAP_LOITER_SW("RC_MAP_LOITER_SW");
static const std::string RC_MAP_POSCTL_SW("RC_MAP_POSCTL_SW");
static const std::string BAT_N_CELLS("BAT_N_CELLS");
static const std::string BAT_V_EMPTY("BAT_V_EMPTY");
static const std::string BAT_V_CHARGED("BAT_V_CHARGED");

std::map<std::string, std::pair<double, int>> params = {
    {COM_RC_IN_MODE, std::make_pair(1, MAV_PARAM_TYPE_INT32)},
    {RC_MAP_ROLL, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {CAL_MAG2_ID, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {CAL_MAG1_ID, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {RC_MAP_AUX2, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {RC_MAP_AUX1, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {RC_MAP_FLAPS, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {RC_MAP_THROTTLE, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {RC_MAP_YAW, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {RC_MAP_PITCH, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {RTL_LAND_DELAY, std::make_pair(0.0, MAV_PARAM_TYPE_REAL32)},
    {RTL_DESCEND_ALT, std::make_pair(30.0, MAV_PARAM_TYPE_REAL32)},
    {RTL_RETURN_ALT, std::make_pair(60.0, MAV_PARAM_TYPE_REAL32)},
    {NAV_DLL_ACT, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {COM_RC_LOSS_T, std::make_pair(0.5, MAV_PARAM_TYPE_REAL32)},
    {NAV_RCL_ACT, std::make_pair(2, MAV_PARAM_TYPE_INT32)},
    {COM_LOW_BAT_ACT, std::make_pair(3, MAV_PARAM_TYPE_INT32)},
    {SYS_AUTOSTART, std::make_pair(4001, MAV_PARAM_TYPE_INT32)},
    {CAL_GYRO0_ID, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {RC_MAP_FLTM_BTN, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {RC_MAP_MODE_SW, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {SYS_AUTOCONFIG, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {MAV_SYS_ID, std::make_pair(2, MAV_PARAM_TYPE_INT32)},
    {CAL_ACC0_ID, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {CAL_MAG0_ID, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {RC_MAP_RETURN_SW, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {RC_MAP_LOITER_SW, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {RC_MAP_POSCTL_SW, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {BAT_N_CELLS, std::make_pair(4, MAV_PARAM_TYPE_INT32)},
    {BAT_V_EMPTY, std::make_pair(3.5, MAV_PARAM_TYPE_REAL32)},
    {BAT_V_CHARGED, std::make_pair(4.2, MAV_PARAM_TYPE_REAL32)},


};

std::vector<std::pair<int, int>> latlngs;
std::shared_ptr<MavlinkPassthrough> mavlink_passthrough;

static void send_param(std::shared_ptr<MavlinkPassthrough>, std::map<std::string, std::pair<double, int>>& params, const std::string& param_id);
static void send_protocol_version(std::shared_ptr<MavlinkPassthrough> mavlink_passthrough);
static void send_autopilot_capabilities(std::shared_ptr<MavlinkPassthrough> mavlink_passthrough);
static void send_requested_message(std::shared_ptr<MavlinkPassthrough> mavlink_passthrough, float id);

static void prepare_vehicle_messages(MavlinkPassthrough& mavlink_passthrough, mavlink_message_t &home_pos, mavlink_message_t &global_pos);
static void subscribe_all(std::shared_ptr<MavlinkPassthrough> mavlink_passthrough);
static void send_telemetry(MavlinkPassthrough& mavlink_passthrough, mavlink_message_t &home_pos, mavlink_message_t &global_pos);

int main(int /* argc */, char** /* argv */)
{
    std::cout << "Starting mock-autopilot" << std::endl;
    bool gcs_connected = false;
    mavlink_message_t home_pos, global_pos;

    Mavsdk mavsdk;
    Mavsdk::Configuration configuration(Mavsdk::Configuration::UsageType::Autopilot);
    mavsdk.set_configuration(configuration);

    ConnectionResult connection_result = mavsdk.setup_udp_remote("127.0.0.1", 14550);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Error setting up UDP connection!" << std::endl;
        return 1;
    }

    // Get Vehicle system that is already created when we configure mavsdk as autopilot
    auto system_vehicle = mavsdk.systems().at(0);
    MavlinkPassthrough mavlink_passthrough_vehicle(system_vehicle);
    prepare_vehicle_messages(mavlink_passthrough_vehicle, home_pos, global_pos);

    while (true) {
        // Send autopilot telemetry
        send_telemetry(mavlink_passthrough_vehicle, home_pos, global_pos);

        // Detect GCS connection or disconnection
        bool detected = false;
        for(auto system : mavsdk.systems()) {
            if (system->get_system_id() == 255) {
                if(!gcs_connected) {
                    mavlink_passthrough = std::make_shared<MavlinkPassthrough>(system);
                    subscribe_all(mavlink_passthrough);
                }
                detected = true;
                gcs_connected = true;
            }
        }

        // Disconnect from GCS system
        if (!detected) {
            gcs_connected = false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

static void prepare_vehicle_messages(MavlinkPassthrough& mavlink_passthrough, mavlink_message_t &home_pos, mavlink_message_t &global_pos) {
    float q[4] = {0, 0, 0, 0};

    mavlink_msg_home_position_pack(
            mavlink_passthrough.get_our_sysid(),
            mavlink_passthrough.get_our_compid(),
            &home_pos,
            465204700,
            66343820,
            431000,
            0,
            0,
            0,
            q,
            0,
            0,
            0,
            0
            );

    mavlink_msg_global_position_int_pack(
            mavlink_passthrough.get_our_sysid(),
            mavlink_passthrough.get_our_compid(),
            &global_pos,
            0,
            465204700,
            66343820,
            440000,
            9,
            0,
            0,
            0,
            45
            );

    // mavlink_msg_heartbeat_pack(
    //         mavlink_passthrough.get_our_sysid(),
    //         mavlink_passthrough.get_our_compid(),
    //         &heartbeat,
    //         MAV_TYPE_QUADROTOR,
    //         MAV_AUTOPILOT_PX4,
    //         0,
    //         0,
    //         MAV_STATE_ACTIVE
    //         );
}

static void subscribe_all(std::shared_ptr<MavlinkPassthrough> mavlink_passthrough) {
    mavlink_passthrough->subscribe_message_async(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, [mavlink_passthrough](const mavlink_message_t& mavlink_message) {
        std::cout << "MAVLINK_MSG_ID_PARAM_REQUEST_LIST: " << mavlink_message.msgid << std::endl;

        for (const auto& param : params) {
            const auto param_id = param.first;
            send_param(mavlink_passthrough, params, param_id);
        }
    });

    mavlink_passthrough->subscribe_message_async(MAVLINK_MSG_ID_PARAM_REQUEST_READ, [mavlink_passthrough](const mavlink_message_t& mavlink_message) {
        mavlink_param_request_read_t param_request_read;
        mavlink_msg_param_request_read_decode(&mavlink_message, &param_request_read);

        if (param_request_read.param_index != -1) {
            std::cout << "Requested param with ID: " << param_request_read.param_index << ". Unsupported, ignoring..." << std::endl;
            return;
        }

        const std::string param_id(param_request_read.param_id);
        std::cout << "Param ID: " << param_id << std::endl;

        send_param(mavlink_passthrough, params, param_id);
    });

    mavlink_passthrough->subscribe_message_async(MAVLINK_MSG_ID_COMMAND_LONG, [mavlink_passthrough](const mavlink_message_t& mavlink_message) {
        std::cout << "MAVLINK_MSG_ID_COMMAND_LONG: " << mavlink_message.msgid << std::endl;

        mavlink_command_long_t cmd_read;
        mavlink_msg_command_long_decode(&mavlink_message, &cmd_read);

        std::cout << "Received COMMAND_LONG with ID: " << cmd_read.command << std::endl;

        const auto cmd_id = cmd_read.command;
        auto mav_result = MAV_RESULT_ACCEPTED;

        switch (cmd_id) {
            case MAV_CMD_REQUEST_MESSAGE:
                send_requested_message(mavlink_passthrough, cmd_read.param1);
                break;
            default:
                std::cout << "Unsupported command" << std::endl;
                return;
        }

        mavlink_message_t message;
        mavlink_msg_command_ack_pack(
                1,
                1,
                &message,
                cmd_id,
                mav_result,
                255,
                -1,
                mavlink_passthrough->get_target_sysid(),
                mavlink_passthrough->get_target_compid()
                );

        mavlink_passthrough->send_message(message);
        std::cout << "Sent ACK for command: " << cmd_id << std::endl;

    });

    mavlink_passthrough->subscribe_message_async(MAVLINK_MSG_ID_COMMAND_INT, [](const mavlink_message_t& mavlink_message) {
        std::cout << "MAVLINK_MSG_ID_COMMAND_INT: " << mavlink_message.msgid << std::endl;
    });

    mavlink_passthrough->subscribe_message_async(MAVLINK_MSG_ID_MISSION_REQUEST_LIST, [mavlink_passthrough](const mavlink_message_t& mavlink_message){
        std::cout << "MAVLINK_MSG_ID_MISSION_REQUEST_LIST: " << mavlink_message.msgid << std::endl;

        const auto mission_size = latlngs.size();

        mavlink_message_t message;
        mavlink_msg_mission_count_pack(
                1,
                1,
                &message,
                mavlink_passthrough->get_target_sysid(),
                mavlink_passthrough->get_target_compid(),
                mission_size,
                MAV_MISSION_TYPE_MISSION
                );
        mavlink_passthrough->send_message(message);
        std::cout << "Sent MISSION_COUNT with size: " << mission_size << std::endl;
    });

    mavlink_passthrough->subscribe_message_async(MAVLINK_MSG_ID_MISSION_REQUEST_INT, [mavlink_passthrough](const mavlink_message_t& mavlink_message) {
        std::cout << "MAVLINK_MSG_ID_MISSION_REQUEST_INT: " << mavlink_message.msgid << std::endl;

        mavlink_mission_request_int_t mission_request_int;
        mavlink_msg_mission_request_int_decode(&mavlink_message, &mission_request_int);

        if (mission_request_int.mission_type != MAV_MISSION_TYPE_MISSION) {
            std::cout << " --> Ignoring request for non-mission item" << std::endl;
        } else {
            const auto requested_item_id = mission_request_int.seq;
            std::cout << " --> Requesting MISSION_ITEM(" << (int)requested_item_id << ")" << std::endl;

            const auto is_current = false;
            const auto autocontinue = true;
            const auto hold_time_s = 0;
            const auto accept_radius_m = 0;
            const auto pass_radius_m = 3;
            const auto yaw_deg = std::nanf("nan");
            const auto altitude_m = 50;

            mavlink_message_t message;
            mavlink_msg_mission_item_int_pack(
                    1,
                    1,
                    &message,
                    mavlink_passthrough->get_target_sysid(),
                    mavlink_passthrough->get_target_compid(),
                    requested_item_id,
                    MAV_FRAME_MISSION,
                    MAV_CMD_NAV_WAYPOINT,
                    is_current,
                    autocontinue,
                    hold_time_s,
                    accept_radius_m,
                    pass_radius_m,
                    yaw_deg,
                    latlngs.at(requested_item_id).first,
                    latlngs.at(requested_item_id).second,
                    altitude_m,
                    MAV_MISSION_TYPE_MISSION
                    );
            mavlink_passthrough->send_message(message);
            std::cout << "Sent mission item with id: " << requested_item_id << std::endl;
        }
    });

    // mavlink_passthrough->subscribe_message_async(MAVLINK_MSG_ID_HEARTBEAT, [](const mavlink_message_t& mavlink_message) {
    //     // std::cout << "MAVLINK_MSG_ID_HEARTBEAT: " << std::to_string(mavlink_message.compid) << std::endl;
    // });
}

static void send_param(std::shared_ptr<MavlinkPassthrough> mavlink_passthrough, std::map<std::string, std::pair<double, int>>& params, const std::string& param_id) {
    const auto param = params[param_id];
    const auto param_value = param.first;
    const auto param_type = param.second;
    const auto param_index = std::distance(params.begin(), params.find(param_id));
    const auto params_count = params.size();

    mavlink_message_t message;
    mavlink_msg_param_value_pack(
            1,
            1,
            &message,
            param_id.c_str(),
            param_value,
            param_type,
            params_count,
            param_index
            );
    mavlink_passthrough->send_message(message);
    std::cout << "Sent " << param_id << " with value: " << param_value << std::endl;
}

static void send_protocol_version(std::shared_ptr<MavlinkPassthrough> mavlink_passthrough) {
    const uint16_t version = 200;
    const uint16_t min_version = 100;
    const uint16_t max_version = 200;
    mavlink_message_t message;
    mavlink_msg_protocol_version_pack(
            mavlink_passthrough->get_our_sysid(),
            mavlink_passthrough->get_our_compid(),
            &message,
            version,
            min_version,
            max_version,
            nullptr,
            nullptr
            );

    mavlink_passthrough->send_message(message);
    std::cout << "Sent protocol version: " << version << " (min: " << min_version << ", max: " << max_version << ")" << std::endl;
}

static void send_autopilot_capabilities(std::shared_ptr<MavlinkPassthrough> mavlink_passthrough) {
    mavlink_message_t message;
    mavlink_msg_autopilot_version_pack(
            1,
            1,
            &message,
            MAV_PROTOCOL_CAPABILITY_MISSION_INT | MAV_PROTOCOL_CAPABILITY_MAVLINK2,
            0x010C0000, // Software version: v1.14.0-dev. 4 bytes (1, 14, 0 , 0). Last 0 means (dev).
            1,
            2,
            3,
            nullptr,
            nullptr,
            nullptr,
            0,
            0,
            1291,
            nullptr
            );

    mavlink_passthrough->send_message(message);
    std::cout << "Sent autopilot capatilities" << std::endl;
}

static void send_requested_message(std::shared_ptr<MavlinkPassthrough> mavlink_passthrough, float id) {
    uint32_t message_id = static_cast<uint32_t>(id);
    switch (message_id) {
        case MAVLINK_MSG_ID_PROTOCOL_VERSION:
            std::cout << "send_protocol_version" << std::endl;
            send_protocol_version(mavlink_passthrough);
            break;
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
            std::cout << "send_autopilot_capabilities" << std::endl;
            send_autopilot_capabilities(mavlink_passthrough);
            break;
        default:
            break;
    }
}

static void send_telemetry(MavlinkPassthrough& mavlink_passthrough, mavlink_message_t &home_pos, mavlink_message_t &global_pos) {

    mavlink_passthrough.send_message(home_pos);
    //std::cout << "Sending home position" << std::endl;
    mavlink_passthrough.send_message(global_pos);
    //std::cout << "Sending position" << std::endl;
}
