#include <chrono>
#include <iostream>
#include <map>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <thread>

using namespace mavsdk;

static const std::string MIS_TAKEOFF_ALT("MIS_TAKEOFF_ALT");
static const std::string CAL_GYRO0_ID("CAL_GYRO0_ID");
static const std::string CAL_ACC0_ID("CAL_ACC0_ID");
static const std::string CAL_MAG0_ID("CAL_MAG0_ID");
static const std::string MPC_XY_CRUISE("MPC_XY_CRUISE");
static const std::string SYS_HITL("SYS_HITL");

std::map<std::string, std::pair<double, int>> params = {
    {MIS_TAKEOFF_ALT, std::make_pair(2.5, MAV_PARAM_TYPE_REAL32)},
    {CAL_GYRO0_ID, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {CAL_ACC0_ID, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {CAL_MAG0_ID, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
    {MPC_XY_CRUISE, std::make_pair(5.5, MAV_PARAM_TYPE_REAL32)},
    {SYS_HITL, std::make_pair(0, MAV_PARAM_TYPE_INT32)},
};

std::vector<std::pair<int, int>> latlngs = {
    std::make_pair(465205340, 66344040),
    std::make_pair(465205380, 66342890),
    std::make_pair(465204770, 66342910),
};

static void send_param(MavlinkPassthrough& mavlink_passthrough, std::map<std::string, std::pair<double, int>>& params, const std::string& param_id);
static void send_protocol_version(MavlinkPassthrough& mavlink_passthrough);
static void send_autopilot_capabilities(MavlinkPassthrough& mavlink_passthrough);

static void start_telemetry(MavlinkPassthrough& mavlink_passthrough);

int main(int /* argc */, char** /* argv */)
{
    std::cout << "Starting fake autopilot" << std::endl;

    Mavsdk mavsdk;
    mavsdk.set_configuration(Mavsdk::Configuration::Autopilot);

    ConnectionResult connection_result = mavsdk.setup_udp_remote("127.0.0.1", 14550);

    if (connection_result != ConnectionResult::SUCCESS) {
        std::cerr << "Error setting up UDP connection!" << std::endl;
        return 1;
    }

    System& system = mavsdk.system();
    MavlinkPassthrough mavlink_passthrough(system);

    mavlink_passthrough.subscribe_message_async(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, [&mavlink_passthrough](const mavlink_message_t& mavlink_message) {
        std::cout << "MAVLINK_MSG_ID_PARAM_REQUEST_LIST: " << mavlink_message.msgid << std::endl;

        for (const auto& param : params) {
            const auto param_id = param.first;
            send_param(mavlink_passthrough, params, param_id);
        }
    });

    mavlink_passthrough.subscribe_message_async(MAVLINK_MSG_ID_PARAM_REQUEST_READ, [&mavlink_passthrough](const mavlink_message_t& mavlink_message) {
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

    mavlink_passthrough.subscribe_message_async(MAVLINK_MSG_ID_COMMAND_LONG, [&mavlink_passthrough](const mavlink_message_t& mavlink_message) {
        std::cout << "MAVLINK_MSG_ID_COMMAND_LONG: " << mavlink_message.msgid << std::endl;

        mavlink_command_long_t cmd_read;
        mavlink_msg_command_long_decode(&mavlink_message, &cmd_read);

        std::cout << "Received COMMAND_LONG with ID: " << cmd_read.command << std::endl;

        const auto cmd_id = cmd_read.command;
        auto mav_result = MAV_RESULT_ACCEPTED;

        switch (cmd_id) {
            case MAV_CMD_REQUEST_PROTOCOL_VERSION:
                send_protocol_version(mavlink_passthrough);
                break;
            case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                send_autopilot_capabilities(mavlink_passthrough);
                break;
            default:
                mav_result = MAV_RESULT_UNSUPPORTED;
                break;
        }

        mavlink_message_t message;
        mavlink_msg_command_ack_pack(
                mavlink_passthrough.get_our_sysid(),
                mavlink_passthrough.get_our_compid(),
                &message,
                cmd_id,
                mav_result,
                255,
                -1,
                mavlink_passthrough.get_target_sysid(),
                mavlink_passthrough.get_target_compid()
                );
        mavlink_passthrough.send_message(message);
        std::cout << "Sent ACK for command: " << cmd_id << std::endl;
    });

    mavlink_passthrough.subscribe_message_async(MAVLINK_MSG_ID_COMMAND_INT, [](const mavlink_message_t& mavlink_message) {
        std::cout << "MAVLINK_MSG_ID_COMMAND_INT: " << mavlink_message.msgid << std::endl;
    });

    mavlink_passthrough.subscribe_message_async(MAVLINK_MSG_ID_HEARTBEAT, [](const mavlink_message_t& /*mavlink_message*/) {
        //std::cout << "MAVLINK_MSG_ID_HEARTBEAT: " << mavlink_message.msgid << std::endl;
    });

    mavlink_passthrough.subscribe_message_async(MAVLINK_MSG_ID_MISSION_REQUEST_LIST, [&mavlink_passthrough](const mavlink_message_t& mavlink_message){
        std::cout << "MAVLINK_MSG_ID_MISSION_REQUEST_LIST: " << mavlink_message.msgid << std::endl;

        const auto mission_size = latlngs.size();

        mavlink_message_t message;
        mavlink_msg_mission_count_pack(
                mavlink_passthrough.get_our_sysid(),
                mavlink_passthrough.get_our_compid(),
                &message,
                mavlink_passthrough.get_target_sysid(),
                mavlink_passthrough.get_target_compid(),
                mission_size,
                MAV_MISSION_TYPE_MISSION
                );
        mavlink_passthrough.send_message(message);
        std::cout << "Sent MISSION_COUNT with size: " << mission_size << std::endl;
    });

    mavlink_passthrough.subscribe_message_async(MAVLINK_MSG_ID_MISSION_REQUEST_INT, [&mavlink_passthrough](const mavlink_message_t& mavlink_message) {
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
                    mavlink_passthrough.get_our_sysid(),
                    mavlink_passthrough.get_our_compid(),
                    &message,
                    mavlink_passthrough.get_target_sysid(),
                    mavlink_passthrough.get_target_compid(),
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
            mavlink_passthrough.send_message(message);
            std::cout << "Sent mission item with id: " << requested_item_id << std::endl;
        }
    });

    start_telemetry(mavlink_passthrough);

    return 0;
}

static void send_param(MavlinkPassthrough& mavlink_passthrough, std::map<std::string, std::pair<double, int>>& params, const std::string& param_id) {
    const auto param = params[param_id];
    const auto param_value = param.first;
    const auto param_type = param.second;
    const auto param_index = std::distance(params.begin(), params.find(param_id));
    const auto params_count = params.size();

    mavlink_message_t message;
    mavlink_msg_param_value_pack(
            mavlink_passthrough.get_our_sysid(),
            mavlink_passthrough.get_our_compid(),
            &message,
            param_id.c_str(),
            param_value,
            param_type,
            params_count,
            param_index
            );
    mavlink_passthrough.send_message(message);
    std::cout << "Sent " << param_id << " with value: " << param_value << std::endl;
}

static void send_protocol_version(MavlinkPassthrough& mavlink_passthrough) {
    const uint16_t version = 200;
    const uint16_t min_version = 100;
    const uint16_t max_version = 200;
    mavlink_message_t message;
    mavlink_msg_protocol_version_pack(
            mavlink_passthrough.get_our_sysid(),
            mavlink_passthrough.get_our_compid(),
            &message,
            version,
            min_version,
            max_version,
            nullptr,
            nullptr
            );
    mavlink_passthrough.send_message(message);
    std::cout << "Sent protocol version: " << version << " (min: " << min_version << ", max: " << max_version << ")" << std::endl;
}

static void send_autopilot_capabilities(MavlinkPassthrough& mavlink_passthrough) {
    mavlink_message_t message;
    mavlink_msg_autopilot_version_pack(
            mavlink_passthrough.get_our_sysid(),
            mavlink_passthrough.get_our_compid(),
            &message,
            8196,
            0,
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
    mavlink_passthrough.send_message(message);
    std::cout << "Sent autopilot capatilities" << std::endl;
}

static void start_telemetry(MavlinkPassthrough& mavlink_passthrough) {
    std::cout << "Starting telemetry" << std::endl;
    float q[4] = {0, 0, 0, 0};

    mavlink_message_t message0;
    mavlink_msg_home_position_pack(
            mavlink_passthrough.get_our_sysid(),
            mavlink_passthrough.get_our_compid(),
            &message0,
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

    mavlink_message_t message;
    mavlink_msg_global_position_int_pack(
            mavlink_passthrough.get_our_sysid(),
            mavlink_passthrough.get_our_compid(),
            &message,
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

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        mavlink_passthrough.send_message(message0);
        //std::cout << "Sending home position" << std::endl;
        mavlink_passthrough.send_message(message);
        //std::cout << "Sending position" << std::endl;
    }
}
