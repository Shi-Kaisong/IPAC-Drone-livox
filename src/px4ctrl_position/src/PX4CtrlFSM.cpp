#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_) : param(param_), controller(controller_)
{
    state = MANUAL_CTRL;
    hover_pose.setZero();
    current_drop_index = 0;
    // 初始化障碍物、圆环等位置（实际应用中可能需要从外部配置或传感器获取）
    obstacle_position = Eigen::Vector3d(12.5, 12.5, 0.0);
    ring_position = Eigen::Vector3d(15.0, 20.0, 1.5);
}

/* 
        Finite State Machine

              system start
                    |
                    |
                    v
    ----- > MANUAL_CTRL <-----------------
    |         ^   |    \                 |
    |         |   |     \                |
    |         |   |      > AUTO_TAKEOFF  |
    |         |   |        /             |
    |         |   |       /              |
    |         |   |      /               |
    |         |   v     /                |
    |       AUTO_HOVER <                 |
    |         ^   |  \  \                |
    |         |   |   \  \               |
    |         |   |    > AUTO_LAND -------
    |         |   |
    |         |   v
    -------- CMD_CTRL

*/

void PX4CtrlFSM::process()
{
    ros::Time now_time = ros::Time::now();
    Controller_Output_t u;
    Desired_State_t des(odom_data);
    bool rotor_low_speed_during_land = false;

    // STEP1: state machine runs
    switch (state)
    {
    case MANUAL_CTRL:
    {
        if (rc_data.enter_hover_mode) // Try to jump to AUTO_HOVER
        {
            if (!odom_is_received(now_time))
            {
                ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). No odom!");
                break;
            }
            if (cmd_is_received(now_time))
            {
                ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
                break;
            }
            if (odom_data.v.norm() > 3.0)
            {
                ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
                break;
            }

            state = AUTO_HOVER;
            controller.resetThrustMapping();
            set_hov_with_odom();
            toggle_offboard_mode(true);

            ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
        }
        else if (param.takeoff_land.enable && takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF) // Try to jump to AUTO_TAKEOFF
        {
            if (!odom_is_received(now_time))
            {
                ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. No odom!");
                break;
            }
            if (cmd_is_received(now_time))
            {
                ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. You are sending commands before toggling into AUTO_TAKEOFF, which is not allowed. Stop sending commands now!");
                break;
            }
            if (odom_data.v.norm() > 0.1)
            {
                ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
                break;
            }
            if (!get_landed())
            {
                ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. land detector says that the drone is not landed now!");
                break;
            }
            if (rc_is_received(now_time)) // Check this only if RC is connected.
            {
                if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered())
                {
                    ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
                    while (ros::ok())
                    {
                        ros::Duration(0.01).sleep();
                        ros::spinOnce();
                        if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered())
                        {
                            ROS_INFO("\033[32m[px4ctrl] OK, you can takeoff again.\033[32m");
                            break;
                        }
                    }
                    break;
                }
            }

            state = AUTO_TAKEOFF;
            controller.resetThrustMapping();
            set_start_pose_for_takeoff_land(odom_data);
            toggle_offboard_mode(true); // toggle on offboard before arm

            for (int i = 0; i < 10 && ros::ok(); ++i) // wait for 0.1 seconds to allow mode change by FMU
            {
                ros::Duration(0.01).sleep();
                ros::spinOnce();
            }
            if (param.takeoff_land.enable_auto_arm)
            {
                toggle_arm_disarm(true);
            }
            takeoff_land.toggle_takeoff_land_time = now_time;

            ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
        }

        if (rc_data.toggle_reboot) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
        {
            if (state_data.current_state.armed)
            {
                ROS_ERROR("[px4ctrl] Reject reboot! Disarm the drone first!");
                break;
            }
            reboot_FCU();
        }

        break;
    }

    case AUTO_HOVER:
    {
        if (!rc_data.is_hover_mode || !odom_is_received(now_time))
        {
            state = MANUAL_CTRL;
            toggle_offboard_mode(false);

            ROS_WARN("[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
        }
        else if (rc_data.is_command_mode && cmd_is_received(now_time))
        {
            if (state_data.current_state.mode == "OFFBOARD")
            {
                state = CMD_CTRL;
                des = get_cmd_des();
                ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
            }
        }
        else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
        {
            state = AUTO_LAND;
            set_start_pose_for_takeoff_land(odom_data);

            ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
        }
        else
        {
            set_hov_with_rc();
            des = get_hover_des();
            if ((rc_data.enter_command_mode) ||
                (takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second))
            {
                takeoff_land.delay_trigger.first = false;
                publish_trigger(odom_data.msg);
                ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[32m");
            }
        }

        break;
    }

    case CMD_CTRL:
    {
        if (!rc_data.is_hover_mode || !odom_is_received(now_time))
        {
            state = MANUAL_CTRL;
            toggle_offboard_mode(false);

            ROS_WARN("[px4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
        }
        else if (!rc_data.is_command_mode || !cmd_is_received(now_time))
        {
            state = AUTO_HOVER;
            set_hov_with_odom();
            des = get_hover_des();
            ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
        }
        else
        {
            des = get_cmd_des();
        }

        if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
        {
            ROS_ERROR("[px4ctrl] Reject AUTO_LAND, which must be triggered in AUTO_HOVER. \
                    Stop sending control commands for longer than %fs to let px4ctrl return to AUTO_HOVER first.",
                      param.msg_timeout.cmd);
        }

        break;
    }

    case AUTO_TAKEOFF:
    {
        if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) // Wait for several seconds to warn prople.
        {
            des = get_rotor_speed_up_des(now_time);
        }
        else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height)) // reach the desired height
        {
            state = QR_CODE_RECOGNITION; // 起飞完成后进入二维码识别状态
            ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> QR_CODE_RECOGNITION\033[32m");
        }
        else
        {
            des = get_takeoff_land_des(param.takeoff_land.speed);
        }

        break;
    }

    case QR_CODE_RECOGNITION:
    {
        des = get_qr_code_recognition_des();
        if (recognize_qr_code())
        {
            state = FLY_TO_OBSTACLE;
            ROS_INFO("\033[32m[px4ctrl] QR_CODE_RECOGNITION --> FLY_TO_OBSTACLE\033[32m");
        }
        else
        {
            // 如果二维码识别失败，尝试再次识别或返回降落
            static int recognition_attempts = 0;
            recognition_attempts++;
            if (recognition_attempts > 3) {
                ROS_ERROR("[px4ctrl] QR code recognition failed after multiple attempts, returning to land");
                state = AUTO_LAND;
                set_start_pose_for_takeoff_land(odom_data);
            }
        }
        break;
    }

    case FLY_TO_OBSTACLE:
    {
        des = get_fly_to_obstacle_des();
        if (reach_obstacle())
        {
            state = CIRCLE_OBSTACLE;
            // 重置绕障计数器
            static double start_angle = std::atan2(odom_data.p.y() - obstacle_position.y(), odom_data.p.x() - obstacle_position.x());
            start_angle = std::atan2(odom_data.p.y() - obstacle_position.y(), odom_data.p.x() - obstacle_position.x());
            ROS_INFO("\033[32m[px4ctrl] FLY_TO_OBSTACLE --> CIRCLE_OBSTACLE\033[32m");
        }
        break;
    }

    case CIRCLE_OBSTACLE:
    {
        des = get_circle_obstacle_des();
        if (finish_circle_obstacle())
        {
            state = DROP_OBJECTS;
            current_drop_index = 0; // 重置投放点索引
            ROS_INFO("\033[32m[px4ctrl] CIRCLE_OBSTACLE --> DROP_OBJECTS\033[32m");
        }
        break;
    }

    case DROP_OBJECTS:
    {
        des = get_drop_objects_des();
        if (finish_drop_objects())
        {
            state = PASS_THROUGH_RING;
            ROS_INFO("\033[32m[px4ctrl] DROP_OBJECTS --> PASS_THROUGH_RING\033[32m");
        }
        break;
    }

    case PASS_THROUGH_RING:
    {
        des = get_pass_through_ring_des();
        if (pass_through_ring())
        {
            state = FLY_TO_LANDING_POINT;
            ROS_INFO("\033[32m[px4ctrl] PASS_THROUGH_RING --> FLY_TO_LANDING_POINT\033[32m");
        }
        break;
    }

    case FLY_TO_LANDING_POINT:
    {
        des = get_fly_to_landing_point_des();
        if (reach_landing_point())
        {
            state = AUTO_LAND;
            set_start_pose_for_takeoff_land(odom_data);
            ROS_INFO("\033[32m[px4ctrl] FLY_TO_LANDING_POINT --> AUTO_LAND\033[32m");
        }
        break;
    }

    case AUTO_LAND:
    {
        if (!rc_data.is_hover_mode || !odom_is_received(now_time))
        {
            state = MANUAL_CTRL;
            toggle_offboard_mode(false);

            ROS_WARN("[px4ctrl] From AUTO_LAND to MANUAL_CTRL(L1)!");
        }
        else if (!rc_data.is_command_mode)
        {
            state = AUTO_HOVER;
            set_hov_with_odom();
            des = get_hover_des();
            ROS_INFO("[px4ctrl] From AUTO_LAND to AUTO_HOVER(L2)!");
        }
        else if (!get_landed())
        {
            des = get_takeoff_land_des(-param.takeoff_land.speed);
        }
        else
        {
            rotor_low_speed_during_land = true;

            static bool print_once_flag = true;
            if (print_once_flag)
            {
                ROS_INFO("\033[32m[px4ctrl] Wait for abount 10s to let the drone arm.\033[32m");
                print_once_flag = false;
            }

            if (extended_state_data.current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) // PX4 allows disarm after this
            {
                static double last_trial_time = 0; // Avoid too frequent calls

                if (now_time.toSec() - last_trial_time > 1.0)
                {
                    if (toggle_arm_disarm(false)) // disarm
                    {
                        print_once_flag = true;
                        state = MANUAL_CTRL;
                        toggle_offboard_mode(false); // toggle off offboard after disarm
                        ROS_INFO("\033[32m[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[32m");
                    }

                    last_trial_time = now_time.toSec();
                }
            }
        }

        break;
    }

    default:
        break;
    }

    // STEP2: estimate thrust model
    if (state == AUTO_HOVER || state == CMD_CTRL)
    {
        controller.estimateThrustModel(imu_data.a, param);
    }

    // STEP3: solve and update new control commands
    if (rotor_low_speed_during_land) // used at the start of auto takeoff
    {
        motors_idling(imu_data, u);
    }
    else
    {
        debug_msg = controller.calculateControl(des, odom_data, imu_data, u);
        debug_msg.header.stamp = now_time;
        debug_pub.publish(debug_msg);
    }

    // STEP4: publish control commands to mavros
    if (param.use_position_ctrl && rc_data.position_ctrl && state == CMD_CTRL && rc_data.is_command_mode && rc_data.is_hover_mode)
    {
        publish_position_ctrl(now_time);
    }
    else
    {
        if (param.use_bodyrate_ctrl)
        {
            publish_bodyrate_ctrl(u, now_time);
        }
        else
        {
            publish_attitude_ctrl(u, now_time);
        }
    }
    // STEP5: Detect if the drone has landed
    land_detector(state, des, odom_data);

    // STEP6: Clear flags beyound their lifetime
    rc_data.enter_hover_mode = false;
    rc_data.enter_command_mode = false;
    rc_data.toggle_reboot = false;
    takeoff_land_data.triggered = false;
}

// 新增函数实现
Desired_State_t PX4CtrlFSM::get_qr_code_recognition_des()
{
    // 实现二维码识别状态下的期望状态
    Desired_State_t des;
    // 保持当前位置悬停，略微升高以获得更好的视野
    des.p = odom_data.p + Eigen::Vector3d(0, 0, 0.5);
    des.v = Eigen::Vector3d::Zero();
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = odom_data.yaw;
    des.yaw_rate = 0.1; // 缓慢旋转以寻找二维码
    return des;
}

Desired_State_t PX4CtrlFSM::get_fly_to_obstacle_des()
{
    // 实现飞行至障碍物状态下的期望状态
    Desired_State_t des;
    des.p = obstacle_position + Eigen::Vector3d(0, 0, 1.0); // 障碍物上方1米
    des.v = (des.p - odom_data.p).normalized() * param.takeoff_land.speed;
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = std::atan2(obstacle_position.y() - odom_data.p.y(), obstacle_position.x() - odom_data.p.x());
    des.yaw_rate = 0.0;
    return des;
}

Desired_State_t PX4CtrlFSM::get_circle_obstacle_des()
{
    // 实现绕障碍物飞行状态下的期望状态
    Desired_State_t des;
    // 绕障碍物的半径
    double radius = 2.0; // 可根据实际情况调整
    // 计算当前位置相对于障碍物的角度
    Eigen::Vector3d relative_position = odom_data.p - obstacle_position;
    double angle = std::atan2(relative_position.y(), relative_position.x());
    // 顺时针绕障碍物飞行，角度增加
    angle += 0.02; // 调整角速度
    des.p = obstacle_position + Eigen::Vector3d(radius * std::cos(angle), radius * std::sin(angle), odom_data.p(2));
    des.v = (des.p - odom_data.p).normalized() * param.takeoff_land.speed;
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = angle + M_PI/2; // 朝向障碍物中心
    des.yaw_rate = 0.0;
    return des;
}

Desired_State_t PX4CtrlFSM::get_drop_objects_des()
{
    // 实现投放物块状态下的期望状态
    Desired_State_t des;
    if (!drop_positions.empty() && current_drop_index < drop_positions.size())
    {
        des.p = drop_positions[current_drop_index];
        des.v = (des.p - odom_data.p).normalized() * param.takeoff_land.speed;
        
        // 当接近投放点时，减速并准备投放
        if ((des.p - odom_data.p).norm() < 0.3) {
            des.v *= 0.5; // 减速
            
            // 高度调整，准备投放
            if (des.p(2) > 0.5) {
                des.p(2) = 0.5; // 降低到投放高度
            }
            
            static ros::Time start_wait_time;
            static bool started_waiting = false;
            
            if (!started_waiting) {
                start_wait_time = ros::Time::now();
                started_waiting = true;
                ROS_INFO("[px4ctrl] Preparing to drop object at position %d", current_drop_index);
            }
            
            // 悬停一段时间后，标记为已投放
            if ((ros::Time::now() - start_wait_time).toSec() > 3.0) {
                current_drop_index++;
                started_waiting = false;
                ROS_INFO("[px4ctrl] Dropped object at position %d", current_drop_index - 1);
            }
        }
    }
    else
    {
        des.p = odom_data.p;
        des.v = Eigen::Vector3d::Zero();
    }
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = odom_data.yaw;
    des.yaw_rate = 0.0;
    return des;
}

Desired_State_t PX4CtrlFSM::get_pass_through_ring_des()
{
    // 实现穿越圆环状态下的期望状态
    Desired_State_t des;
    des.p = ring_position;
    des.v = (des.p - odom_data.p).normalized() * param.takeoff_land.speed * 1.5; // 加速穿越
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = std::atan2(ring_position.y() - odom_data.p.y(), ring_position.x() - odom_data.p.x());
    des.yaw_rate = 0.0;
    
    // 安全检查，如果距离圆环太近，减速
    if ((des.p - odom_data.p).norm() < 0.5) {
        des.v *= 0.7;
    }
    
    return des;
}

Desired_State_t PX4CtrlFSM::get_fly_to_landing_point_des()
{
    // 实现飞行至降落点状态下的期望状态
    Desired_State_t des;
    des.p = landing_point + Eigen::Vector3d(0, 0, 1.0); // 降落点上方1米
    des.v = (des.p - odom_data.p).normalized() * param.takeoff_land.speed;
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = std::atan2(landing_point.y() - odom_data.p.y(), landing_point.x() - odom_data.p.x());
    des.yaw_rate = 0.0;
    return des;
}

// 新增辅助函数实现
bool PX4CtrlFSM::recognize_qr_code()
{
    // 从二维码中解析的位置数据
    std::string category1, category2, direction;
    bool qr_detected = false;
    
    // 预定义类别与位置的映射关系（实际应用中可能从配置文件加载）
    std::map<std::string, Eigen::Vector3d> category_positions = {
        {"apple", {10.0, 10.0, 1.0}},
        {"banana", {15.0, 10.0, 1.0}},
        {"cherry", {20.0, 10.0, 1.0}},
        {"date", {10.0, 15.0, 1.0}},
        {"elderberry", {15.0, 15.0, 1.0}},
        {"fig", {20.0, 15.0, 1.0}}
    };
    
    // 预定义方向与降落点的映射关系
    std::map<std::string, Eigen::Vector3d> direction_positions = {
        {"left", {5.0, 5.0, 0.0}},
        {"right", {25.0, 5.0, 0.0}}
    };
    
    // 打开视频流
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        ROS_ERROR("[px4ctrl] 无法打开视频流，尝试使用备用设备");
        cap.open(1); // 尝试使用备用设备
        if (!cap.isOpened()) {
            ROS_ERROR("[px4ctrl] 所有视频设备均无法打开");
            return false;
        }
    }
    
    // 设置摄像头参数
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    // 设置超时时间（秒）
    const double timeout = 15.0;
    ros::Time start_time = ros::Time::now();
    
    // 尝试识别二维码，直到成功或超时
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < timeout) {
        // 读取视频帧
        cv::Mat frame;
        bool ret = cap.read(frame);
        if (!ret) {
            ROS_WARN("[px4ctrl] 视频流读取失败");
            break;
        }
        
        // 实例化QR码检测器
        cv::QRCodeDetector qrcoder;
        
        // QR检测并解码
        std::string codeinfo;
        std::vector<cv::Point> points;
        cv::Mat straight_qrcode;
        bool detected = qrcoder.detectAndDecode(frame, codeinfo, points, straight_qrcode);
        
        if (detected && !codeinfo.empty()) {
            // 绘制QR码的检测结果
            if (!points.empty()) {
                std::vector<std::vector<cv::Point>> contours = {points};
                cv::drawContours(frame, contours, 0, cv::Scalar(0, 0, 255), 2);
            }
            
            // 打印解码结果
            ROS_INFO("[px4ctrl] 识别到二维码: %s", codeinfo.c_str());
            
            // 解析二维码内容（格式假设为: "category1,category2,left" 或 "category1,category2,right"）
            try {
                // 分割内容
                std::vector<std::string> parts;
                std::stringstream ss(codeinfo);
                std::string part;
                
                while (std::getline(ss, part, ',')) {
                    parts.push_back(part);
                }
                
                // 验证格式
                if (parts.size() == 3) {
                    category1 = parts[0];
                    category2 = parts[1];
                    direction = parts[2];
                    
                    // 检查类别和方向是否有效
                    if (category_positions.find(category1) != category_positions.end() &&
                        category_positions.find(category2) != category_positions.end() &&
                        direction_positions.find(direction) != direction_positions.end()) {
                        
                        // 获取对应位置
                        Eigen::Vector3d pos1 = category_positions[category1];
                        Eigen::Vector3d pos2 = category_positions[category2];
                        Eigen::Vector3d landing = direction_positions[direction];
                        
                        // 清空原有投放点
                        this->drop_positions.clear();
                        
                        // 添加投放点，先投放category1，再投放category2
                        this->drop_positions.push_back(pos1);
                        this->drop_positions.push_back(pos2);
                        
                        // 设置降落点
                        landing_point = landing;
                        
                        ROS_INFO("[px4ctrl] 成功从二维码解析位置数据:");
                        ROS_INFO("[px4ctrl] 投放点1: %s -> (%f, %f, %f)", category1.c_str(), pos1.x(), pos1.y(), pos1.z());
                        ROS_INFO("[px4ctrl] 投放点2: %s -> (%f, %f, %f)", category2.c_str(), pos2.x(), pos2.y(), pos2.z());
                        ROS_INFO("[px4ctrl] 降落点: %s -> (%f, %f, %f)", direction.c_str(), landing.x(), landing.y(), landing.z());
                        
                        qr_detected = true;
                        break;
                    } else {
                        ROS_WARN("[px4ctrl] 二维码内容包含无效类别或方向: %s, %s, %s", 
                                category1.c_str(), category2.c_str(), direction.c_str());
                    }
                } else {
                    ROS_WARN("[px4ctrl] 二维码格式错误，需要三个部分: category1,category2,direction");
                }
            } catch (const std::exception& e) {
                ROS_ERROR("[px4ctrl] 解析二维码内容失败: %s", e.what());
            }
        }
    }
    
    // 释放视频流资源
    cap.release();
    
    if (qr_detected) {
        return true;
    } else {
        ROS_ERROR("[px4ctrl] 二维码识别超时或失败");
        return false;
    }
}

bool PX4CtrlFSM::reach_obstacle()
{
    // 判断是否到达障碍物附近
    return (odom_data.p - (obstacle_position + Eigen::Vector3d(0, 0, 1.0))).norm() < 0.3;
}

bool PX4CtrlFSM::finish_circle_obstacle()
{
    // 判断是否绕障碍物飞行完成
    static double start_angle = std::atan2(odom_data.p.y() - obstacle_position.y(), odom_data.p.x() - obstacle_position.x());
    double current_angle = std::atan2(odom_data.p.y() - obstacle_position.y(), odom_data.p.x() - obstacle_position.x());
    double delta_angle = current_angle - start_angle;
    
    // 处理角度循环
    if (delta_angle < 0) delta_angle += 2 * M_PI;
    
    // 完成一圈（2π弧度）
    return delta_angle >= 2 * M_PI;
}

bool PX4CtrlFSM::finish_drop_objects()
{
    // 判断是否投放完成所有物块
    return drop_positions.empty() || current_drop_index >= drop_positions.size();
}

bool PX4CtrlFSM::pass_through_ring()
{
    // 判断是否穿越圆环
    // 简化版：检查是否通过圆环位置
    static bool passed = false;
    if (!passed && (odom_data.p - ring_position).norm() < 0.3) {
        passed = true;
        ROS_INFO("[px4ctrl] Passed through the ring!");
    }
    return passed;
}

bool PX4CtrlFSM::reach_landing_point()
{
    // 判断是否到达降落点上方
    return (odom_data.p - (landing_point + Eigen::Vector3d(0, 0, 1.0))).norm() < 0.3;
}

void PX4CtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u)
{
    u.q = imu.q;
    u.bodyrates = Eigen::Vector3d::Zero();
    u.thrust = 0.04;
}

void PX4CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
    static State_t last_state = State_t::MANUAL_CTRL;
    if (last_state == State_t::MANUAL_CTRL && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF))
    {
        takeoff_land.landed = false; // Always holds
    }
    last_state = state;

    if (state == State_t::MANUAL_CTRL && !state_data.current_state.armed)
    {
        takeoff_land.landed = true;
        return; // No need of other decisions
    }

    // land_detector parameters
    constexpr double POSITION_DEVIATION_C = -0.5; // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
    constexpr double VELOCITY_THR_C = 0.1;        // Constraint 2: velocity below VELOCITY_MIN_C m/s.
    constexpr double TIME_KEEP_C = 3.0;           // Constraint 3: Time(s) the Constraint 1&2 need to keep.

    static ros::Time time_C12_reached; // time_Constraints12_reached
    static bool is_last_C12_satisfy;
    if (takeoff_land.landed)
    {
        time_C12_reached = ros::Time::now();
        is_last_C12_satisfy = false;
    }
    else
    {
        bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
        if (C12_satisfy && !is_last_C12_satisfy)
        {
            time_C12_reached = ros::Time::now();
        }
        else if (C12_satisfy && is_last_C12_satisfy)
        {
            if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C) //Constraint 3 reached
            {
                takeoff_land.landed = true;
            }
        }
        is_last_C12_satisfy = C12_satisfy;
    }
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
    takeoff_land.start_pose.head<3>() = odom.p;
    takeoff_land.start_pose(3) = odom.yaw;
}

Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const ros::Time now)
{
    Desired_State_t des;
    double t = (now - takeoff_land.toggle_takeoff_land_time).toSec();
    double ratio = t / AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME;
    ratio = min(1.0, max(0.0, ratio));

    des.p = takeoff_land.start_pose.head<3>();
    des.v = Eigen::Vector3d::Zero();
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = takeoff_land.start_pose(3);
    des.yaw_rate = 0.0;

    return des;
}

Desired_State_t PX4CtrlFSM::get_takeoff_land_des(const double speed)
{
    Desired_State_t des;
    double t = (ros::Time::now() - takeoff_land.toggle_takeoff_land_time).toSec();

    des.p = takeoff_land.start_pose.head<3>();
    des.p(2) += speed * t;
    des.v = Eigen::Vector3d(0, 0, speed);
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = takeoff_land.start_pose(3);
    des.yaw_rate = 0.0;

    return des;
}

void PX4CtrlFSM::set_hov_with_odom()
{
    hover_pose.head<3>() = odom_data.p;
    hover_pose(3) = odom_data.yaw;
    last_set_hover_pose_time = ros::Time::now();
}

void PX4CtrlFSM::set_hov_with_rc()
{
    // Adjust hover position with RC sticks
    Eigen::Vector3d dp;
    dp.x() = -rc_data.roll * 0.02;
    dp.y() = -rc_data.pitch * 0.02;
    dp.z() = -rc_data.throttle * 0.02;

    hover_pose.head<3>() += dp;
    hover_pose(3) += -rc_data.yaw * 0.01;
}

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = on_off ? "OFFBOARD" : "MANUAL";

    if (set_FCU_mode_srv.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent)
    {
        ROS_INFO("\033[32m[px4ctrl] %s mode enabled\033[32m", on_off ? "OFFBOARD" : "MANUAL");
        return true;
    }
    else
    {
        ROS_ERROR("[px4ctrl] Failed to %s OFFBOARD mode", on_off ? "enable" : "disable");
        return false;
    }
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;

    if (arming_client_srv.call(arm_cmd) &&
        arm_cmd.response.success)
    {
        ROS_INFO("\033[32m[px4ctrl] Vehicle %s\033[32m", arm ? "ARMED" : "DISARMED");
        return true;
    }
    else
    {
        ROS_ERROR("[px4ctrl] Failed to %s vehicle", arm ? "ARM" : "DISARM");
        return false;
    }
}

void PX4CtrlFSM::reboot_FCU()
{
    mavros_msgs::CommandLong reboot_cmd;
    reboot_cmd.request.broadcast = false;
    reboot_cmd.request.command = 182; // CMD_PREFLIGHT_REBOOT_SHUTDOWN
    reboot_cmd.request.param1 = 1.0;  // Reboot autopilot
    reboot_cmd.request.param2 = 0.0;  // Do not reboot onboard computer

    if (reboot_FCU_srv.call(reboot_cmd) &&
        reboot_cmd.response.success)
    {
        ROS_INFO("\033[32m[px4ctrl] FCU rebooting\033[32m");
    }
    else
    {
        ROS_ERROR("[px4ctrl] Failed to reboot FCU");
    }
}

void PX4CtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
    mavros_msgs::AttitudeTarget att_msg;
    att_msg.header.stamp = stamp;
    att_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

    att_msg.body_rate.x = u.bodyrates.x();
    att_msg.body_rate.y = u.bodyrates.y();
    att_msg.body_rate.z = u.bodyrates.z();
    att_msg.thrust = u.thrust;

    ctrl_FCU_pub.publish(att_msg);
}

void PX4CtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
    mavros_msgs::AttitudeTarget att_msg;
    att_msg.header.stamp = stamp;
    att_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    att_msg.orientation.w = u.q.w();
    att_msg.orientation.x = u.q.x();
    att_msg.orientation.y = u.q.y();
    att_msg.orientation.z = u.q.z();
    att_msg.thrust = u.thrust;

    ctrl_FCU_pub.publish(att_msg);
}

void PX4CtrlFSM::publish_position_ctrl(const ros::Time &stamp)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.pose.position.x = cmd_data.pos_sp(0);
    pose.pose.position.y = cmd_data.pos_sp(1);
    pose.pose.position.z = cmd_data.pos_sp(2);

    Eigen::Quaterniond q = Eigen::AngleAxisd(cmd_data.yaw_sp, Eigen::Vector3d::UnitZ());
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();

    local_pos_pub.publish(pose);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
    geometry_msgs::PoseStamped trigger_msg;
    trigger_msg.header = odom_msg.header;
    trigger_msg.pose = odom_msg.pose.pose;
    traj_start_trigger_pub.publish(trigger_msg);
}

Desired_State_t PX4CtrlFSM::get_hover_des()
{
    Desired_State_t des;
    des.p = hover_pose.head<3>();
    des.v = Eigen::Vector3d::Zero();
    des.a = Eigen::Vector3d::Zero();
    des.j = Eigen::Vector3d::Zero();
    des.yaw = hover_pose(3);
    des.yaw_rate = 0.0;
    return des;
}

Desired_State_t PX4CtrlFSM::get_cmd_des()
{
    Desired_State_t des;
    des.p = cmd_data.pos_sp;
    des.v = cmd_data.vel_sp;
    des.a = cmd_data.acc_sp;
    des.j = Eigen::Vector3d::Zero();
    des.yaw = cmd_data.yaw_sp;
    des.yaw_rate = cmd_data.yaw_rate_sp;
    return des;
}

bool PX4CtrlFSM::rc_is_received(const ros::Time &now_time)
{
    return (now_time - rc_data.header.stamp).toSec() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const ros::Time &now_time)
{
    return (now_time - cmd_data.header.stamp).toSec() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const ros::Time &now_time)
{
    return (now_time - odom_data.header.stamp).toSec() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const ros::Time &now_time)
{
    return (now_time - imu_data.header.stamp).toSec() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::bat_is_received(const ros::Time &now_time)
{
    return (now_time - bat_data.header.stamp).toSec() < param.msg_timeout.bat;
}

bool PX4CtrlFSM::recv_new_odom()
{
    static ros::Time last_time = ros::Time(0);
    bool ret = (odom_data.header.stamp != last_time);
    last_time = odom_data.header.stamp;
    return ret;
}   