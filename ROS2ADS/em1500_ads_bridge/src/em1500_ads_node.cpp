#include <iostream>
#include <string>
#include "AdsLib.h"
#include "AdsVariable.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <thread>
#include <chrono>

#include <Eigen/Dense>
#include <vector>
#include <cmath>

struct Pose {
    double surge;
    double sway;
    double heave;
    double roll;
    double pitch;
    double yaw;
};

std::vector<Eigen::Vector3d> p_AibC = {
    { 0.70416, -1.02102, 0.11258 },
    { 0.53215, -1.12033, 0.11258 },
    { -1.23631, -0.09931, 0.11258 },
    { -1.23631,  0.09931, 0.11258 },
    { 0.53215,   1.12033, 0.11258 },
    { 0.70416,   1.02102, 0.11258 }
};

std::vector<Eigen::Vector3d> p_BitC = {
    { 0.95000,  -0.09000, 0.0 },
    { -0.39706, -0.86772, 0.0 },
    { -0.55294, -0.77772, 0.0 },
    { -0.55294,  0.77772, 0.0 },
    { -0.39706,  0.86772, 0.0 },
    { 0.95000,   0.09000, 0.0 }
};

void ComputeLengths(double s, double w, double h,
                    double r, double p, double y,
                    double outL[6])
{
    double cr = cos(r), sr = sin(r);
    double cp = cos(p), sp = sin(p);
    double cy = cos(y), sy = sin(y);

    double r00 = cy * cp;
    double r01 = cy * sp * sr - sy * cr;
    double r02 = cy * sp * cr + sy * sr;

    double r10 = sy * cp;
    double r11 = sy * sp * sr + cy * cr;
    double r12 = sy * sp * cr - cy * sr;

    double r20 = -sp;
    double r21 =  cp * sr;
    double r22 =  cp * cr;

    Eigen::Vector3d p_tb(s, w, h);

    for (int i = 0; i < 6; i++)
    {
        Eigen::Vector3d t = p_BitC[i];

        Eigen::Vector3d Rt(
            r00 * t.x() + r01 * t.y() + r02 * t.z(),
            r10 * t.x() + r11 * t.y() + r12 * t.z(),
            r20 * t.x() + r21 * t.y() + r22 * t.z()
        );

        Eigen::Vector3d Pi = p_tb + Rt;
        Eigen::Vector3d diff = Pi - p_AibC[i];

        outL[i] = diff.norm();
    }
}

namespace craneads {

    struct AdsVariables
    {
        AdsVariables() = delete;

        explicit AdsVariables(AdsDevice& route)
            : pose{route, "MAIN.stEM1500Pose"}
        { }

        AdsVariable<Pose> pose;
    };

    class AdsHandler
    {
    public:
        explicit AdsHandler(const AmsNetId remoteNetId, const std::string remoteIpV4)
            : remoteNetId_(remoteNetId)
            , remoteIpV4_(remoteIpV4)
            , route_{remoteIpV4_, remoteNetId_, AMSPORT_R0_PLC_TC3}
            , ads_(route_) { }

        Pose getPose() {return ads_.pose;}

    private:
        const AmsNetId remoteNetId_;
        const std::string remoteIpV4_;
        AdsDevice route_;
        AdsVariables ads_;
    };
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ads_em1500_node");

    const AmsNetId remoteNetId { 169, 254, 119, 135, 1, 1 };
    const std::string remoteIpV4 = "192.168.1.15";

    craneads::AdsHandler adsHandler(remoteNetId, remoteIpV4);

    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "stewart/legs_em1500",
        10
    );

    bool ref_initialized = false;
    double L_ref[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        Pose p = adsHandler.getPose();

        double surge = -p.surge;
        double sway  = p.sway;
        double heave = p.heave + 1.205;

        double roll  = -p.roll;
        double pitch = p.pitch;
        double yaw   = p.yaw;

        double L[6];
        ComputeLengths(surge, sway, heave, roll, pitch, yaw, L);

        if (!ref_initialized)
        {
            for (int i = 0; i < 6; i++)
                L_ref[i] = L[i];
            ref_initialized = true;
        }

        double qi[6];
        for (int i = 0; i < 6; i++)
            qi[i] = L[i] - L_ref[i];

        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(6);
        for (int i = 0; i < 6; i++)
            msg.data[i] = qi[i];

        std::cout << "----- ROS2 ADS Bridge -----\n";
        std::cout << "Surge:  " << surge << "\n";
        std::cout << "Sway:   " << sway  << "\n";
        std::cout << "Heave:  " << heave << "   (raw PLC: " << p.heave << ")\n";
        std::cout << "Roll:   " << roll  << "\n";
        std::cout << "Pitch:  " << pitch << "\n";
        std::cout << "Yaw:    " << yaw   << "\n";

        std::cout << "Cylinder Stroke:\n";
        for (int i = 0; i < 6; i++)
            std::cout << "  qi" << i+1 << " = " << qi[i] << "\n";

        publisher->publish(msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
