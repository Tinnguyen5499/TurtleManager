#ifndef MOCAPPUBLISHER_H
#define MOCAPPUBLISHER_H

#include <vector>
#include <NatNetTypes.h>

#include "rclcpp/rclcpp.hpp"
#include "mocap_optitrack_interfaces/msg/rigid_body_array.hpp"

#include <unordered_map>   // ← add

using namespace std;
using namespace std::chrono;
class MoCapPublisher: public rclcpp::Node
{
private:
    //Attributes
    rclcpp::Publisher<mocap_optitrack_interfaces::msg::RigidBodyArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    high_resolution_clock::time_point t_start;

    //Methods
    void sendFakeMessage();

    std::unordered_map<int, std::string> id_to_name_;
    

public:
    // Definition of the construtors
    MoCapPublisher();

    // Send methods
    void sendRigidBodyMessage(sRigidBodyData* bodies_ptr, int nRigidBodies);
 
    // Getters
    std::string getServerAddress();
    int getConnectionType();
    std::string getMulticastAddress();
    uint16_t getServerCommandPort();
    uint16_t getServerDataPort();
    
    // Setters
    void setIdToName(const std::unordered_map<int, std::string>& m) { id_to_name_ = m; }  // ← add
};
 
#endif