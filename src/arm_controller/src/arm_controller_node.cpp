#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state) {
    ROS_INFO("Received JointState message");
    
    for (size_t i = 0; i < joint_state->position.size(); ++i) {
        ROS_INFO("Position of joint %zu: %f", i, joint_state->position[i]);
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "arm_controller_node");
    ros::NodeHandle nh;

    if (argc != 5) {
        ROS_ERROR("Please insert the value of the reference");
        return 1;
    }

    ros::Publisher jointPublisher1 = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J0_controller/command", 1);
    ros::Publisher jointPublisher2 = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J1_controller/command", 1);
    ros::Publisher jointPublisher3 = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J2_controller/command", 1);
    ros::Publisher jointPublisher4 = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J3_controller/command", 1);

    double joint1_position = std::stod(argv[1]);
    double joint2_position = std::stod(argv[2]);
    double joint3_position = std::stod(argv[3]);
    double joint4_position = std::stod(argv[4]);

    ros::Subscriber subscriber = nh.subscribe("arm/joint_states", 10, jointStateCallback);
    ros::Rate loopRate(10);

    while (ros::ok()) {
        std_msgs::Float64 jointCommand1, jointCommand2, jointCommand3, jointCommand4;
        jointCommand1.data = joint1_position;
        jointCommand2.data = joint2_position;
        jointCommand3.data = joint3_position;
        jointCommand4.data = joint4_position;

        ROS_INFO("Sending reference to joint 1: %.2f", jointCommand1.data);
        ROS_INFO("Sending reference to joint 2: %.2f", jointCommand2.data);
        ROS_INFO("Sending reference to joint 3: %.2f", jointCommand3.data);
        ROS_INFO("Sending reference to joint 4: %.2f", jointCommand4.data);

        jointPublisher1.publish(jointCommand1);
        jointPublisher2.publish(jointCommand2);
        jointPublisher3.publish(jointCommand3);
        jointPublisher4.publish(jointCommand4);

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
