#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cmath>

/* Change Record：
    1. Initial version: Wrong Force Analysis
    2. s9_c2 相对 s9_c1读取速度， 在Segment上算damping
 */ 

class DistanceCalculator
{
public:
    DistanceCalculator()
    {
        // Get the Cable_length and the Segment_length, also used for UnitVector
        start_pose_sub = nh.subscribe("/start_pos", 100, &DistanceCalculator::startPosCallback, this);
        end_pose_sub = nh.subscribe("/end_pos", 100, &DistanceCalculator::endPosCallback, this);
        start_link_sub = nh.subscribe("/start_link", 100, &DistanceCalculator::startLinkCallback, this);
        end_link_sub = nh.subscribe("/end_link", 100, &DistanceCalculator::endLinkCallback, this);

        // G_force for Stiffness_cable. Cylinder_end_force for Stiffness_segment -> F_spring
        G_sub = nh.subscribe("/cableEnd_ft_sensor", 100, &DistanceCalculator::GCallback, this);
        force_sub = nh.subscribe("/prismatic_ft_sensor", 100, &DistanceCalculator::forceCallback, this);

        // Get the Stiffness_set of each joint and Segment count -> Stiffness_cable

        // Get the vel_relative (s_end_c2 to s_end_c1) and VERIFY the damping
        vel_rel_sub = nh.subscribe("/vel_rel", 100, &DistanceCalculator::velRelCallback, this);
        // pris_joint_sub = nh.subscribe("/pris_joint", 100, &DistanceCalculator::prisJointCallback, this);
    }

    void startPosCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        start_pose = msg->pose.pose.position;
        start_pose_set = true;
        tryInitializeCableLength(); // Attempt to initialize
    }

    void endPosCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        end_pose = msg->pose.pose.position;
        end_pose_set = true;
        tryInitializeCableLength(); // Attempt to initialize
    }

    void startLinkCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        start_link = msg->pose.pose.position;
    }

    void endLinkCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        end_link = msg->pose.pose.position;
    }

    void velRelCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        vel_rel_x = msg->twist.twist.linear.x;
        vel_rel_y = msg->twist.twist.linear.y;
        vel_rel_z = msg->twist.twist.linear.z;
    }

    void GCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
        G_x = msg->wrench.force.x;
        G_y = msg->wrench.force.y;
        G_z = msg->wrench.force.z;
        // calculateStiffness(); // Calculate stiffness based on updated force
    }

    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
        force_x = msg->wrench.force.x;
        force_y = msg->wrench.force.y;
        force_z = msg->wrench.force.z;
        calculateDamping(); // Calculate damping based on updated force
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber start_pose_sub, end_pose_sub, G_sub, force_sub, start_link_sub, end_link_sub, vel_rel_sub;
    geometry_msgs::Point start_pose, end_pose, start_link, end_link, pris_vel;
    bool start_pose_set = false, end_pose_set = false, cable_initialized = false;

    double initial_cable_length = 0.0, current_length = 0.0, current_segment = 0.0;
    double G_x = 0.0, G_y = 0.0, G_z = 0.0;
    double force_x = 0.0, force_y = 0.0, force_z = 0.0;
    double vel_rel_x = 0.0, vel_rel_y = 0.0, vel_rel_z = 0.0;

    // Mannual Tune Params
    double segment_length = 0.1;
    double joint_stiffness = 50000;
    // double segment_count = 10;

    void tryInitializeCableLength()
    {
        // Initialize the cable length only if both base and load positions are available
        if (start_pose_set && end_pose_set && !cable_initialized)
        {
            std::cout << "Initial start_pose recorded: ("
                      << start_pose.x << ", " << start_pose.y << ", " << start_pose.z << ")" << std::endl;

            std::cout << "Initial end_pose recorded: ("
                      << end_pose.x << ", " << end_pose.y << ", " << end_pose.z << ")" << std::endl;

            initial_cable_length = calculateLength(start_pose, end_pose);
            cable_initialized = true; // Mark cable length as initialized
            ROS_INFO("Initial cable length recorded: %.5f", initial_cable_length);
        }
    }

    double calculateLength(const geometry_msgs::Point &pos1, const geometry_msgs::Point &pos2)
    {
        // double segment_length = 0.1;
        return std::sqrt(std::pow(pos1.x - pos2.x, 2) +
                         std::pow(pos1.y - pos2.y, 2) +
                         std::pow(pos1.z - pos2.z, 2)) + segment_length / 2;
    }

    double calculateDistance(const geometry_msgs::Point &pos1, const geometry_msgs::Point &pos2)
    {
        // double segment_length = 0.1;
        return std::sqrt(std::pow(pos1.x - pos2.x, 2) +
                         std::pow(pos1.y - pos2.y, 2) +
                         std::pow(pos1.z - pos2.z, 2));
    }

    // void calculateStiffness()
    // {
    //     if (!cable_initialized)
    //         return; // If cable length hasn't been initialized, skip

    //     current_length = calculateLength(start_pose, end_pose);
    //     double elongation = current_length - initial_cable_length;
    //     ROS_INFO("Current cable elongation: %.5f, Initial length: %.5f", elongation, initial_cable_length);

    //     if (elongation > 0)
    //     { // Calculate stiffness only if cable is taut\stretched
    //       // When cable is bended, The length MUST shorten than initial. -> UnitVector useless
    //         geometry_msgs::Point unit_vector_cable = calculateUnitVector(start_pose, end_pose);

    //         // Calculate the G projection onto the CABLE unit vector
    //         double projected_G = G_x * unit_vector_cable.x +
    //                              G_y * unit_vector_cable.y +
    //                              G_z * unit_vector_cable.z;

    //         double stiffness = std::abs(projected_G / elongation);
    //         ROS_INFO("Projected G: %.5f, Elongation: %.5f, Calculated stiffness: %.5f",
    //                  projected_G, elongation, stiffness);
    //     }
    //     else
    //     {
    //         ROS_WARN("Cable is slack, stiffness cannot be calculated.");
    //     }
    // }

    void calculateDamping()
    {
        if (!cable_initialized)
            return; // If cable length hasn't been initialized, skip

        current_length = calculateLength(start_pose, end_pose);
        double elongation = current_length - initial_cable_length;
        ROS_INFO("Current cable elongation: %.5f, Initial length: %.5f", elongation, initial_cable_length);

        geometry_msgs::Point unit_vector_segment = calculateUnitVector(start_link, end_link);
        std::cout << "Unit_vector_segment recorded: ("
                      << unit_vector_segment.x << ", " << unit_vector_segment.y << ", " << unit_vector_segment.z << ")" << std::endl;


        if (elongation > 0)
        {   // Calculate damping when cable is taut\stretched
            // When cable is bended, The length MUST shorten than initial. -> UnitVector_Cable FOR STIFFNESS
            geometry_msgs::Point unit_vector_cable = calculateUnitVector(start_pose, end_pose);
            std::cout << "Unit_vector_cable recorded: ("
                      << unit_vector_cable.x << ", " << unit_vector_cable.y << ", " << unit_vector_cable.z << ")" << std::endl;


            // Calculate the G projection onto the CABLE unit vector
            double projected_G = G_x * unit_vector_cable.x +
                                 G_y * unit_vector_cable.y +
                                 G_z * unit_vector_cable.z;

            double stiffness = std::abs(projected_G / elongation);
            ROS_INFO("Projected G: %.5f, Elongation: %.5f, Calculated stiffness: %.5f",
                     projected_G, elongation, stiffness);
            
            // Calculate the G projection onto the SEGMENT unit vector
            double projected_G_seg = std::abs(
                                     G_x * unit_vector_segment.x +
                                     G_y * unit_vector_segment.y +
                                     G_z * unit_vector_segment.z );
            // Calculate the total force projection onto the SEGMENT unit vector
            double projected_force = std::abs(
                                     force_x * unit_vector_segment.x +
                                     force_y * unit_vector_segment.y +
                                     force_z * unit_vector_segment.z );
            // Calculate the SETTING spring FORCE in SEGMENT
            current_segment = calculateLength(start_link, end_link);
            double segment_elogation = current_segment - segment_length;
            double spring_force = joint_stiffness * segment_elogation;
            std::cout << "Segment elongation and Spring force recorded: ("
                      << segment_elogation << ", " << spring_force << ")" << std::endl;

            // Get the RELATIVE vel onto the SEGMENT unit vector
            double vel_rel = std::abs(
                                     vel_rel_x * unit_vector_segment.x +
                                     vel_rel_y * unit_vector_segment.y +
                                     vel_rel_z * unit_vector_segment.z );

            // double damping = std::abs(projected_force - projected_G_seg - spring_force / vel_rel);
            double damping = std::abs( (projected_force - spring_force) / vel_rel);
            ROS_INFO("Total force: %.5f, G force: %.5f, Spring force: %.5f, Relative velocity: %.5f, Calculated damping: %.5f",
                     projected_force, projected_G_seg, spring_force, vel_rel, damping);
        }
        else
        {
            ROS_WARN("Cable is slack, stiffness cannot be calculated.");

            // Calculate the G projection onto the SEGMENT unit vector
            double projected_G_seg = std::abs(
                                     G_x * unit_vector_segment.x +
                                     G_y * unit_vector_segment.y +
                                     G_z * unit_vector_segment.z );
            // Calculate the total force projection onto the SEGMENT unit vector
            double projected_force = std::abs(
                                     force_x * unit_vector_segment.x +
                                     force_y * unit_vector_segment.y +
                                     force_z * unit_vector_segment.z );
            // Calculate the SETTING spring FORCE
            // double spring_force = 0.0;

            // Get the RELATIVE vel onto the SEGMENT unit vector
            double vel_rel = std::abs(
                                     vel_rel_x * unit_vector_segment.x +
                                     vel_rel_y * unit_vector_segment.y +
                                     vel_rel_z * unit_vector_segment.z );

            double damping = std::abs( (projected_force - 0.0) / vel_rel);
            ROS_INFO("Total force: %.5f, G force: %.5f, prismatic velocity: %.5f, Calculated damping: %.5f",
                     projected_force, projected_G_seg, vel_rel, damping);
        }
    }

    geometry_msgs::Point calculateUnitVector(const geometry_msgs::Point &pos1, const geometry_msgs::Point &pos2)
    {
        geometry_msgs::Point unit_vector;
        
        double distance = calculateDistance(pos1, pos2);

        if (distance > 0.0)
        {
            unit_vector.x = (pos2.x - pos1.x) / distance;
            unit_vector.y = (pos2.y - pos1.y) / distance;
            unit_vector.z = (pos2.z - pos1.z) / distance;
        }
        return unit_vector;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_calculator");
    DistanceCalculator distanceCalculator;
    ros::spin();
    return 0;
}
