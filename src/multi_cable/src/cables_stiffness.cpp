#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cmath>

/* Change Record：
    1. Multiple cables情景下，测试多根绳子的stiffness
 */ 

class DistanceCalculator
{
public:
    DistanceCalculator()
    {
        // Get the Cable_length and the Segment_length, also used for UnitVector
        start_pose_sub = nh.subscribe("/start_pos", 1000, &DistanceCalculator::startPosCallback, this);
        end_pose_sub = nh.subscribe("/end_pos", 1000, &DistanceCalculator::endPosCallback, this);
        start_link_sub = nh.subscribe("/start_link", 1000, &DistanceCalculator::startLinkCallback, this);

        // G_force for Stiffness_cable. Cylinder_end_force for Stiffness_segment -> F_spring
        G_sub = nh.subscribe("/cableEnd_ft_sensor", 1000, &DistanceCalculator::GCallback, this);
        force_sub = nh.subscribe("/prismatic_ft_sensor", 1000, &DistanceCalculator::forceCallback, this);
    }

    void startPosCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        start_pose = msg->pose.pose.position;
        start_vel = msg->twist.twist.linear;
        start_pose_set = true;
        tryInitializeCableLength(); // Attempt to initialize
    }

    void endPosCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        end_pose = msg->pose.pose.position;
        end_link = msg->pose.pose.position;
        end_vel = msg->twist.twist.linear;
        end_pose_set = true;
        tryInitializeCableLength(); // Attempt to initialize
    }

    void startLinkCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        start_link = msg->pose.pose.position;
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
    ros::Subscriber start_pose_sub, end_pose_sub, G_sub, force_sub, start_link_sub;
    geometry_msgs::Point start_pose, end_pose, start_link, end_link;
    geometry_msgs::Vector3 start_vel, end_vel;

    bool start_pose_set = false, end_pose_set = false, cable_initialized = false;
    double initial_cable_length = 0.0, current_length = 0.0;
    double G_x = 0.0, G_y = 0.0, G_z = 0.0;
    double force_x = 0.0, force_y = 0.0, force_z = 0.0;
    double vel_rel_x = 0.0, vel_rel_y = 0.0, vel_rel_z = 0.0;

    // Mannual Tune Params
    double segment_length = 0.1;
    double cable_stiffness = 10000;
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
        return std::sqrt(std::pow(pos1.x - pos2.x, 2) +
                         std::pow(pos1.y - pos2.y, 2) +
                         std::pow(pos1.z - pos2.z, 2)) + segment_length / 2;
    }

    double calculateDistance(const geometry_msgs::Point &pos1, const geometry_msgs::Point &pos2)
    {
        return std::sqrt(std::pow(pos1.x - pos2.x, 2) +
                         std::pow(pos1.y - pos2.y, 2) +
                         std::pow(pos1.z - pos2.z, 2));
    }


    void calculateDamping()
    {
        if (!cable_initialized)
            return; // If cable length hasn't been initialized, skip

        current_length = calculateLength(start_pose, end_pose);
        double elongation = current_length - initial_cable_length;
        ROS_INFO("Current cable elongation: %.5f, Initial length: %.5f", elongation, initial_cable_length);

        geometry_msgs::Point unit_vector_cable = calculateUnitVector(start_pose, end_pose);
        std::cout << "Unit_vector_cable recorded: ("
                    << unit_vector_cable.x << ", " << unit_vector_cable.y << ", " << unit_vector_cable.z << ")" << std::endl;

        if (elongation > 0)
        {   // Calculate damping when cable is taut\stretched
            // When cable is bended, The length MUST shorten than initial. -> UnitVector_Cable FOR STIFFNESS
            // geometry_msgs::Point unit_vector_cable = calculateUnitVector(start_pose, end_pose);
            // std::cout << "Unit_vector_cable recorded: ("
            //           << unit_vector_cable.x << ", " << unit_vector_cable.y << ", " << unit_vector_cable.z << ")" << std::endl;

            // Calculate the G projection onto the CABLE unit vector
            double projected_G = std::abs(
                                 G_x * unit_vector_cable.x +
                                 G_y * unit_vector_cable.y +
                                 G_z * unit_vector_cable.z );
            double stiffness = std::abs(projected_G / elongation);
            ROS_INFO("Projected G: %.5f, Elongation: %.5f, Calculated stiffness: %.5f",
                     projected_G, elongation, stiffness);
            
            // Calculate the total force projection onto the CABLE unit vector
            double projected_force = std::abs(
                                     force_x * unit_vector_cable.x +
                                     force_y * unit_vector_cable.y +
                                     force_z * unit_vector_cable.z );

            // double stiffness = std::abs(projected_force / elongation);
            // ROS_INFO("Projected Force: %.5f, Elongation: %.5f, Calculated stiffness: %.5f",
            //          projected_force, elongation, stiffness);

            // Calculate the SETTING spring FORCE in CABLE
            double spring_force = cable_stiffness * elongation;
            std::cout << "Segment elongation and Spring force recorded: ("
                      << elongation << ", " << spring_force << ")" << std::endl;

            // Get the RELATIVE vel onto the CABLE unit vector
            // end - star!!! 与unitVec_cable方向一致
            double vel_rel = std::abs(
                                     (end_vel.x - start_vel.x) * unit_vector_cable.x +
                                     (end_vel.y - start_vel.y) * unit_vector_cable.y +
                                     (end_vel.z - start_vel.z) * unit_vector_cable.z );

            std::cout << "Vel_relative recorded: ("
                    << end_vel.x - start_vel.x << ", " 
                    << end_vel.y - start_vel.y << ", "
                    << end_vel.z - start_vel.z << ")" << std::endl;

            double damping = std::abs( (projected_force - spring_force) / vel_rel);
            ROS_INFO("Total force: %.5f, Spring force: %.5f, Relative velocity: %.5f, Calculated damping: %.5f",
                     projected_force, spring_force, vel_rel, damping);
        }
        else
        {
            ROS_WARN("Cable is slack, stiffness cannot be calculated.");

            // Calculate the total force projection onto the CABLE unit vector
            double projected_force = std::abs(
                                     force_x * unit_vector_cable.x +
                                     force_y * unit_vector_cable.y +
                                     force_z * unit_vector_cable.z );
            // Calculate the SETTING spring FORCE
            // double spring_force = 0.0;

            // Get the RELATIVE vel onto the CABLE unit vector
            // end - star!!! 与unitVec_cable方向一致
            double vel_rel = std::abs(
                                     (end_vel.x - start_vel.x) * unit_vector_cable.x +
                                     (end_vel.y - start_vel.y) * unit_vector_cable.y +
                                     (end_vel.z - start_vel.z) * unit_vector_cable.z );

            double damping = std::abs( (projected_force - 0.0) / vel_rel);
            ROS_INFO("Total force: %.5f, prismatic velocity: %.5f, Calculated damping: %.5f",
                     projected_force, vel_rel, damping);
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
