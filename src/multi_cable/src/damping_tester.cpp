#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cmath>
#include <fstream> // Include for file handling

class DistanceCalculator
{
public:
    DistanceCalculator() : file("/home/carlson/ros/multi_cable/multi_cable_data.csv")
    {
        // Add CSV header
        file << "Time, Elongation, Force along Cable\n";

        // Get the Cable_length and the Segment_length, also used for UnitVector
        start_pose_sub = nh.subscribe("/start_pos", 1000, &DistanceCalculator::startPosCallback, this);
        end_pose_sub = nh.subscribe("/end_pos", 1000, &DistanceCalculator::endPosCallback, this);
        start_link_sub = nh.subscribe("/start_link", 1000, &DistanceCalculator::startLinkCallback, this);

        // G_force for Stiffness_cable. Cylinder_end_force for Stiffness_segment -> F_spring
        G_sub = nh.subscribe("/cableEnd_ft_sensor", 1000, &DistanceCalculator::GCallback, this);
        force_sub = nh.subscribe("/prismatic_ft_sensor", 1000, &DistanceCalculator::forceCallback, this);
    }

    ~DistanceCalculator()
    {
        // Close the file when the object is destroyed
        file.close();
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
    }

    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
        force_x = msg->wrench.force.x;
        force_y = msg->wrench.force.y;
        force_z = msg->wrench.force.z;
        calculateDampingAndForce(); // Calculate damping and force along cable
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

    // File handling for CSV
    std::ofstream file;

    // Mannual Tune Params
    double segment_length = 0.1;
    // double cable_stiffness = 10000;

    void tryInitializeCableLength()
    {
        if (start_pose_set && end_pose_set && !cable_initialized)
        {
            initial_cable_length = calculateLength(start_pose, end_pose);
            cable_initialized = true;
            ROS_INFO("Initial cable length recorded: %.5f", initial_cable_length);
        }
    }

    double calculateLength(const geometry_msgs::Point &pos1, const geometry_msgs::Point &pos2)
    {
        geometry_msgs::Point unit_vector = calculateUnitVector(pos1, pos2);
        
        geometry_msgs::Point adjusted_pos2;
        adjusted_pos2.x = pos2.x + unit_vector.x * (segment_length / 2);
        adjusted_pos2.y = pos2.y + unit_vector.y * (segment_length / 2);
        adjusted_pos2.z = pos2.z + unit_vector.z * (segment_length / 2);
        
        // 重新计算调整后的位置与 pos1 之间的长度
        return std::sqrt(std::pow(pos1.x - adjusted_pos2.x, 2) +
                        std::pow(pos1.y - adjusted_pos2.y, 2) +
                        std::pow(pos1.z - adjusted_pos2.z, 2));
    }


    double calculateDistance(const geometry_msgs::Point &pos1, const geometry_msgs::Point &pos2)
    {
        return std::sqrt(std::pow(pos1.x - pos2.x, 2) +
                         std::pow(pos1.y - pos2.y, 2) +
                         std::pow(pos1.z - pos2.z, 2));
    }

    void calculateDampingAndForce()
    {
        if (!cable_initialized)
            return;

        current_length = calculateLength(start_pose, end_pose);
        double elongation = current_length - initial_cable_length;
        double simulation_time = ros::Time::now().toSec(); // Get current simulation time

        // Calculate unit vector along the cable
        geometry_msgs::Point unit_vector = calculateUnitVector(start_pose, end_pose);

        // Calculate the force projection along the cable
        double Project_G = force_z * unit_vector.z;

        double force_along_cable = force_z;

        double stiffness = std::abs(force_along_cable / elongation);

        // ROS and std::cout outputs remain unchanged
        ROS_INFO("Current cable elongation: %.5f, Initial length: %.5f", elongation, initial_cable_length);
        ROS_INFO("Force along the cable: %.5f", force_along_cable);
        ROS_INFO("Projected_G: %.5f", Project_G);
        ROS_INFO("Calculated stiffness: %.5f",stiffness);
        std::cout << "Force from sensor recorded: ("
                  << force_x << ", " << force_y << ", " << force_z << ")" << std::endl;
        std::cout << "Unit_vector_cable recorded: ("
                  << unit_vector.x << ", " << unit_vector.y << ", " << unit_vector.z << ")" << std::endl;

        

        // Save data to CSV
        file << simulation_time << ", " << elongation << ", " << force_along_cable << "\n";
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
