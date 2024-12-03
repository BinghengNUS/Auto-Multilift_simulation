#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cmath>

class DistanceCalculator
{
public:
    DistanceCalculator()
    {
        start_pose_sub = nh.subscribe("/start_pos", 100, &DistanceCalculator::startPosCallback, this);
        end_pose_sub = nh.subscribe("/end_pos", 100, &DistanceCalculator::endPosCallback, this);
        force_sub = nh.subscribe("/cableEnd_ft_sensor", 100, &DistanceCalculator::forceCallback, this);
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

    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
        force_z = msg->wrench.force.z;
        calculateStiffness(); // Calculate stiffness based on updated force
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber start_pose_sub, end_pose_sub, force_sub;
    geometry_msgs::Point start_pose, end_pose;
    bool start_pose_set = false, end_pose_set = false, cable_initialized = false;
    double initial_cable_length = 0.0, current_length = 0.0, force_z = 0.0;

    void tryInitializeCableLength()
    {
        // Initialize the cable length only if both base and load positions are available
        if (start_pose_set && end_pose_set && !cable_initialized)
        {
            std::cout << "Initial start_pose recorded: ("
                      << start_pose.x << ", " << start_pose.y << ", " << start_pose.z << ")" << std::endl;

            std::cout << "Initial end_pose recorded: ("
                      << end_pose.x << ", " << end_pose.y << ", " << end_pose.z << ")" << std::endl;

            // initial_cable_length = calculateDistance(start_pose, end_pose);
            initial_cable_length = 1.06000;
            cable_initialized = true; // Mark cable length as initialized
            ROS_INFO("Initial cable length recorded: %.5f", initial_cable_length);
        }
    }

    double calculateDistance(const geometry_msgs::Point &pos1, const geometry_msgs::Point &pos2)
    {
        return std::sqrt(std::pow(pos1.x - pos2.x, 2) +
                         std::pow(pos1.y - pos2.y, 2) +
                         std::pow(pos1.z - pos2.z, 2));
    }

    void calculateStiffness()
    {
        if (!cable_initialized)
            return; // If cable length hasn't been initialized, skip

        current_length = calculateDistance(start_pose, end_pose);
        double elongation = current_length - initial_cable_length;

        ROS_INFO("Current cable elongation: %.5f, Initial length: %.5f", elongation, initial_cable_length);

        if (elongation > 0)
        { // Calculate stiffness only if cable is stretched
            double stiffness = std::abs(force_z / elongation);
            ROS_INFO("Cable force: %.5f, Calculated stiffness: %.5f", std::abs(force_z), stiffness);
        }
        else
        {
            ROS_WARN("Cable is slack, stiffness cannot be calculated.");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_calculator");
    DistanceCalculator distanceCalculator;
    ros::spin();
    return 0;
}
