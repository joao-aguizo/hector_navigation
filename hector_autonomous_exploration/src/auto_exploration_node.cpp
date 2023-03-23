#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mbf_msgs/ExePathAction.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <hector_exploration_planner/hector_exploration_planner.h>


class AutoExploration {

typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> ExePathActionClient;

public:
    AutoExploration() : listener(buffer)
    {
        // initialize the planner
        buffer.setUsingDedicatedThread(true);
        costmap_2d_ros = new costmap_2d::Costmap2DROS("global_costmap", buffer);
        planner = new hector_exploration_planner::HectorExplorationPlanner();
        planner->initialize("hector_exploration_planner", costmap_2d_ros);

        // get parameters
        nh.param<float>("goalTimerPeriod", goalTimerPeriod, 15.0);
        nh.param<std::string>("mbfController", mbfController, "TebLocalPlannerROS");

        // action client for move_base_flex/exe_path 'sub-server'
        exePathActionClient = new ExePathActionClient("move_base_flex/exe_path", true);
        exePathActionClient->waitForServer();

        // the start and stop service to enable or disable exploration
        exploreSrvServer = nh.advertiseService("start_stop_exploration", &AutoExploration::exploreHandler, this);

        // configure the time
        goalTimer = nh.createTimer(ros::Duration(goalTimerPeriod), &AutoExploration::goalCallback, this, false, false);

        ROS_INFO("Autonomous exploration node ready!");
    }

protected:
    ros::NodeHandle nh;

    ros::Timer goalTimer;
    ros::ServiceServer exploreSrvServer;
    ExePathActionClient* exePathActionClient;
    hector_exploration_planner::HectorExplorationPlanner* planner;
    costmap_2d::Costmap2DROS* costmap_2d_ros;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    
    bool explorationEnabled;
    float goalTimerPeriod;
    std::string mbfController;

    void goalCallback(const ros::TimerEvent& event){
        if (explorationEnabled){
            // check if goal is not active TODO!
            
            // call planner
            geometry_msgs::PoseStamped robot_pose;
            costmap_2d_ros->getRobotPose(robot_pose);

            // create the goal
            mbf_msgs::ExePathGoal goal;

            // call the planner and fill the goal
            planner->doExploration(robot_pose, goal.path.poses);
            goal.path.header.frame_id = "map";
            goal.path.header.stamp = ros::Time::now();
            goal.controller = mbfController;
            exePathActionClient->sendGoal(goal);
        }
    }

    bool exploreHandler(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
        res.success = true;
        res.message = "Successfully defined state.";
        if (req.data == explorationEnabled){
            res.message = "Already defined as requested!";
        }
        explorationEnabled = req.data;
        return true;
    }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  AutoExploration ae;

  ros::spin();

  return 0;
}
