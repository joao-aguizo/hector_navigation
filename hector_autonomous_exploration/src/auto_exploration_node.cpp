#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mbf_msgs/ExePathAction.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <hector_exploration_planner/hector_exploration_planner.h>

class AutoExploration
{

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
        nh.param<float>("replanTimerPeriod", replanTimerPeriod, 1.0);
        nh.param<int>("maxPlanRetries", maxPlanRetries, 5);
        nh.param<int>("maxControllerRetries", maxControllerRetries, 5);
        nh.param<std::string>("mbfController", mbfController, "TebLocalPlannerROS");
        nh.param<bool>("mbfToleranceFromAction", mbfToleranceFromAction, true);
        nh.param<float>("mbfDistTolerance", mbfDistTolerance, 1.0);
        nh.param<float>("mbfAngleTolerance", mbfAngleTolerance, 3.14);

        // action client for move_base_flex/exe_path 'sub-server'
        exePathActionClient = new ExePathActionClient("move_base_flex/exe_path", true);
        exePathActionClient->waitForServer();

        // the start and stop service to enable or disable exploration
        exploreSrvServer = nh.advertiseService("start_stop_exploration", &AutoExploration::exploreHandler, this);

        // configure replanning timer
        replanTimer = nh.createTimer(ros::Duration(replanTimerPeriod), &AutoExploration::replanCallback, this, false, true);

        ROS_INFO("Autonomous exploration node ready!");
    }

private:
    ros::NodeHandle nh;
    int planFailCounter = 0;
    int controllerFailCounter = 0;
    bool explorationEnabled = false;
    std::mutex mtx; // mutex for common flag

protected:
    ros::Timer replanTimer;
    ros::ServiceServer exploreSrvServer;
    ExePathActionClient *exePathActionClient;
    hector_exploration_planner::HectorExplorationPlanner *planner;
    costmap_2d::Costmap2DROS *costmap_2d_ros;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;

    int maxPlanRetries = 0;
    int maxControllerRetries = 0;
    bool mbfToleranceFromAction = false;
    float replanTimerPeriod = 0.0;
    float mbfDistTolerance = 0.0;
    float mbfAngleTolerance = 0.0;
    std::string mbfController = "";

    void replanCallback(const ros::TimerEvent &event)
    {
        if (getExplorationEnabled())
        {
            // get robot pose in costmap
            geometry_msgs::PoseStamped robot_pose;
            costmap_2d_ros->getRobotPose(robot_pose);

            // create the goal
            mbf_msgs::ExePathGoal goal;

            // call the planner and fill the goal
            if (!planner->doExploration(robot_pose, goal.path.poses))
            {
                // increment planning fail counter
                planFailCounter++;

                // check if plan failed more than the max allowed retries
                if (planFailCounter > maxPlanRetries)
                {
                    setExplorationEnabled(false);
                    ROS_WARN("Maximum planning retries exceeded! Exploration stopped.");
                }
                return;
            }

            // plan succeeded at least once, lets reset the plan fail counter
            planFailCounter = 0;

            goal.path.header.frame_id = costmap_2d_ros->getGlobalFrameID();
            goal.path.header.stamp = ros::Time::now();
            goal.controller = mbfController;
            goal.tolerance_from_action = mbfToleranceFromAction;
            goal.dist_tolerance = mbfDistTolerance;
            goal.angle_tolerance = mbfAngleTolerance;
            exePathActionClient->sendGoal(goal, boost::bind(&AutoExploration::goalDoneCallback, this, _1, _2));
        }
    }

    void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const mbf_msgs::ExePathResultConstPtr &result)
    {
        if (getExplorationEnabled())
        {

            // something is really wrong, abort!
            if (state == actionlib::SimpleClientGoalState::StateEnum::LOST)
            {
                setExplorationEnabled(false);
                return;
            }

            // check for controller failures and count them
            if (state == actionlib::SimpleClientGoalState::StateEnum::ABORTED)
            {
                controllerFailCounter++;

                // if controller fail counter exceeds maximum we return
                if (controllerFailCounter > maxControllerRetries)
                {
                    controllerFailCounter = 0;
                    setExplorationEnabled(false);
                    ROS_WARN("Maximum controller retries exceeded! Exploration stopped.");
                }
                else
                {
                    // check if controller failure was a non-recoverable or critical one
                    // http://docs.ros.org/en/kinetic/api/mbf_msgs/html/action/ExePath.html
                    switch (result->outcome)
                    {
                    case mbf_msgs::ExePathResult::FAILURE:
                    case mbf_msgs::ExePathResult::CANCELED:
                    case mbf_msgs::ExePathResult::NO_VALID_CMD:
                    case mbf_msgs::ExePathResult::COLLISION:
                    case mbf_msgs::ExePathResult::TF_ERROR:
                    case mbf_msgs::ExePathResult::NOT_INITIALIZED:
                    case mbf_msgs::ExePathResult::INVALID_PLUGIN:
                    case mbf_msgs::ExePathResult::INTERNAL_ERROR:
                    case mbf_msgs::ExePathResult::OUT_OF_MAP:
                    case mbf_msgs::ExePathResult::MAP_ERROR:
                    case mbf_msgs::ExePathResult::STOPPED:
                        ROS_WARN("Non-recoverable or critical 'exe_path' outcome. Aborting exploration...");
                        setExplorationEnabled(false);
                        break;
                    }
                }
            }
        }
    }

    bool exploreHandler(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        res.success = true;
        res.message = "Successfully defined state.";
        if (req.data == getExplorationEnabled())
        {
            res.message = "Already defined as requested!";
        }
        setExplorationEnabled(req.data);
        return true;
    }

    bool getExplorationEnabled()
    {
        std::lock_guard<std::mutex> locker(mtx);
        return explorationEnabled;
    }

    void setExplorationEnabled(bool value)
    {
        std::lock_guard<std::mutex> locker(mtx);
        explorationEnabled = value;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    AutoExploration ae;

    ros::spin();

    return 0;
}
