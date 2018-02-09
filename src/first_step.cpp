#include <vector>
#include <string>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

//remove unnecessary includes

 

class State {
	public:
		explicit State(geometry_msgs::Pose ee_pose, std::vector<double> joint_values)
		:	m_ee_pose(ee_pose),
			m_joint_values(joint_values)
		{
			
		}
		
		geometry_msgs::Pose getPose();
		
		//TODO copy and move constructor
	
	private:
		geometry_msgs::Pose 	m_ee_pose;
		std::vector<double> 	m_joint_values;

};

geometry_msgs::Pose State::getPose(){
	return m_ee_pose;
}


/***********************************************************************/
class Transition {
	public:
		Transition(State & init_state, State & goal_state)
		:	m_init_state(init_state),
			m_goal_state(goal_state),
			m_plan()
		{
			
		}
		
		bool updatePlan(moveit::planning_interface::MoveGroupInterface::Plan & new_plan);
		
		geometry_msgs::Pose getStartPose();
		
		geometry_msgs::Pose getGoalPose();
		
		const moveit::planning_interface::MoveGroupInterface::Plan & getPlan();
		
	private:
		State m_init_state;
		State m_goal_state;
		moveit::planning_interface::MoveGroupInterface::Plan m_plan;
		
};


bool Transition::updatePlan(moveit::planning_interface::MoveGroupInterface::Plan & new_plan){
	m_plan.trajectory_ = new_plan.trajectory_;
	m_plan.start_state_ = new_plan.start_state_;
}

geometry_msgs::Pose Transition::getStartPose(){
	return m_init_state.getPose();
}
		
geometry_msgs::Pose Transition::getGoalPose(){
	return m_goal_state.getPose();
}

const moveit::planning_interface::MoveGroupInterface::Plan & Transition::getPlan(){
	return m_plan;
}


/***********************************************************************/
class Monitor {
	public:
		Monitor(std::string planning_group)
		:	m_move_group(planning_group), 
			m_scene(),
			m_current_state(m_move_group.getCurrentPose().pose, m_move_group.getCurrentJointValues())
		{
			//TODO: import scene objects for the planner
		}
		
		void run(ros::NodeHandle & node);
		
		bool plan(Transition & transition);
		
		bool execute(Transition & transition);
		
		
	private:
		moveit::planning_interface::MoveGroupInterface m_move_group;
		moveit::planning_interface::PlanningSceneInterface m_scene;
		State m_current_state;
};


/*
 * Main processing function for handling the entire competition
 * Handles planning, execution, and task assignment
 * 
 * */
void Monitor::run(ros::NodeHandle & node){
	//scene is now set up with collision objects
	
	//Create a robot model and state for Forward Kinematics
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	
	
	//Create Pose from joint states
	//vector<double> j_v_d = {0.00, 3.14, -1.12, 1.51, 3.77, -1.51, 0.00};
	std::vector<double> j_v_d = m_move_group.getCurrentJointValues();
	kinematic_state->setJointGroupPositions("manipulator", j_v_d);
	const Eigen::Affine3d &ee_state_default = kinematic_state->getGlobalLinkTransform("ee_link");
	//Create a Default state
	geometry_msgs::Pose d = Eigen::toMsg(ee_state_default);
	State defaultState(d, j_v_d);
	
	//Create pose from joint states
	std::vector<double> j_v_p = {0.20, 3.14, -0.40, 2.45, 2.70, -1.51, 0.00};
	kinematic_state->setJointGroupPositions("manipulator", j_v_p);
	const Eigen::Affine3d &ee_state_part = kinematic_state->getGlobalLinkTransform("ee_link");
	//Create a At Part state
	geometry_msgs::Pose p = Eigen::toMsg(ee_state_part);
	State atPartState(p, j_v_p);
	
	
	//Create a transition from Default state to At Part state
	Transition t_default_to_part(defaultState, atPartState);
	bool success = plan(t_default_to_part);
	
	
	//execute plan if it successful
	if(success){
		execute(t_default_to_part);
	}
	
	
	ROS_INFO("All done.");
}



bool Monitor::plan(Transition & transition){
	
	//Set start and end waypoints, the cartesian path computes the interpolated positions between waypoints
	std::vector<geometry_msgs::Pose> waypoints;
	//waypoints.push_back(transition.getStartPose());
	waypoints.push_back(transition.getGoalPose());
	
	//Compute the cartesian trajectory with the waypoints
	moveit_msgs::RobotTrajectory trajectory;
	double eef_step = 0.01;
	double jump_threshold = 0.0;
	double amt = m_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	
	//Check for successful plan
	if(amt == -1){
		ROS_ERROR("Error during Cartesian Path Planning");
		return false;
	}
	else if (amt != 1.0){
		ROS_ERROR("Failed to find full Cartesian path, only %f of path found", amt);
		return false;
	}
	else{
		ROS_INFO("Full Cartesian Path found.");
		
		//Save the plan in the transition
		moveit::planning_interface::MoveGroupInterface::Plan new_plan;
		new_plan.trajectory_ = trajectory;
		robotStateToRobotStateMsg(*(m_move_group.getCurrentState()), new_plan.start_state_);
		transition.updatePlan(new_plan);
		return true;
	}
	
}

bool Monitor::execute(Transition & transition){
	
	moveit::planning_interface::MoveItErrorCode success;
	success = m_move_group.execute(transition.getPlan());
	if(success == moveit_msgs::MoveItErrorCodes::SUCCESS){
		ROS_INFO("Transition plan was successfully executed.");
		return true;
	}
	else{
		ROS_ERROR("Error during plan execution!");
		return false;
	}
}






/***********************************************************************/
int main(int argc, char* argv[]){
	
	/* Node setup */
	ros::init(argc, argv, "first_step");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
	
	/* Setup and start high level system */
	Monitor monitor("manipulator");
	monitor.run(node_handle);
	
	
	/* Node cleanup */
	ros::shutdown();
	return 0;
}
