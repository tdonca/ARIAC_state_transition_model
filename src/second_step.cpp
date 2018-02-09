#include <vector>
#include <string>
#include <fstream>
#include <map>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
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
			
			//TODO: set up order service with callback
		}
		
		void run(ros::NodeHandle & node);
		
		bool plan(Transition & transition);
		
		bool execute(Transition & transition);
		
		//TODO:
		void orderReceived();
		
		//TODO:
		void processOrder();
		
		//TODO:
		void grabPart();
		
		//TODO:
		void placePart();
		
		//TODO:
		void finishOrder();
		
	private:
		
		bool loadObjectsFromStream(const std::string & in_file);
	
	
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
	
	//Create a robot model and state for Forward Kinematics
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	
	//Load scene objects into temporary planning scene from file
	if(!loadObjectsFromStream("scene/ariac.scene")){
		ROS_ERROR("Problem loading collision objects");
		return;
	}
	ROS_INFO("Successfully added collision objects to the world.");
	
	//Check for the added objects
	std::vector<std::string> objs = m_scene.getKnownObjectNames();
	for(int i = 0; i < objs.size(); ++i){
		ROS_INFO("Object %s", objs[i].c_str());
	}
	
	
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
	ros::Duration(5.0).sleep();
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


bool Monitor::loadObjectsFromStream(const std::string & in_file){
	//open stream
	std::ifstream fin;
	fin.open("/home/tudor/devROS/ARIAC/ariac_ws/ariac.scene");
	
	//load objects
	std::vector<moveit_msgs::CollisionObject> scene_objects;
	if(!fin.good()){
		ROS_ERROR("Error with opening.");
		return false;
	}
	else{
		std::string name;
		std::getline(fin, name);
		while(true){
			//look for item delimiter (* name)
			std::string marker;
			fin >> marker;
			if(!fin.good() || fin.eof()){
				ROS_ERROR("Error with stream.");
				return false;
			}
			if(marker == "*"){
				//create object in world frame
				moveit_msgs::CollisionObject obj;
				obj.header.frame_id = m_move_group.getPlanningFrame();
				obj.operation = obj.ADD;
				//get object name
				getline(fin, obj.id);
				boost::algorithm::trim(obj.id);
				//get number of shapes for the object
				unsigned int num_shapes;
				fin >> num_shapes;
				for(std::size_t i = 0; i < num_shapes && fin.good() && !fin.eof(); ++i){
					shape_msgs::SolidPrimitive obj_shape;
					//get shape type and dimensions
					std::string shp;
					fin >> shp;
					if(shp == "box"){
						obj_shape.type=obj_shape.BOX;
						obj_shape.dimensions.resize(3);
						fin >> obj_shape.dimensions[0] >> obj_shape.dimensions[1] >> obj_shape.dimensions[2];
					}
					else if(shp == "sphere"){
						obj_shape.type=obj_shape.SPHERE;
						obj_shape.dimensions.resize(1);
						fin >> obj_shape.dimensions[0];
					}
					else if(shp == "cylinder"){
						obj_shape.type=obj_shape.CYLINDER;
						obj_shape.dimensions.resize(2);
						fin >> obj_shape.dimensions[1] >> obj_shape.dimensions[0];
					}
					else if (shp == "cone"){
						obj_shape.type=obj_shape.CONE;
						obj_shape.dimensions.resize(2);
						fin >> obj_shape.dimensions[0] >> obj_shape.dimensions[1];
					}
					else{
						ROS_ERROR("Error with shape type.");
						return false;
					}
					//get pose
					geometry_msgs::Pose obj_pose;
					fin >> obj_pose.position.x >> obj_pose.position.y >> obj_pose.position.z;
					fin >> obj_pose.orientation.x >> obj_pose.orientation.y >> obj_pose.orientation.z >> obj_pose.orientation.w;
					//get color
					double r, g, b, a;
					fin >> r >> g >> b >> a;
					//do nothing with colors for now
					
					//add shape to collision object
					obj.primitives.push_back(obj_shape);
					obj.primitive_poses.push_back(obj_pose);
				}
				//add collision object to list
				scene_objects.push_back(obj);
			}
			else{
				ROS_INFO("No more * markers.");
				break;
			}
			
			
		}
		
	}
		
	//use PlanningSceneInterface to load objects
	if(m_scene.applyCollisionObjects(scene_objects)){
		return true;
	}
	else{
		ROS_ERROR("Error with applying objects.");
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
