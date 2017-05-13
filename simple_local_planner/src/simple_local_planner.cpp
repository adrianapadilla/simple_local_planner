
#include "simple_local_planner/simple_local_planner.h"

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(simple_local_planner, SimplePlannerROS, simple_local_planner::SimplePlannerROS, nav_core::BaseLocalPlanner)

namespace simple_local_planner{

	SimplePlannerROS::SimplePlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

	SimplePlannerROS::SimplePlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
         : costmap_ros_(NULL), tf_(NULL), initialized_(false)
         {
		// initialize planner
		initialize(name, tf, costmap_ros);
         }

	SimplePlannerROS::~SimplePlannerROS() {}

	void SimplePlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
	{

		// check if the plugin is already initialized
		if(!initialized_)
		{
		// copy adress of costmap and Transform Listener (handed over from move_base)
		costmap_ros_ = costmap_ros;
		tf_ = tf;


		// subscribe to topics (to get odometry information, we need to get a handle to the topic in the global namespace)
		ros::NodeHandle gn;
		amcl_sub = gn.subscribe("amcl_pose", 100, &SimplePlannerROS::amclCallback, this);

		// set initialized flag
		initialized_ = true;

		// this is only here to make this process visible in the rxlogger right from the start
		ROS_DEBUG("Simple Local Planner plugin initialized.");
		}
		else
		{
		ROS_WARN("This planner has already been initialized, doing nothing.");
		}

	}

	bool SimplePlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
	{

		// check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

		//reset next counter
		count = 1; 

		//set plan, length and next goal
		plan = orig_global_plan; 
		length = (plan).size();  
		setNext(); 

		// set goal as not reached
		goal_reached_ = false;

		return true;

	}

	bool SimplePlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{

		// check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

		if(length != 0){ 

			setNowError();

			if(distance < 0.3){ 


				if(count<(length-1)){

					if((length - 1 - count) < 11){ 
						count = length - 1;
					}else{
						count += 10; 
					}
					setNext();
				}else{
	
					setVelZ();
					goal_reached_ = true;

				}


			}else{

				if(fabs(nError.az) > 25*D2R){

					setRot();

				}else{
	
					setVel();

				}

			}


		}

		// set retrieved commands to reference variable
		ROS_DEBUG("Retrieving velocity command: (%f, %f, %f)", cmd.linear.x, cmd.linear.y, cmd.angular.z);
		cmd_vel = cmd;  

		return true;

	}

	bool SimplePlannerROS::isGoalReached()
	{
		// check if plugin initialized
		if(!initialized_)
		{
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}

		// this info comes from compute velocity commands:
		return goal_reached_;

	}

	void SimplePlannerROS::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg)
	{

		ROS_INFO("Seq: [%d]", msg->header.seq);

		now.x = msg->pose.pose.position.x;
		now.y = msg->pose.pose.position.y;
		now.az = getYaw(*msg); 
		setNowError(); 

	}


	double SimplePlannerROS::getYaw(geometry_msgs::PoseWithCovarianceStamped msg)
	{

		double q[4];
		q[0]= msg.pose.pose.orientation.x;
		q[1]= msg.pose.pose.orientation.y;
		q[2]= msg.pose.pose.orientation.z;
		q[3]= msg.pose.pose.orientation.w;

		double t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
		double t4 = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);  

		return std::atan2(t3, t4);

	}

	void SimplePlannerROS::setVel()
	{

		// the output speed has been adjusted with a P regulator, that depends on how close we are to our current goal
		cmd.linear.x= distance;
	
		// keeping a small angular speed so that the movement is smooth //note that nError.az is small here
		cmd.angular.z= 0.75*(nError.az);

	}

	void SimplePlannerROS::setRot()
	{



		// the angular speed has been adjusted with a P regulator, that depends on how close we are to pointing at our current goal
		if (fabs(nError.az) > 50*D2R){

			cmd.angular.z=(nError.az)*0.3;

			// linear speed is zero while the angle is too big
			cmd.linear.x= 0.0;
			cmd.linear.y= 0.0;

		}else{

			cmd.angular.z=(nError.az)*0.5;

			// keeping a small linear speed so that the movement is smooth
			cmd.linear.x= 0.05;
			cmd.linear.y= 0.05;
		}

	}

	void SimplePlannerROS::setVelZ()
	{

		cmd.linear.x= 0;
		cmd.linear.y= 0;
		cmd.angular.z=0;

	}

	void SimplePlannerROS::setNext()
	{

		next.x = plan[count].pose.position.x;
		next.y = plan[count].pose.position.y;

	}

	void SimplePlannerROS::setNowError()
	{

		double d;

		nError.x = (next.x - now.x);
		nError.y = (next.y - now.y);

		// if these two variables are null, the tangent doesn't exist 
		// plus, the angle error is irrelevant because we have arrived at our destination

		if (nError.y == 0 & nError.x == 0){  
			d = now.az;
		}else{	
			d = std::atan2(nError.y, nError.x);
		}

		distance = std::sqrt(nError.x*nError.x +nError.y*nError.y);
		nError.az = d - now.az;

		// make sure that we chose the smallest angle, so that the robot takes the shortest turn
		if ( nError.az > 180*D2R ) { nError.az -= 360*D2R; }
		if ( nError.az < -180*D2R ) { nError.az += 360*D2R;  }

	}



}
