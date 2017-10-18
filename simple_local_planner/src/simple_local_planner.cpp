
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

		beginning2 = ros::Time::now().toSec();

		// copy adress of costmap and Transform Listener (handed over from move_base)
		costmap_ros_ = costmap_ros;
		tf_ = tf;


		// subscribe to topics (to get odometry information, we need to get a handle to the topic in the global namespace)
		ros::NodeHandle gn;
		amcl_sub = gn.subscribe("amcl_pose", 100, &SimplePlannerROS::amclCallback, this);
        	path_pub = gn.advertise<visualization_msgs::Marker>("visualization_marker", 10);

		//initializing the visualization markers
		points.header.frame_id = "/map";
	 
		points.header.stamp = ros::Time::now();
	 
		points.ns = "path_drawing";
	 
		points.action = visualization_msgs::Marker::ADD;
	 
		points.id = 0;
	 
		//points.pose.position.x = now.x;
		//points.pose.position.y = now.y;
		//points.pose.position.z = 0.5;
	 
		points.type = visualization_msgs::Marker::POINTS;
	 
		points.scale.x = 0.1;
		points.scale.y = 0.1;
	 
		points.color.g = 1.0f;
		points.color.a = 1.0;

		average = 0;
		num = 0;
		firstTime = 1;
		number1 = 1;
		hasStarted = 0;
                pathLength = 0;
		p = 0;
		minus = 0;
		//beforee = ros::Time::now().toSec();

		//open file
		file.open("/home/adriana/adrianaTFG/simple_cs/simple_cs_10.txt", ios::out);
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

		ending2 = ros::Time::now().toSec();
		//ROS_INFO("between: %f", (ros::Time::now().toSec()-beforee)/2);
		//average += (ros::Time::now().toSec()-beforee);
		//num++;

		if(number1){
			four = ros::Time::now().toSec()-beginning2;
			number1 = 0;
		}
		ROS_INFO("minus: %f", (minus));
		ROS_INFO("four: %f", (four));
		ROS_INFO("between2: %f", (ending2-beginning2-minus-four));
		average += 0.05;//(ending2-beginning2-minus-four);
		//num++;

		p++;
		minus = p;//0.2*p;

		//beforee = ros::Time::now().toSec(); 

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

			if(firstTime){
				startTime = ros::Time::now().toSec();
				firstTime = 0;
			}

			double beginning = ros::Time::now().toSec();

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

					stopTime = ros::Time::now().toSec();
					
					//time to reach goal
					ROS_INFO("journey duration: %f", (stopTime-startTime));

					//path length
					ROS_INFO("path length: %f", pathLength);

					//average executation time (for computational cost)
					ROS_INFO("avrg exec time: %f", average/num);

					if(file.is_open()){
						ROS_INFO("I'm OPEN!");
						//file.close();
					}else{
						ROS_INFO("I'm NOT open!");
					}

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


			double ending = ros::Time::now().toSec();
			average += (ending-beginning);
			num++;
			//ROS_INFO("avrg exec time: %f", average/num);
		}

		// set retrieved commands to reference variable
		ROS_DEBUG("Retrieving velocity command: (%f, %f, %f)", cmd.linear.x, cmd.linear.y, cmd.angular.z);
		file << cmd.linear.x << " "<< cmd.linear.y << " "<< cmd.angular.z << endl;
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

		pos before;
		if(hasStarted){
			before.x = now.x;
			before.y = now.y;
		}

		now.x = msg->pose.pose.position.x;
		now.y = msg->pose.pose.position.y;
		now.az = getYaw(*msg); 
		setNowError();

		if(hasStarted){
			pathLength += std::sqrt((now.x-before.x)*(now.x-before.x) + (now.y-before.y)*(now.y-before.y));
			ROS_INFO("%f, %f, %f", stopTime, startTime, pathLength);
		}

		pathVisualization();
		
		hasStarted = 1;

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

	void SimplePlannerROS::pathVisualization()
	{
	 
	        geometry_msgs::Point p;
 
	        p.x = now.x;
	        p.y = now.y;
	        p.z = 0.5;
 
      	        points.points.push_back(p);
   
	     
		path_pub.publish(points); 
	 
	 }



}




/*	void SimplePlannerROS::bubbleVisualization()
	{
		/////// Visualization in rviz
	 
		visualization_msgs::Marker points, line_strip;
	 
		points.header.frame_id = "/draw_bubble";
		line_strip.header.frame_id = "/draw_bubble";
	 
		points.header.stamp = ros::Time::now();
		line_strip.header.stamp = ros::Time::now();
	 
		points.ns = "bubble_drawing";
		line_strip.ns = "bubble_drawing";
	 
		points.action = visualization_msgs::Marker::ADD;
		line_strip.action = visualization_msgs::Marker::ADD;
	 
		points.id = 0;
		line_strip.id = 1;
	 
		points.pose = poseNow;
		line_strip.pose = poseNow;
	 
		points.type = visualization_msgs::Marker::POINTS;
		line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	 
		//size of dots and lines
		points.scale.x = 0.2;
		points.scale.y = 0.2;
		line_strip.scale.x = 0.1;
	 
		points.color.b = 1.0f;
		points.color.a = 1.0;
	 
		line_strip.color.b = 1.0f;
		line_strip.color.a = 1.0;
	 
		for(int j = 0; j<180; j++){
	 
		    geometry_msgs::Point p;
	 
		    p.x = std::cos(j*D2R)*bubble[j];
		    p.y = std::sin(j*D2R)*bubble[j];
		    p.z = 0;
	 
		    points.points[j] = p;
		    line_strip.points[j] = p;
	 
		} 
	     
		bubble_pub.publish(points); 
		bubble_pub.publish(line_strip);
	 
	 }
*/
