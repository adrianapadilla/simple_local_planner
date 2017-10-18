
#ifndef SIMPLE_LOCAL_PLANNER_ROS_H_
#define SIMPLE_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// local planner specific classes which provide some macros
#include <base_local_planner/goal_functions.h>

// time
#include <time.h>

//files
#include <fstream>
#include <iostream>
using namespace std;

// msgs
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

// boost classes  TODO do I need?
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// other
#include <array>
#include <vector>

// definitions
#define PI 3.14159265
#define D2R 0.0174532925      // = 3.14159265/180

namespace simple_local_planner{

  /**
   * @class SimplePlannerROS
   * @brief Plugin to the ros base_local_planner. Implements a wrapper for the Elastic Band Method
   */

   struct pos {

	double x, y, az;	

   };


  class SimplePlannerROS : public nav_core::BaseLocalPlanner{

    public:
      /**
       * @brief Default constructor for the ros wrapper
       */
      SimplePlannerROS();

      /**
       * @brief Constructs the ros wrapper
       * @param name The name to give this instance of the elastic band local planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      SimplePlannerROS(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~SimplePlannerROS();

      /**
       * @brief Initializes the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Set the plan that the controller is following; also reset Simple-planner
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

    private:

      //Pointer to external objects (do NOT delete object)
      costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap  
      tf::TransformListener* tf_; ///<@brief pointer to Transform Listener 


      // Topics & Services
      ros::Subscriber amcl_sub; ///<@brief subscribes to the amcl topic
      ros::Publisher path_pub; ///<@brief publishes to the bubble shape to visualize on rviz 


      // Data
      pos now; // present frame
      pos next; // next frame
      pos nError; // error between present and next frames
      double distance;
      int length; // number of frames in the global plan 
      int count; // keeps track of the number for the next frame in the global plan
      std::vector<geometry_msgs::PoseStamped> plan; // contains the global plan
      geometry_msgs::Twist cmd; // contains the velocity
      visualization_msgs::Marker points;
      double average;
      int num;
      double average2, average3;
      int num2, howmanytimes;
      int p, hmt;
      double minus;
      double haha, haha2;
      double beforee;
      ofstream file;

     //measuring
      double stopTime, startTime;
      double beginning2, ending2;
      bool firstTime, hasStarted, number1;
      double pathLength;
      double four;

      // Flags
      bool goal_reached_;
      bool initialized_;

      // Velocity methods
      /**
      * @brief Set Vel: function that sets linear speed 
      */
      void setVel();

      /**
      * @brief Set Rot: function that sets angular speed 
      */
      void setRot();

      /**
      * @brief Set Vel Z: function that sets linear and angular speed to ZERO
      */
      void setVelZ();


      // Methods
      /**
       * @brief Amcl Callback: function is called whenever a new amcl msg is published on that topic 
       * @param Pointer to the received message
       */
      void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg);

      /**
      * @brief getYaw: function calculates the Yaw angle from the position message that amcl sent 
      * @param msg: passes the amcl position info to the function
      */
      double getYaw(geometry_msgs::PoseWithCovarianceStamped msg);

      /**
      * @brief setNowError: calculates the error between the next and present frame
      */
      void setNowError();

      /**
      * @brief getNext: uses count to set the next goal frame 
      */
      void setNext();

      void pathVisualization();

  };
};

#endif

