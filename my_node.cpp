#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <math.h>
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
float x_, y_,z_,x1_,y1_,z1_,husky_x,husky_y,husky_z,t=0;
void p_get(const trajectory_msgs::MultiDOFJointTrajectory& qd)
{
  if(qd.points.size()>0)
  {
      x_ = qd.points[0].transforms[0].translation.x;
      y_ = qd.points[0].transforms[0].translation.y;
      z_ = qd.points[0].transforms[0].translation.z;
        std::cout<<"test_done"<<std::endl;

  }
}
void q_get(const geometry_msgs::Pose& q)
{
  x1_=q.position.x;
  y1_=q.position.y;
  z1_=q.position.z;
  std::cout<<z1_<<std::endl;
}
void get_pos(const nav_msgs::Odometry& husky_p)
{
  husky_x=husky_p.pose.pose.position.x;
  husky_y=husky_p.pose.pose.position.y;
  husky_z=husky_p.pose.pose.position.z;
}
int main(int argc,char** argv)
{

    ros::init(argc,argv,"my_node");
    ros::NodeHandle nh;
    ros::Publisher traj_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 10);
    ros::Publisher polynomial_trajectory_pub_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("path_segments", 1);
    // ros::Publisher position_pub = nh.advertise<geometry_msgs::Vector>("/firefly/command/current_reference", 10);
    ros::Rate sleep_Rate(1);
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("command/current_reference",10,p_get);
    ros::Subscriber sub = nodeHandle.subscribe("ground_truth/pose",10,q_get);
    ros::Subscriber subscrbe = nodeHandle.subscribe("/odometry/filtered",10,get_pos);
    while(ros::ok())
    {

        ros::Time t0 = ros::Time::now();
        mav_trajectory_generation::Vertex::Vector vertices;
        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

        start.makeStartOrEnd(Eigen::Vector3d(x_,y_,z_), derivative_to_optimize);
        vertices.push_back(start);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(3,3,1));
        // vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(husky_x,husky_y,0.4));
        vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(husky_x,husky_y,0.25), derivative_to_optimize);
        vertices.push_back(end);

        std::vector<double> segment_times;
        const double v_max = 10;
        const double a_max = 10;
        segment_times = estimateSegmentTimes(vertices, v_max, a_max);

        const int N = 10;
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        mav_trajectory_generation::Segment::Vector segments;
        opt.getSegments(&segments);
        std::cout<<"seg_size"<<segments.size()<<std::endl;

        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);

        // double sampling_time = .1;
        // int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        // Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

        // double t_start = 2.0;
        // double t_end = 10.0;
        // double dt = 0.01;
        // std::vector<Eigen::VectorXd> result;
        // std::vector<double> sampling_times; // Optional.
        // trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);

        visualization_msgs::MarkerArray markers;
        double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";

        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
        mav_planning_msgs::PolynomialTrajectory4D msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,&msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        polynomial_trajectory_pub_.publish(msg);
        traj_pub.publish(markers);
        ROS_INFO("Take %f sec to get optimal traject", (ros::Time::now() - t0).toSec());





        vertices.clear();
        sleep_Rate.sleep();
        ros::spinOnce();
        if(z1_<0.33&&((husky_x-x_)<0.01)&& ((husky_y-y_)<0.01))
          break;
    }
    return 0;
}
