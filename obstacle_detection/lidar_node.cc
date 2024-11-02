/*
 * Obstacle detection and avoidance
 */

#include <stdio.h>
#include <stdlib.h>
#include <queue>

#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/transport/Node.hh>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

bool m_debug = false;

std::string topic_pub = "/cmd_vel";
ignition::transport::Node m_node;
auto m_twistPub = m_node.Advertise<ignition::msgs::Twist>(topic_pub);

std::mutex m_isObstacleMutex;
bool m_isObstacle = false;
int m_stepCount = 0;

//std::mutex m_isStoppedMutex;
bool m_isStopped = true;

typedef std::pair<double, double> Coord2D;
std::queue<Coord2D> m_goals;

typedef ignition::msgs::Vector3d Vector3d;
Vector3d m_position;
Vector3d m_goal;

// Max steps to take after obstacle is no longer detected.
int MAX_STEPS = 5;

/*
Sample odometry

pose {
  position {
    x: 6.4591636705915088
    y: 5.2835379170630251e-13
  }
  orientation {
    z: 0.26600853338802949
    w: 0.96397067391324187
  }
}
twist {
  linear {
    x: 2.1316282072803006e-12
  }
  angular {
    z: 0.49999999964830028
  }
}
*/

void printPose(ignition::msgs::Pose &a_pose)
{
  ignition::msgs::Quaternion orientation = a_pose.orientation();
  ignition::msgs::Vector3d position = a_pose.position();
  std::cout << "pose:" << std::endl;
  std::cout << "\tposition: "
            << position.x() << ", "
            << position.y() << ", "
            << position.z()
            << std::endl;
  std::cout << "\torientation: "
            << orientation.w() << ", "
            << orientation.x() << ", "
            << orientation.y() << ", "
            << orientation.z()
            << std::endl;
}

void printTwist(ignition::msgs::Twist &a_twist)
{
  ignition::msgs::Vector3d angular = a_twist.angular();
  ignition::msgs::Vector3d linear = a_twist.linear();
  std::cout << "twist:" << std::endl;
  std::cout << "\tlinear: "
            << linear.x() << ", "
            << linear.y() << ", "
            << linear.z()
            << std::endl;
  std::cout << "\tangular: "
            << angular.x() << ", "
            << angular.y() << ", "
            << angular.z()
            << std::endl;
}

//======================================================================
//
//======================================================================
bool isObstacle()
{
  std::scoped_lock lock(m_isObstacleMutex);
  return m_isObstacle;
}

void setIsObstacle(bool a_obstacle)
{
  std::scoped_lock lock(m_isObstacleMutex);
  m_isObstacle = a_obstacle;
}

//======================================================================
//
//======================================================================
bool isStopped()
{
  //std::scoped_lock lock(m_isStoppedMutex);
  return m_isStopped;
}

void setIsStopped(bool a_stop)
{
  //std::scoped_lock lock(m_isStoppedMutex);
  m_isStopped = a_stop;
}

//======================================================================
// Function called each time lidar topic update is received.
//======================================================================
void cbLidar(const ignition::msgs::LaserScan &a_scan)
{
  ignition::msgs::Twist data;

  bool isShortRange = false;
  int i = 0;
  for (; i < a_scan.ranges_size(); i++)
  {
    if (a_scan.ranges(i) < 1.0)
    {
      isShortRange = true;
      break;
    }
  }

  // IF we have a short range, assume it's an obstacle
  // Once we no longer detect a short range, lets move the vehicle a short
  // distance to avoid "seeing" the obstacle again.
  // After we have moved this short distance, clear the obstacle detected flag.
  if (isShortRange)
  {
    setIsObstacle(true);
    m_stepCount = 0;

    // IF our first detection is on the right side of the scan, rotate left.
    // else rotate right.
    bool spinLeft = (i < (a_scan.ranges_size() / 2));
    if (spinLeft)
      data.mutable_angular()->set_z(0.5);
    else
      data.mutable_angular()->set_z(-0.5);

    data.mutable_linear()->set_x(0.0);

    if (m_debug) std::cout << "Obstacle! Rotate [" << i << "]:" << (spinLeft ? "left" : "right") << std::endl;
    m_twistPub.Publish(data);
  }
  else if (m_stepCount++ < MAX_STEPS)
  {
    if (m_debug) std::cout << "Stepping!" << std::endl;
    data.mutable_linear()->set_x(0.5);
    data.mutable_angular()->set_z(0.0);
    m_twistPub.Publish(data);
  }
  else
      setIsObstacle(false);
}

//======================================================================
// Function called each time odometry topic update is received.
//======================================================================
void cbOdometry(const ignition::msgs::Odometry &a_odom)
{
  ignition::msgs::Twist data;

  // IF an object is detected, don't do anything. Let the lidar callback handle avoiding the object.
  if (isObstacle())
    return;

  // IF we are stopped and still have available goals, get the next goal
  if (isStopped() && !m_goals.empty())
  {
    Coord2D goal = m_goals.front();
    m_goals.pop();
    m_goal.set_x(goal.first);
    m_goal.set_y(goal.second);
    if (m_debug)
      std::cout << std::endl << "----------- New Goal! " << goal.first << ", " << goal.second << "------------" << std::endl;
  }

  // Get Pose data
  ignition::msgs::Pose pose = a_odom.pose();

  // Create a quaternion
  ignition::msgs::Quaternion orientation = pose.orientation();
  tf2::Quaternion q(orientation.x(), orientation.y(), orientation.z(), orientation.w());

  // Convert to Euler angles
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Calculate position difference to goal
  ignition::msgs::Vector3d position = pose.position();

  double diff_x = m_goal.x() - position.x();
  double diff_y = m_goal.y() - position.y();

  double angle_to_goal = atan2(diff_y, diff_x);

  // Calculate our heading error
  double angle_diff = angle_to_goal - yaw;

  if (abs(diff_x) < 0.3 && abs(diff_y) < 0.3)
  {
    // Reach goal. Stop
    if (m_debug) std::cout << std::endl << "Stop!";
    data.mutable_linear()->set_x(0.0);
    data.mutable_angular()->set_z(0.0);
    setIsStopped(true);
  }
  else if (abs(angle_diff) > 0.5)
  {
    // spin in place
    data.mutable_linear()->set_x(0.0);
    double angle_direction = (angle_diff > 0) ? 1.0 : -1.0;
    data.mutable_angular()->set_z(0.3 * angle_direction);
    if (m_debug) std::cout << "Spinning! Rotate " << (angle_direction ? "left" : "right");
    setIsStopped(false);
  }
  else
  {
    // move to goal. No spin
    if (m_debug) std::cout << "Moving!";
    data.mutable_linear()->set_x(0.5);
    data.mutable_angular()->set_z(0.0);
    setIsStopped(false);
  }
  if (m_debug) std::cout << "\tposition: " << position.x() << ", " << position.y() << ", " << position.z() << std::endl;
  //if (m_debug) std::cout << " -> Yaw: " << yaw << ", " << "Goal: " << angle_to_goal << ", Diff: " << angle_diff << std::endl;
  m_twistPub.Publish(data);
}

//======================================================================
// Read coordinates from file and add them to goal queue
//======================================================================
bool setGoals(const char* filename)
{
  bool ok = true;

  FILE *file = fopen(filename, "r");
  char line[128];
  if (file != NULL)
  {
    while (fgets(line, sizeof(line), file))
    {
      char * token = strtok(line, ",");
      double x = atof(token);
      token = strtok(NULL, ",");
      double y = atof(token);

      if (m_debug) std::cout << "Adding goal: (" << x << "," << y << ")" << std::endl;
      m_goals.push(Coord2D(x,y));
    }

    fclose(file);
  }
  else
  {
    std::cerr << "Failed to open coordinate file: " << filename << std::endl << std::endl;
    ok = false;
  }

  return ok;
}

//======================================================================
/// Main function
//======================================================================
int main(int argc, char **argv)
{
  // Check input parameters
  if (argc < 2)
  {
    std::cerr << "Point coordinates file is required as input:\n"
              << "  lidar_node waypoint.txt"
              << std::endl << std::endl;
    return -1;
  }

  // Read goals
  if (!setGoals(argv[1]))
    return -1;

  // Subscribe to lidar topic
  std::string topic_sub = "/lidar";
  if (!m_node.Subscribe(topic_sub, cbLidar))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub << "]" << std::endl;
    return -1;
  }
  else
    if (m_debug) std::cout << "Subscribed to topic [" << topic_sub << "]" << std::endl;

  // Subscribe to odometry topic
  topic_sub = "/model/vehicle/odometry";
  if (!m_node.Subscribe(topic_sub, cbOdometry))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub << "]" << std::endl;
    return -1;
  }
  else
    if (m_debug) std::cout << "Subscribed to topic [" << topic_sub << "]" << std::endl;

  ignition::transport::waitForShutdown();

  return 0;
}
