/*
 * Obstacle detection and avoidance
 */

#include <stdio.h>
#include <stdlib.h>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/transport/Node.hh>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

bool m_debug = false;

// Topic to publish linear and angular data
std::string topic_pub = "/cmd_vel";
ignition::transport::Node m_node;
auto m_twistPub = m_node.Advertise<ignition::msgs::Twist>(topic_pub);

// Step and rotation values
const double STEP_VAL = 0.5;
const double ROT_VAL = 0.3;

// Obstacle detected flag
std::mutex m_isObstacleMutex;
bool m_isObstacle = false;
int m_stepCount = 0;
bool m_sampleTaken = false;

// Scan sectors
enum SECTORS
{
  BACK_RIGHT,
  RIGHT_BACK,
  RIGHT_FRONT,
  FRONT_RIGHT,
  FRONT_LEFT,
  LEFT_RIGHT,
  LEFT_BACK,
  BACK_LEFT
};

const int SCAN_SECTORS = 8;
const int SECTION_SIZE = 640 / SCAN_SECTORS;
std::vector<int> m_sectors(SCAN_SECTORS, 0);
std::vector<int> m_sectorsCount(SCAN_SECTORS, 0);
std::vector<int> m_sectorsCountInit(SCAN_SECTORS, 0);
const int SECTOR_THRESHOLD = 5;

// Vehicle stopped flag
bool m_isStopped = true;

// Vehicle moving flag
std::mutex m_isMovingMutex;
bool m_isMoving = false;

// Queue for our waypoint coordinates
typedef std::pair<double, double> Coord2D;
std::queue<Coord2D> m_goals;

// Current coordinate to reach
ignition::msgs::Vector3d m_goal;

// Max steps to take after obstacle is no longer detected.
int MAX_STEPS = 2;

// Threshold limit that we reached our goal
double GOAL_THRESHOLD = 0.3;

// State Machine states
enum obs_states
{
  INIT,
  STATIC,
  DYNAMIC,
  SCANNING,
  INVALID
};
enum obs_states m_OBS_STATE = SCANNING;

//======================================================================
// Print Pose
//======================================================================
void printPose(ignition::msgs::Pose &a_pose)
{
  ignition::msgs::Quaternion orientation = a_pose.orientation();
  ignition::msgs::Vector3d position = a_pose.position();
  std::cout << "pose:";
  std::cout << "\tposition: "
            << std::fixed << std::setprecision(4)
            << position.x() << ", "
            << position.y() << ", "
            << position.z();
  std::cout << "\torientation: "
            << std::fixed << std::setprecision(4)
            << orientation.w() << ", "
            << orientation.x() << ", "
            << orientation.y() << ", "
            << orientation.z()
            << std::endl;
}

//======================================================================
// Print Twist
//======================================================================
void printTwist(ignition::msgs::Twist &a_twist)
{
  ignition::msgs::Vector3d angular = a_twist.angular();
  ignition::msgs::Vector3d linear = a_twist.linear();
  std::cout << "twist:";
  std::cout << "\tlinear: "
            << std::fixed << std::setprecision(4)
            << linear.x() << ", "
            << linear.y() << ", "
            << linear.z();
  std::cout << "\tangular: "
            << std::fixed << std::setprecision(4)
            << angular.x() << ", "
            << angular.y() << ", "
            << angular.z()
            << std::endl;
}

//======================================================================
// Convert state to string
//======================================================================
std::string stateToString(enum obs_states a_state)
{
  switch (a_state)
  {
  case INIT:
    return "INIT";
  case STATIC:
    return "STATIC";
  case DYNAMIC:
    return "DYNAMIC";
  case SCANNING:
    return "SCANNING";
  default:
    return "INVALID";
    break;
  }
}

//======================================================================
// Update state machine state
//======================================================================
void setState(enum obs_states a_state)
{
  if (m_debug)
    std::cout << "SET STATE. new: " << stateToString(a_state) << ", old: " << stateToString(m_OBS_STATE) << std::endl;

  m_OBS_STATE = a_state;
}

//======================================================================
// Vehicle moving flag setter and getter
//======================================================================
bool isMoving()
{
  std::scoped_lock lock(m_isMovingMutex);
  return m_isMoving;
}

void setMoving(ignition::msgs::Twist &a_twist)
{
  std::scoped_lock lock(m_isMovingMutex);
  m_isMoving = (a_twist.linear().x() > 0.01);

  if (m_debug)
    std::cout << (m_isMoving ? "" : "NOT ") << "MOVING"
              << std::fixed << std::setprecision(4)
              << ". X: " << a_twist.linear().x()
              << ", Z: " << a_twist.angular().z() << std::endl;
}

//======================================================================
// Obstacle flag setter and getter
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
// Vehicle stopped flag setter and getter
//======================================================================
bool isStopped()
{
  return m_isStopped;
}

void setIsStopped(bool a_stop)
{
  m_isStopped = a_stop;
}

//======================================================================
// Check if both sectors are the same within the given threshold
//======================================================================
bool similarCount(std::vector<int> a_sector1, std::vector<int> a_sector2, int a_threshold)
{
  bool similar = true;

  for (int i = 0; i < SCAN_SECTORS; i++)
  {
    if (abs(a_sector1[i] - a_sector2[i]) > a_threshold)
    {
      similar = false;
      break;
    }
  }

  return similar;
}

//======================================================================
// Sum up the total sector counts for the specified sectors
//======================================================================
int sampleSum(std::vector<int> a_sectorsCount, enum SECTORS a_start, enum SECTORS a_end)
{
  int sum = 0;

  // Check for overflow
  if (a_end > a_sectorsCount.size())
    return sum;

  // Sum up the counts
  for (int i = a_start; i <= a_end; i++)
    sum += a_sectorsCount[i];

  return sum;
}

//======================================================================
// This method detects if an obstacle is static or dynamic (moving)
//======================================================================
void obstacleType(std::vector<int> a_sectorsCount)
{
  ignition::msgs::Twist data;

  // Obstacle detected. Stop vehicle
  data.mutable_linear()->set_x(0.0);
  data.mutable_angular()->set_z(0.0);
  m_twistPub.Publish(data);

  // IF our vehicle is not moving, then start sequence to determine obstacle type.
  if (!isMoving())
  {
    // IF we haven't taken a sample, take one.
    if (!m_sampleTaken)
    {
      m_sectorsCountInit = a_sectorsCount;
      m_sampleTaken = true;
      setState(INIT);

      if (m_debug)
        std::cout << "obstacleType() Sampling" << std::endl;
    }
    // ELSE determine obstacle type.
    else
    {
      // IF the counts are still the same, then consider this a static variable
      if (similarCount(m_sectorsCountInit, a_sectorsCount, SECTOR_THRESHOLD))
      {
        setState(STATIC);

        if (m_debug)
          std::cout << "obstacleType() Static" << std::endl;
      }
      // ELSE its a moving/dynamic obstacle
      else
      {
        setState(DYNAMIC);

        if (m_debug)
          std::cout << "obstacleType() Dynamic" << std::endl;
      }

      m_sampleTaken = false;
    }
  }
}

//======================================================================
// Method to manage vehicle around static obstacles
//======================================================================
void obstacleStatic(std::vector<int> a_sectorsCount, bool a_isObstacles)
{
  ignition::msgs::Twist data;

  // IF have obstacles in sight, then manouver vehicle away from it
  if (a_isObstacles)
  {
    m_stepCount = 0;

    // IF the obstacle is behind us, move away
    if (a_sectorsCount[BACK_RIGHT] && a_sectorsCount[BACK_LEFT])
    {
      data.mutable_linear()->set_x(STEP_VAL);
      data.mutable_angular()->set_z(0.0);
      m_twistPub.Publish(data);

      if (m_debug)
        std::cout << "obstacleStatic(). BACK. Away" << std::endl;
    }
    // ELSE rotate away from obstacle
    else
    {
      // IF our greatest area of detection is on the right side of the scan, rotate left.
      // else rotate right.
      int sumRight = sampleSum(a_sectorsCount, BACK_RIGHT, FRONT_RIGHT);
      int sumLeft = sampleSum(a_sectorsCount, FRONT_LEFT, BACK_LEFT);
      bool spinLeft = (sumRight > sumLeft);
      if (spinLeft)
        data.mutable_angular()->set_z(ROT_VAL);
      else
        data.mutable_angular()->set_z(-ROT_VAL);

      data.mutable_linear()->set_x(0.0);

      m_twistPub.Publish(data);

      if (m_debug)
        std::cout << "obstacleStatic(). Front. Rotate: "
                  << (spinLeft ? "left" : "right") << "[" << sumRight << "," << sumLeft << "]" << std::endl;
    }
  }
  // ELSE IF we haven't moved away, take a step
  else if (m_stepCount++ < MAX_STEPS)
  {
    data.mutable_linear()->set_x(STEP_VAL);
    data.mutable_angular()->set_z(0.0);
    m_twistPub.Publish(data);

    if (m_debug)
      std::cout << "obstacleStatic(). Stepping" << std::endl;
  }
  // ELSE obstacle is no longer visible, so clear obstacle flag and update state
  else
  {
    setIsObstacle(false);
    setState(SCANNING);
  }
}

//======================================================================
// Method to manage vehicle around dynamic obstacles
//======================================================================
void obstacleDynamic(std::vector<int> a_sectorsCount, bool a_isObstacles)
{
  // IF we have obstacles, then immediately stop and change state
  // to restart obstacle detection sequence
  if (a_isObstacles)
  {
    ignition::msgs::Twist data;
    data.mutable_linear()->set_x(0.0);
    data.mutable_angular()->set_z(0.0);
    m_twistPub.Publish(data);
    setState(INIT);

    if (m_debug)
      std::cout << "obstacleDynamic(). Stop" << std::endl;
  }
  // ELSE if we don't have obstacles, clear obstacle flag and update state
  else
  {
    setIsObstacle(false);
    setState(SCANNING);

    if (m_debug)
      std::cout << "obstacleDynamic(). Reset" << std::endl;
  }
}

//======================================================================
// Function called each time lidar topic update is received.
//======================================================================
void cbLidar(const ignition::msgs::LaserScan &a_scan)
{
  ignition::msgs::Twist data;
  bool isObstacleDetected = false;

  // Reset sector counts
  std::fill(m_sectorsCount.begin(), m_sectorsCount.end(), 0);

  // Go over scan data and count the number of obstacles detected per sector.
  for (int i = 0; i < m_sectors.size(); i++)
  {
    int start = (i == 0) ? 0 : m_sectors[i - 1];
    for (int j = start; j < m_sectors[i]; j++)
    {
      if (a_scan.ranges(j) < 1.0)
      {
        isObstacleDetected = true;
        m_sectorsCount[i]++;
      }
    }
  }

  if (m_debug && isObstacleDetected)
  {
    std::cout << std::endl;
    std::cout << "Obstacle Sectors: [";
    int i = 0;
    for (; i < (m_sectorsCount.size() - 1); i++)
      std::cout << m_sectorsCount[i] << ",";
    std::cout << m_sectorsCount[i] << "]" << std::endl;
  }

  // IF we detect an obstacle AND our current state is SCANNING,
  // change state to INIT and set obstacle flag
  if (isObstacleDetected && (m_OBS_STATE == SCANNING))
  {
    setIsObstacle(true);
    setState(INIT);
  }

  // State Machine
  switch (m_OBS_STATE)
  {
  case INIT:
    obstacleType(m_sectorsCount);
    break;

  case STATIC:
    obstacleStatic(m_sectorsCount, isObstacleDetected);
    break;

  case DYNAMIC:
    obstacleDynamic(m_sectorsCount, isObstacleDetected);
    break;
  }
}

//======================================================================
// Function called each time odometry topic update is received.
//======================================================================
void cbOdometry(const ignition::msgs::Odometry &a_odom)
{
  ignition::msgs::Twist data;

  if (m_debug)
    std::cout << "cbOdometry() position: "
              << std::fixed << std::setprecision(4)
              << a_odom.pose().position().x() << ", "
              << a_odom.pose().position().y() << ", "
              << a_odom.pose().position().z()
              << std::endl;

  ignition::msgs::Twist twist = a_odom.twist();
  setMoving(twist);

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
      std::cout << std::endl
                << "----------- New Goal! " << goal.first << ", " << goal.second << "------------" << std::endl;
  }

  // Get Pose data
  ignition::msgs::Pose pose = a_odom.pose();

  // Create a TF2 quaternion from our pose
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

  // Calculate the angle to our goal
  double angle_to_goal = atan2(diff_y, diff_x);

  // Calculate our heading error
  double angle_diff = angle_to_goal - yaw;

  // IF current location near our goal, within threshold, stop.
  // We reached the goal
  if ((abs(diff_x) < GOAL_THRESHOLD) && (abs(diff_y) < GOAL_THRESHOLD))
  {
    data.mutable_linear()->set_x(0.0);
    data.mutable_angular()->set_z(0.0);
    setIsStopped(true);

    if (m_debug)
      std::cout << std::endl
                << "cbOdometry() Reached GOAL!" << std::endl;
  }
  // ELSE IF our heading error is off, rotate towards our goal
  else if (abs(angle_diff) > 0.1)
  {
    double angle_direction = (angle_diff > 0.0) ? 1.0 : -1.0;
    data.mutable_linear()->set_x(0.0);
    data.mutable_angular()->set_z(ROT_VAL * angle_direction);
    setIsStopped(false);

    if (m_debug)
      std::cout << "cbOdometry() Spinning! Rotate "
                << ((angle_direction > 0.0) ? "left" : "right") << "[" << angle_diff << "]" << std::endl;
  }
  // ELSE, we are heading the correct way. Move to goal
  else
  {
    data.mutable_linear()->set_x(STEP_VAL);
    data.mutable_angular()->set_z(0.0);
    setIsStopped(false);

    if (m_debug)
      std::cout << "cbOdometry() Moving!" << std::endl;
  }

  m_twistPub.Publish(data);
}

//======================================================================
// Read coordinates from file and add them to goal queue
//======================================================================
bool setGoals(const char *filename)
{
  bool ok = true;

  FILE *file = fopen(filename, "r");
  char line[128];
  if (file != NULL)
  {
    while (fgets(line, sizeof(line), file))
    {
      char *token = strtok(line, ",");
      double x = atof(token);
      token = strtok(NULL, ",");
      double y = atof(token);

      m_goals.push(Coord2D(x, y));

      if (m_debug)
        std::cout << "Adding goal: (" << x << "," << y << ")" << std::endl;
    }

    fclose(file);
  }
  else
  {
    std::cerr << "Failed to open coordinate file: " << filename << std::endl
              << std::endl;
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
              << std::endl
              << std::endl;
    return -1;
  }

  // Initialize sector ranges
  for (int i = 0; i < m_sectors.size(); i++)
    m_sectors[i] = SECTION_SIZE * (i + 1);

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
  else if (m_debug)
    std::cout << "Subscribed to topic [" << topic_sub << "]" << std::endl;

  // Subscribe to odometry topic
  topic_sub = "/model/vehicle/odometry";
  if (!m_node.Subscribe(topic_sub, cbOdometry))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub << "]" << std::endl;
    return -1;
  }
  else if (m_debug)
    std::cout << "Subscribed to topic [" << topic_sub << "]" << std::endl;

  ignition::transport::waitForShutdown();

  return 0;
}
