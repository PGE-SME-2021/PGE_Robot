/*
 * Automatic Addison
 * Date: May 28, 2021
 * ROS Version: ROS 1 - Melodic
 * Website: https://automaticaddison.com
 * This ROS node uses the A* path planning algorithm to find
 *  the least-cost path from a starting location to a goal location.
 * Subscribe: ROS node that subscribes to the following topics:
 *   /costmap : Our costmap that will be generated as soon as an 
 *             initial pose is set (nav_msgs::OccupancyGrid).
 *   /goal_2d : Goal position and orientation (geometry_msgs::PoseStamped).
 *   Note: The current pose of the robot comes by looking up the 
 *        transform between the odom frame and the base_link using 
 *        the ROS tf package.
 *
 * Publish: This node will publish to the following topics:
 *   /waypoint_2d : The desired locations where the robot needs to go in order 
 *                  to get to the goal (geometry_msgs/PoseStamped).
 *   /path: The planned path as determined by the A* path planning algorithm. 
 *          This path can be loaded into RViz to see the planned path on 
 *          the map (nav_msgs::Path).
 *
 * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */
 
// Include the relevant libraries
#include "ros/ros.h"
#include "navigation_data_pub/practical_common.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <tf/transform_listener.h>
#include <vector>
#include <math.h>
#include <iostream>

using namespace std;

// Create the subscriber our subscriber and publisher.
ros::Subscriber subMap, subGoal;
ros::Publisher pub;
ros::Publisher pathPub;

// Create our occupancy grid object. 
nav_msgs::OccupancyGrid::Ptr _map(new nav_msgs::OccupancyGrid());

// Here we define what a cell is going to be made up of
struct cell
{
    cell() : index(-1), x(-1), y(-1), theta(-1), F(INT32_MAX), G(INT32_MAX), H(INT32_MAX),
             prevX(-1), prevY(-1) {}
    cell(const cell &incoming);

    int index; // The index in the nav_msgs::OccupancyGrid
    int x;     // x, y as grid cells are pose in meters/mapResolution (10)
    int y;
    double theta; // The final waypoint is the goal and requires heading theta
    int F;        // The total cost of the cell, will be calculated as G + H
    int G;        // Cost (distancetraveled) from start (current position)
    int H;        // Manhattan distance to the goal location
    int prevX;    // The map grid coordinates of the previous cell
    int prevY;
};


// Copy constructor
cell::cell(const cell &incoming)
{
    index = incoming.index;
    x = incoming.x;
    y = incoming.y;
    theta = incoming.theta;
    F = incoming.F;
    G = incoming.G;
    H = incoming.H;
    prevX = incoming.prevX;
    prevY = incoming.prevY;
}

cell start;
cell goal;
bool goalActive = false;

// Copy the supplied costmap to a new _map we can access freely
void map_handler(const nav_msgs::OccupancyGridPtr &costmap)
{
    static bool init_complete = false;
    // This piece of code will only execute the first time a map is 
		// received.
    if (init_complete == false)
    {
        _map->header.frame_id = costmap->header.frame_id;
        _map->info.resolution = costmap->info.resolution;
        _map->info.width = costmap->info.width;
        _map->info.height = costmap->info.height;
        _map->info.origin.position.x = costmap->info.origin.position.x;
        _map->info.origin.position.y = costmap->info.origin.position.y;
        _map->info.origin.orientation.x = costmap->info.origin.orientation.x;
        _map->info.origin.orientation.y = costmap->info.origin.orientation.y;
        _map->info.origin.orientation.z = costmap->info.origin.orientation.z;
        _map->info.origin.orientation.w = costmap->info.origin.orientation.w;
        _map->data.resize(costmap->data.size());

        cout << "Map recieved. Initializing _map size "
             << _map->info.width << " x " << _map->info.height<<" = "<<costmap->data.size() <<"  at resolution "
             << _map->info.resolution<<"\nOrigin: "
             << _map->info.origin.position.x<<", "<< _map->info.origin.position.x<<endl;


        init_complete = true;
    }

    //origins in cells instead of meters.
    int originX = 1 - (_map->info.origin.position.x / _map->info.resolution);
    int originY = 1 - (_map->info.origin.position.y / _map->info.resolution);


    // This part we can do every time to ensure we see updates.
    // Copy the contents of the costmap into the occupancy grid path_planner uses internally
    // Starting at 0, 0. Data at x<0 or y<0 in the gmap is lost, so set start position accordingly
        for(int row = originY; row < _map->info.height ; row++)
        {
            for(int col = originX; col < _map->info.width; col++)
            {
                _map->data[getIndex(col-originY, row-originX, costmap)]
                            = costmap->data[getIndex(col, row, costmap)];
            }
        }

}

// Set our start cell as the current grid cell
bool update_start_cell()
{
    static tf::TransformListener listener;
    tf::StampedTransform odom_base_tf;

    if(listener.canTransform("odom","base_link", ros::Time(0), NULL))
    {
        listener.lookupTransform("odom", "base_link", ros::Time(0), odom_base_tf);

        // Dont forget the grid cell is pose in meters / map resolution
        start.x = odom_base_tf.getOrigin().x()/ map_resolution(_map);
        start.y = odom_base_tf.getOrigin().y()/ map_resolution(_map);

        tf::Quaternion q(0, 0, odom_base_tf.getRotation().z(), odom_base_tf.getRotation().w());
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        start.theta = yaw;
        start.index = getIndex(start.x, start.y, _map);
        return true;
    }
    else
    {
        cout<<"UNABLE TO LOOKUP ODOM -> BASE_LINK TRANSFORM, no path planned"<<endl;
        return false;
    }
}

// Set the goal recieved and set goalActive = true to start pathfinding
void set_goal(const geometry_msgs::PoseStamped &desiredPose)
{
    //dont forget that grid cell is pose in meters / map resolution
    goal.x = (int)(desiredPose.pose.position.x / map_resolution(_map));
    goal.y = (int)(desiredPose.pose.position.y / map_resolution(_map));
    goal.theta = desiredPose.pose.orientation.z;
    goal.index = getIndex(goal.x, goal.y, _map);
    goal.H = 0; // Must set to zero to identify when found
    goalActive = true;
    cout << "goal active set true" << endl;
}

// Check if cell with index recieved is in the supplied open or closed list
bool contains(vector<cell> &list, int toCheck)
{
    for (int i = 0; i < list.size(); i++)
    {
        if (list[i].index == toCheck)
        {
            return true;
        }
    }
    // If not found in above loop it is not found
    return false;
}

// Helper to calculate G - the cost to traverse this particular cell
double getG(int x, int y, int currentX, int currentY, double currentG)
{
    // Cost is infinite if cell is an obstacle
    if (is_obstacle(x, y, _map))
    {
        return INT32_MAX;
    }
    // If cell is not diagonal, the cost to move is 10
    else if (x == currentX || y == currentY)
    {
        return currentG + 10;
    }
    // If cost is 14.142, the cell is diagonal
    else
    {
        return currentG + 14.142;
    }
}

// Helper to calculate H heuristic - the Manhattan distance from this cell to goal
double getH(int x, int y)
{
    return (abs(goal.x - x) + abs(goal.y - y)) * 10;
}

// Helper to calculate F, but avoid integer rollover
double getF(int g, int h)
{
    if (g == INT32_MAX)
    {
        return g;
    }
    else
    {
        return g + h;
    }
}

// Deletes all waypoints between start and the furthest waypoint with a straight, unobstructed path
void optimize(vector<cell> &path)
{
    // Is there an obstacle between start and cell we're checking
    bool obstacle_on_line = true;

    // path[0] = index of goal cell in path[]
    int furthestFreeCell = 0;

    // Starting at last goal (path[0]) and checking each waypoint until we find clear straight line to a cell
    while (obstacle_on_line == true && path[furthestFreeCell++].index != path.back().index)
    {
        cout<<"furthest free cell = "<<path[furthestFreeCell].index<<" and val = "<<(int)_map->data[path[furthestFreeCell].index]<<endl;
				
        // Iterate between points. set our start and endpoints for iterating
        int startX, endX, startY, endY;
        if (start.x <= path[furthestFreeCell].x)
        {
            startX = start.x;
            endX = path[furthestFreeCell].x;
        }
        else
        {
            startX = path[furthestFreeCell].x;
            endX = start.x;
        }

        if (start.y <= path[furthestFreeCell].y)
        {
            startY = start.y;
            endY = path[furthestFreeCell].y;
        }
        else
        {
            startY = path[furthestFreeCell].y;
            endY = start.y;
        }

        // If any one thing in for loop detects an obstacle, this will be set back to true
        // If no obstacles detected, we have found an unobstructed straight line to a waypoint
        obstacle_on_line = false;
				
        // Check every y for every x in range
        for (int x = startX; x != endX && obstacle_on_line == false; x++)
        {
            // Make sure we don't try to calculate slope of vertical line where y is always the same
            if (startY == endY)
            {
                obstacle_on_line = is_obstacle(x, startY, _map);
            }
            else
            {
                obstacle_on_line = is_obstacle(x, (int)get_y_intercept(start.x, start.y, path[furthestFreeCell].x, path[furthestFreeCell].y, x), _map);
            }
        }
        // Check every x for every y in range
        for (int y = startY; y != endY && obstacle_on_line == false; y++)
        {
            // Handle horizontal lines
            if (startX == endX)
            {
                obstacle_on_line = is_obstacle(startX, y, _map);
            }
            else
            {
                obstacle_on_line = is_obstacle((int)get_x_intercept(start.x, start.y, path[furthestFreeCell].x, path[furthestFreeCell].y, y), y, _map);
            }
        }
    }
    // If in a gray area, optimize might think the closest straight path is the current location
    // Make sure at least on cell remains or planner will simply publish the start (current) and the robot won't move
    if(furthestFreeCell == path.size()-1)
    {
        furthestFreeCell--;
    }
    cout << "FOUND FURTHEST STRAIGHT LINE FROM START TO waypoint at x, y = " << path[furthestFreeCell].x << ", " << path[furthestFreeCell].y << endl;

    // Pop cells off waypoint list until we get to furthestFreeCell
    while (path[furthestFreeCell].index != path.back().index)
    {
        path.pop_back();
    }
    // Put start back for visualization waypoint list
    path.push_back(cell(start));
}
// Publish the next waypoint is 2d form, ignoring quaternion nature of PoseSatamped data type
void publish_waypoint(cell nextWaypoint)
{
    geometry_msgs::PoseStamped waypoint;
    waypoint.header.frame_id = "map";
    waypoint.header.stamp = ros::Time::now();
    //convert cell x, y coords to position in meters
    waypoint.pose.position.x = (double)(nextWaypoint.x) / 10 + .05;
    waypoint.pose.position.y = (double)(nextWaypoint.y) / 10 + .05;
    waypoint.pose.position.z = 0;
    waypoint.pose.orientation.x = 0;
    waypoint.pose.orientation.y = 0;
    waypoint.pose.orientation.z = nextWaypoint.theta;
    waypoint.pose.orientation.w = 0;

    cout << "Publishing waypoint and theta " << waypoint.pose.position.x << ", " << waypoint.pose.position.y << "   " << waypoint.pose.orientation.z << endl;
    pub.publish(waypoint);
}

// Optional - publish path for rviz
void publishPathforRviz(vector<cell> &path)
{
    nav_msgs::Path waypoints;
    waypoints.header.frame_id = "map";
    waypoints.header.stamp = ros::Time::now();
    waypoints.poses.resize(path.size());

    for (int i = 0; i < path.size(); i++)
    {
        // Path finds integers of grid cells, we need to publish waypoints
        // as doubles, and add .05 meters so we aim for middle of the cells
        waypoints.poses[i].header.frame_id = "map";
        waypoints.poses[i].header.stamp = ros::Time::now();
        waypoints.poses[i].pose.position.x = (double)(path[i].x) / 10 + .05;
        waypoints.poses[i].pose.position.y = (double)(path[i].y) / 10 + .05;
        waypoints.poses[i].pose.position.z = 0;
        waypoints.poses[i].pose.orientation.x = 0;
        waypoints.poses[i].pose.orientation.y = 0;
        waypoints.poses[i].pose.orientation.z = 0;
        waypoints.poses[i].pose.orientation.w = 1;
    }
    pathPub.publish(waypoints);
}

// Trace the path through the closed list and return the next waypoint to be published
int trace(vector<cell> &closed)
{
    vector<cell> path;
    //Closed.back() will be our goal, and will be element [0] in path
    path.push_back(cell(closed.back()));
    bool pathComplete = false;
    while (pathComplete == false)
    {
        bool found = false;
        // Check the closed list for the parent cell of the last cell in path[]
        for (int i = 0; found == false && i < closed.size(); i++)
        {
            // When we find the parent cell, push it to path and we will lool for its parent cell next
            if (closed[i].x == path.back().prevX && closed[i].y == path.back().prevY)
            {
                path.push_back(cell(closed[i]));
                found = true;
            }
        }
        // When last element in path[] is the same as our start, we have the complete path
        if (path.back().index == start.index)
        {
            pathComplete = true;
        }
    }

    // Remove waypoints between start and whicever cell we cen get to going in a straght line
    if(path.size() > 2)
    {
        optimize(path);
    }

    // Optionally call this function to publish path to visualize on rviz
    publishPathforRviz(path);

    // The waypoint at back() is currently our start point. By removing it, the new back() will
    // be our first waypoint - the one we need to publish to the drive controller.
    if (path.back().index != goal.index)
    {
        path.pop_back();
    }

    // If goal, publish goal heading, else publish the heading we took to get here anyway
    if (path.back().index != path.front().index)
    {
        double deltaX = path.back().x - start.x;
        double deltaY = path.back().y - start.y;
        path.back().theta = atan2(deltaY, deltaX);
    }
    publish_waypoint(path.back());
    return path.back().index;
}

int find_path()
{
    vector<cell> open;
    vector<cell> closed;
    cell current(start);
    // Special case start G must be initialized to 0
    current.G = 0;
    current.H = getH(start.x, start.y);
    current.F = current.G + current.H;
    current.index = (getIndex(current.x, current.y, _map));
    open.push_back(cell(current));

    // Main loop cycles until .H = 0, which means we have found the goal
    while (current.H > 0)
    {
        // Iterate over the cells all around the current cell
        for (int x = current.x - 1; x <= current.x + 1; x++)
        {
            for (int y = current.y - 1; y <= current.y + 1; y++)
            {
                // Don't check elements out of array bounds
                if (is_in_bounds(x, y, _map))
                {
                    // When we run out of open elements, the must not be a path
                    if (open.size() == 0)
                    {
                        cout << "NO PATH FOUND" << endl;
                        goalActive = false;
                        return -1;
                    }
                    // If in the open list, check for lower cost path to here
                    if (contains(open, getIndex(x, y, _map)) == true)
                    {
                        // Iterate the list until we find the relevant cell
                        int i = 0;
                        while (open[i].index != getIndex(x, y, _map))
                        {
                            i++;
                        }

                        int tempG = getG(x, y, current.x, current.y, current.G);
                        int tempH = getH(x, y);
                        // If this calculation results in lower F cost, replace cells parents with current cell
                        if (tempG + tempH < open[i].F && tempG != INT32_MAX)
                        {
                            open[i].F = tempG + tempH;
                            open[i].G = tempG;
                            open[i].prevX = current.x;
                            open[i].prevY = current.y;
                        }
                    }
                    // If this one not not in open or closed list, add it
                    else if (contains(closed, getIndex(x, y, _map)) == false)
                    {
                        // Create the cell object with current cell data
                        cell newCell;
                        newCell.x = x;
                        newCell.y = y;
                        newCell.index = getIndex(x, y, _map);
                        newCell.prevX = current.x;
                        newCell.prevY = current.y;
                        // Calc G dis from start
                        newCell.G = getG(x, y, current.x, current.y, current.G);
                        //calc H to goal
                        newCell.H = getH(x, y);
                        //calc F = g+h
                        if (getF(newCell.G, newCell.H) < INT32_MAX)
                        {
                            newCell.F = getF(newCell.G, newCell.H);
                        }
                        // Add to closed list if obstacle, else add to open list
                        if (newCell.F == INT32_MAX)
                        {
                            closed.push_back(cell(newCell));
                        }
                        else
                        {
                            open.push_back(newCell);
                        }
                    }
                }
            }
        }

        // When we've checked all neighbors, add current cell to closed list and remove from open
        closed.push_back(cell(current));
        bool found = false;
        for (int i = 0; found == false; i++)
        {
            if (open[i].index == current.index)
            {
                open.erase(open.begin() + i);
                found = true;
            }
        }

        // Find the cell in open list with lowest f cost
        int lowestF = 0;
        for (int i = 0; i < open.size(); i++)
        {
            if (open[i].F < open[lowestF].F)
            {
                lowestF = i;
            }
        }

        // Now make the current = cell we found with lowest f cost
        current.index = open[lowestF].index;
        current.x = open[lowestF].x;
        current.y = open[lowestF].y;
        current.theta = open[lowestF].theta;
        current.F = open[lowestF].F;
        current.G = open[lowestF].G;
        current.H = open[lowestF].H;
        current.prevX = open[lowestF].prevX;
        current.prevY = open[lowestF].prevY;
    }
    // At this stage, we have found the goal. set goal's parents to last cell found
    //then add to goal to closed list.
    goal.prevX = closed.back().x;
    goal.prevY = closed.back().y;
    closed.push_back(cell(goal));

    //trace back through closed list
    int nextWaypoint = trace(closed);

    ///////////////////TEMP____remove THIS setting false for moving robot//////////////////////////
    // goalActive = false;
    return nextWaypoint;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle node;

    // Start tf::listener early so it has time to fill buffer
    update_start_cell();

    // Subscribe to _map and goal location
    subMap = node.subscribe("costmap", 0, map_handler);
    subGoal = node.subscribe("goal_2d", 0, set_goal);

    // Advertise publisher
    pub = node.advertise<geometry_msgs::PoseStamped>("waypoint_2d", 0);
    pathPub = node.advertise<nav_msgs::Path>("path", 0);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        if (goalActive == true)
        {
            if(update_start_cell() == false)
            {
                start.index = 0;
                goal.index = 0;
            }
                 cout<< "start and goal = "
                 << start.x << ", " << start.y << "......" << goal.x << ", " << goal.y << endl;

            // Stop until we receive a new goal
            if (start.index == goal.index)
            {
                cout << "Arrived, goalActive set false" << endl;
                publish_waypoint(goal);
                goalActive = false;
            }
            // keep updating path every loop_rate()
            else
            {
                int nextWaypoint = find_path();
                if (nextWaypoint == -1)
                {
                    cout << "NO PATH FOUND" << endl;
                    goalActive = false;
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
