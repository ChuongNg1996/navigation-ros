#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "geometry_msgs/PoseStamped.h"
using namespace std;

int i = 0;
bool done = 1;

void next_waypoint (const move_base_msgs::MoveBaseActionResult& result)
{
    if(result.status.text == "Goal reached.")
    {
        done = 1;
        cout << "waypoint: "<<i<< " reached"<<endl;
        i++;
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Set_Goals");
    ros::NodeHandle node;
    ros::Publisher goals_pub = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    ros::Subscriber goal_reached = node.subscribe ("/move_base/result",10, next_waypoint);
    vector<geometry_msgs::PoseStamped> waypoints_array; 
    geometry_msgs::PoseStamped waypoint;
    ros::Duration(1.0).sleep();
    //------------ GLOBAL COST MAP SETUP 1------------//
    /*
    update_frequency: 2.0
    width: 150.0
    height: 150.0
    resolution: 0.05
    */

    //------------------------------------------------//

    //------------ WAYPOINT 1 ------------//
    waypoint.pose.position.x = 60.7447166443;
    waypoint.pose.position.y = 28.2257862091;
    waypoint.pose.position.z = 0.0;
    waypoint.pose.orientation.x = 0.0;
    waypoint.pose.orientation.y = 0.0;
    waypoint.pose.orientation.z = -0.029773208343;
    waypoint.pose.orientation.w = 0.999556679766;
    waypoint.header.frame_id ="odom";
    waypoints_array.push_back(waypoint);

    //------------ WAYPOINT 2 ------------//
    waypoint.pose.position.x = 130.920623779;
    waypoint.pose.position.y = 7.94414520264;
    waypoint.pose.orientation.z = -0.25460803139;
    waypoint.pose.orientation.w = 0.967044337325;
    waypoints_array.push_back(waypoint);

    //------------ WAYPOINT 3 ------------//
    waypoint.pose.position.x = 196.826202393;
    waypoint.pose.position.y = -17.037065506;
    waypoint.pose.orientation.z = -0.04253047096393;
    waypoint.pose.orientation.w = 0.999095170161;
    waypoints_array.push_back(waypoint);

    //------------ WAYPOINT 4 ------------//
    waypoint.pose.position.x = 270.537231445;
    waypoint.pose.position.y = -28.1860637665;
    waypoint.pose.orientation.z = -0.0561101972342;
    waypoint.pose.orientation.w = 0.998424581912;
    waypoints_array.push_back(waypoint);

    //------------ WAYPOINT 5 ------------//
    waypoint.pose.position.x = 279.898284912;
    waypoint.pose.position.y = 38.3095436096;
    waypoint.pose.orientation.z = 0.755969401057;
    waypoint.pose.orientation.w = 0.654606954336;
    waypoints_array.push_back(waypoint);

    //------------ WAYPOINT 6 ------------//
    waypoint.pose.position.x = 262.068634033;
    waypoint.pose.position.y = 108.920783997;
    waypoint.pose.orientation.z = 0.848084601025;
    waypoint.pose.orientation.w = 0.529860839753;
    waypoints_array.push_back(waypoint);

    //------------ WAYPOINT 7 ------------//
    waypoint.pose.position.x = 234.986434937;
    waypoint.pose.position.y = 179.082687378;
    waypoint.pose.orientation.z = 0.803453074527;
    waypoint.pose.orientation.w = 0.595368085334;
    waypoints_array.push_back(waypoint);

    //------------ WAYPOINT 8 ------------//
    waypoint.pose.position.x = 237.432632446;
    waypoint.pose.position.y = 250.30418396;
    waypoint.pose.orientation.z = 0.370308072549;
    waypoint.pose.orientation.w = 0.928909000605;
    waypoints_array.push_back(waypoint);

    //------------ WAYPOINT 9 ------------//
    waypoint.pose.position.x = 255.111358643;
    waypoint.pose.position.y = 272.562866211;
    waypoint.pose.orientation.z = 0.407238499496;
    waypoint.pose.orientation.w = 0.913321851555;
    waypoints_array.push_back(waypoint);

    //------------ WAYPOINT 10 ------------//
    waypoint.pose.position.x = 181.116088867;
    waypoint.pose.position.y = 270.51083374;
    waypoint.pose.orientation.z = 0.941923711767;
    waypoint.pose.orientation.w = 0.33582692151;
    waypoints_array.push_back(waypoint);


        //------------ WAYPOINT 1 ------------//
    // waypoint.pose.position.x = 1.36949205399;
    // waypoint.pose.position.y = 5.36546564102;
    // waypoint.pose.position.z = 0.0;
    // waypoint.pose.orientation.x = 0.0;
    // waypoint.pose.orientation.y = 0.0;
    // waypoint.pose.orientation.z =0.657537118;
    // waypoint.pose.orientation.w =0.753422151554;
    // waypoint.header.frame_id ="wamv/odom";
    // waypoints_array.push_back(waypoint);

    // //------------ WAYPOINT 2 ------------//
    // waypoint.pose.position.x = 2.00639057159;
    // waypoint.pose.position.y = 9.39048957825;
    // waypoint.pose.orientation.z = 0.67212737158;
    // waypoint.pose.orientation.w = 0.740435545049;
    // waypoints_array.push_back(waypoint);

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        if(done == 1)
        {
            goals_pub.publish(waypoints_array[i]);
            done = 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}