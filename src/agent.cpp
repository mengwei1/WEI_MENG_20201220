#include "ros/ros.h"
#include "planner_agent/get_plan.h"
#include "planner_agent/update_goal.h"
#include "planner_agent/agent_feedback.h"
#include <cstdlib>
#include <string>

using std::string;

ros::ServiceClient client;
planner_agent::location current_location;
string serial_id;

bool update_goal(planner_agent::update_goal::Request   &rq,
                  planner_agent::update_goal::Response  &res){
  planner_agent::get_plan srv;
  srv.request.goal = rq.goal;
  srv.request.serial_id = serial_id;
  if(client.call(srv)) {
    current_location.x = rq.goal.x;
    current_location.y = rq.goal.y;
    res.result = 1;
    return true;
  } else {
    res.result = 0;
    return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agent");
  if (argc != 4)
  {
    ROS_INFO("usage: get_plan serial_id current_position_x current_position_y");
    return 1;
  }

  serial_id = argv[1];
  long current_position_x = atoll(argv[2]);
  long current_position_y = atoll(argv[3]);
  current_location.x = current_position_x;current_location.y = current_position_y;

  ros::NodeHandle n;
  ros::Publisher feedback = n.advertise<planner_agent::agent_feedback>("agent_feedback", 1000);
  ros::ServiceServer service = n.advertiseService("update_goal", update_goal);
  client = n.serviceClient<planner_agent::get_plan>("get_plan");

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    planner_agent::agent_feedback msg;

    msg.serial_id = serial_id;
    msg.current_location.x = current_location.x;
    msg.current_location.y = current_location.y;

    feedback.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}