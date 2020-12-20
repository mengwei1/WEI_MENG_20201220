#include "ros/ros.h"
#include "planner_agent/location.h"
#include "planner_agent/get_plan.h"
#include "planner_agent/agent_feedback.h"
#include "pathfinding.h"

#include <sstream>
#include <string>

using std::string;

struct Agent {
    string serialID;
    long x, y;

    Agent(): serialID(""), x(0), y(0) {}

    Agent(string serialID_, long x_, long y_): serialID(serialID_), x(x_), y(y_) {}
};

namespace std {
    template <> struct hash<Agent> {
        typedef Agent argument_type;
        typedef std::size_t result_type;
        std::size_t operator()(const Agent& agent) const noexcept {
            return std::hash<int>()((agent.x << 1) ^ (agent.x >> 1));
        }
    };
}

std::unordered_map<string, Agent> agents;

bool positionChanged(long oldX, long oldY, long newX, long newY) {
    return (oldX != newX) || (oldY != newY);
}

void agent_feedback_callback(const planner_agent::agent_feedback::ConstPtr& msg)
{
  string serial_id = msg->serial_id;
  long x = msg->current_location.x;
  long y = msg->current_location.y;
  ROS_INFO("agent feedback %s at %ld %ld", serial_id.c_str(), x, y);
  if(agents.find(serial_id) == agents.end()) {
      agents[serial_id] = Agent{serial_id, x, y};
  } else if(positionChanged(1L, 1L, 2L, 1L)){
      agents[serial_id] = Agent{serial_id, x, y};
  }
}

bool generate_plan(planner_agent::get_plan::Request   &rq,
                   planner_agent::get_plan::Response  &res)
{
  std::vector<GridLocation> path;

  string serial_id = rq.serial_id;
  long x = rq.goal.x;
  long y = rq.goal.y;

  ROS_INFO("generating plan for %s at %ld %ld", serial_id.c_str(), x, y);

  if(agents.find(serial_id) == agents.end()) {
      ROS_INFO("requesting agent has never feedback, skipping");
      return false;
  }

  long current_x = agents[serial_id].x;
  long current_y = agents[serial_id].y;

  if(!positionChanged(x, y, current_x, current_y)) {
      ROS_INFO("requesting goal equals to agent's current position, skipping");
      return false;
  }

  ROS_INFO("requesting goal differ to current position %ld %ld, processing", current_x, current_y);

  GridLocation start{current_x, current_y}, goal{x, y};
  path = searchOnDefaultGraph(start, goal);
  for(int i = 0; i < path.size(); i++) {
      planner_agent::location l;
      l.x = path[i].x;l.y = path[i].y;
      res.path.push_back(l);
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("agent_feedback", 1000, agent_feedback_callback);
  ros::ServiceServer service = n.advertiseService("get_plan", generate_plan);
  ROS_INFO("Ready to generate_plan.");

  ros::spin();

  return 0;
}
