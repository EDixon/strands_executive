#include "ros/ros.h"
#include "strands_executive_msgs/GetSchedule.h"
#include "task.h"
#include "scheduler.h"
#include <vector>
#include <algorithm>
#include "ros_datacentre_msgs/StringPairList.h"
#include "ros_datacentre/message_store.h"

using namespace std;
using namespace ros_datacentre;
using namespace ros_datacentre_msgs;

// /*constructor without parameter now, automatically set now to false*/
// Task::Task(unsigned int ID, double s, double e, double d, string start_pos, string end_pos)
// {
//   id = ID;
//   start = s;
//   end = e;
//   duration = d;
//   s_pos = start_pos;
//   e_pos = end_pos;
//   no = false;
//   cond = false;
// }

bool save_problems = true;
int scheduler_version = -1;
string output_file = "";
int timeout = 0;

Task * createSchedulerTask(const strands_executive_msgs::Task & _task, const ros::Time & _earliestStart, const bool & _demand=false) {


  auto startAfter = _earliestStart > _task.start_after ? _earliestStart : _task.start_after;

  // ROS_INFO_STREAM("" << _earliestStart);
  // ROS_INFO_STREAM("" << _task.start_after);
  // ROS_INFO_STREAM("" << startAfter);
  

	Task* t = new Task(_task.task_id,
						startAfter.toSec(),
						_task.end_before.toSec(),
						_task.max_duration.toSec(),
						_task.start_node_id,
						_task.end_node_id, 
            _demand);

	return t;
}

// bool compareTasks (const Task * i, const Task * j) { 
bool compareTasks ( Task * i,  Task * j) { 
	return (i->getExecTime()<j->getExecTime()); 
}


bool getSchedule(strands_executive_msgs::GetSchedule::Request  &req,
         			strands_executive_msgs::GetSchedule::Response &res) {
  
  ROS_INFO_STREAM("Got a request for a schedule " << req.tasks.size() << " tasks ");

  static ros::NodeHandle nh;

  static MessageStoreProxy messageStore(nh, "scheduling_problems");


  std::vector<Task*> tasks;

  vector< pair<string, string> > stored;

  // for(strands_executive_msgs::Task task : req.tasks) {
  for(auto & task : req.tasks) {
    // ROS_INFO_STREAM(task.task_id << " start " << task.start_after << ", end " << task.end_before
        // << ", duration " << task.max_duration);

    if(save_problems) {
      static string taskType(get_ros_type(task));
      stored.push_back( make_pair(taskType, messageStore.insert(task)) );
    }


    bool first = task.task_id == req.first_task ? true : false;

  	tasks.push_back(createSchedulerTask(task, req.earliest_start, first));
    if(first) {
      ROS_INFO_STREAM(task.task_id << " is first");
    }
  }


  if(save_problems) { 
    StringPairList spl;
    for(auto & pair : stored) {
      spl.pairs.push_back(ros_datacentre::makePair(pair.first, pair.second));
    }
    messageStore.insert(spl);
  }

  Scheduler scheduler(&tasks);


  if(scheduler.solve(scheduler_version, output_file, timeout)) {
  	std::sort(tasks.begin(), tasks.end(), compareTasks);

  	for(auto & tp : tasks) {
  		res.task_order.push_back(tp->getID());
  		res.execution_times.push_back(ros::Time(tp->getExecTime()));
  		delete tp;
  	} 

  }
  else {

	  // manage memory explicitly until Lenka changes to smart pointers
	  for(auto & tp : tasks) {
	  	delete tp;
	  } 

	}
  return true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "schedule_server");

  if(ros::param::has("~save_problems")) {
    ros::param::get("~save_problems", save_problems);
  } 

  if(ros::param::has("~scheduler_version")) {
    ros::param::get("~scheduler_version", scheduler_version);
    ROS_INFO_STREAM("Running scheduler version " << scheduler_version);
  } 

  if(ros::param::has("~output_file")) {
    ros::param::get("~output_file", output_file);
    ROS_INFO_STREAM("Saving experimental output to " << output_file);
  } 

  if(ros::param::has("~timeout")) {
    ros::param::get("~timeout", timeout);
    ROS_INFO_STREAM("Using timeout " << timeout);
  } 

  if(save_problems) {
    ROS_INFO("Writing scheduling problems to ros_datacentre");
  }
  else{
    ROS_INFO("Not writing scheduling problems to ros_datacentre");
  }

  	ros::NodeHandle nh;

  	ros::ServiceServer service = nh.advertiseService("get_schedule", getSchedule);
  	ROS_INFO("Ready to serve schedules");
  	ros::spin();

	return 0;
}
