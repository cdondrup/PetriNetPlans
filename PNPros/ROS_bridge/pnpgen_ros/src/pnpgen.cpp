#include <sstream>

#include <PNPgen/pnpgenerator.h>
#include "ros/ros.h"
#include "pnp_msgs/PNPGeneratePlan.h"

using namespace std;

string create_PNP_from_linear_plan(const string& plan, const string& er) {

    PNPGenerator pnpgen("linear_plan");

    // generate the main PNP from the linear plan
    pnpgen.setMainLinearPlan(plan);

    if (er!="") {
        ROS_INFO("Executions rules are not supported, yet.");
    }

    // save the PNP
    stringstream ss;
    ss << pnpgen.pnp;
    return ss.str();
}

bool add(pnp_msgs::PNPGeneratePlan::Request  &req,
         pnp_msgs::PNPGeneratePlan::Response &res)
{
    res.pnml = create_PNP_from_linear_plan(req.plan, req.er);
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pnpgen_linear");
  ros::NodeHandle n("~");

  ros::ServiceServer service = n.advertiseService("generate_plan", add);
  ROS_INFO("Ready to generate plan.");
  ros::spin();

  return 0;
}

//int main(int argc, char **argv)
//{
//    // gen_IROS15_example();

//    // gen_ICAPS16_example();
    
//    if (argc<2) {
//        cout << "    Use: " << argv[0] << " <planfile> [<erfile>]" << endl;
//        cout << "Example: " << argv[0] << " DIAG_printer.plan DIAG_printer.er" << endl;
//        return -1;
//    }
    
//    string planfile = string(argv[1]);

//    string erfile="";
//    if (argc==3)
//        erfile = string(argv[2]);

//    create_PNP_from_linear_plan(planfile,erfile);

//    return 0;
//}

