#include <sstream>

#include <PNPgen/pnpgenerator.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "ros/ros.h"
#include "pnpgen_ros/PNPGeneratePlan.h"
#include "pnpgen_ros/Plan.h"

namespace pnpgen_ros {
    class PNPGenServer
    {
    public:
        PNPGenServer();
        std::string createPNPFromLinearPlan(const Plan &plan);
        bool callback(pnpgen_ros::PNPGeneratePlan::Request  &req, pnpgen_ros::PNPGeneratePlan::Response &res);
        Place *addAction(std::string name, Place *p, int duration=0);
        
    private:
        inline std::string generateUUID() {
            return num_to_str<boost::uuids::uuid>(boost::uuids::random_generator()());
        }
  
        template<typename T>
        inline std::string num_to_str(T num) {
            std::stringstream ss;
            ss << num;
            return ss.str();
        }
        
        ros::ServiceServer service;
        PNPGenerator *pnpgen;
    };
}

