#include "pnpgen_ros/pnpgen.h"

namespace pnpgen_ros {
    PNPGenServer::PNPGenServer() {
        ros::NodeHandle n("~");
      
        service = n.advertiseService("generate_plan", &PNPGenServer::callback, this);
        ROS_INFO("Ready to generate plan.");
    }

    std::string PNPGenServer::createPNPFromLinearPlan(const pnpgen_ros::Plan& plan) {
    
        pnpgen = new PNPGenerator(generateUUID());
        Place *p = pnpgen->pnp.pinit;

        for(std::vector<pnpgen_ros::PNPActionArray>::const_iterator it = plan.actions.begin();
                it != plan.actions.end(); ++it) {
            Transition *tf, *tj; int y = 0;
            if(it->pnp_action_array.size() > 1) {
                // fork transition
                tf = pnpgen->pnp.addTransition("[]"); tf->setX(p->getX()+1); tf->setY(p->getY());
                pnpgen->pnp.connect(p,tf);
                tj = pnpgen->pnp.addTransition("[]");
            }
            for(std::vector<pnpgen_ros::PNPAction>::const_iterator action = it->pnp_action_array.begin();
                action != it->pnp_action_array.end(); ++action) {
                if(it->pnp_action_array.size() > 1) {
                    // initial places of actions
                    Place *pi1 = pnpgen->pnp.addPlace("X",-1); pi1->setX(tf->getX()+1); pi1->setY(tf->getY()-y);
                    pnpgen->pnp.connect(tf,pi1);
                    p = pi1;
                    y-=3;
                }
                std::string name = action->name;
                for(std::vector<std::string>::const_iterator param = action->parameters.begin();
                    param != action->parameters.end(); ++param)
                    name += "_"+(*param);
                ROS_INFO_STREAM("Adding " << name << " action " << action->duration);
                p = addAction(name, p, action->duration);
                if(it->pnp_action_array.size() > 1) {
                    pnpgen->pnp.connect(p,tj);
                }
            }
            if(it->pnp_action_array.size() > 1) {
                tj->setX(p->getX()+1); tj->setY(p->getY());
                p = pnpgen->pnp.addPlace("X",-1); p->setX(tj->getX()+1); p->setY(tj->getY()+1);
                pnpgen->pnp.connect(tj,p);
            }
        }
        p->setName("goal");
    
        // return the PNP
        stringstream ss;
        ss << pnpgen->pnp;
        return ss.str();
    }
    
    bool PNPGenServer::callback(pnpgen_ros::PNPGeneratePlan::Request &req, pnpgen_ros::PNPGeneratePlan::Response &res)
    {
        res.pnml = createPNPFromLinearPlan(req.plan);
        return true;
    }

    Place* PNPGenServer::addAction(std::string name, Place *p, int duration) {
        if (duration>0) {
            Place *poa; // place to save in the stack
            p = pnpgen->pnp.addTimedAction(name,p,duration,&poa);
            pnpgen->addActionToStacks(name,poa);
        }
        else {
            Place *poa=p; // place to save in the stack
            p = pnpgen->pnp.addAction(name,p);
            pnpgen->addActionToStacks(name,poa);
        }
        return p;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pnpgen");
    pnpgen_ros::PNPGenServer p = pnpgen_ros::PNPGenServer();
    ros::spin();
    return 0;
}

