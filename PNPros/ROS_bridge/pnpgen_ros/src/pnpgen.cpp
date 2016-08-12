#include "pnpgen_ros/pnpgen.h"

namespace pnpgen_ros {
    PNPGenServer::PNPGenServer() {
        ros::NodeHandle n("~");
      
        service = n.advertiseService("generate_plan", &PNPGenServer::callback, this);
        ROS_INFO("Ready to generate plan.");
    }

    std::string PNPGenServer::createPNPFromLinearPlan(const pnpgen_ros::Plan& plan) {
        std::map<std::string, PNPExecutionRule> before, after;
        std::vector<PNPExecutionRule> rest;
        for(std::vector<PNPExecutionRule>::const_iterator it=plan.execution_rules.pnp_execution_rule_array.begin();
            it != plan.execution_rules.pnp_execution_rule_array.end(); ++it) {
            if(it->timing == PNPExecutionRule::BEFORE) {
                before[it->action_name] = *it;
            } else if (it->timing == PNPExecutionRule::AFTER) {
                after[it->action_name] = *it;
            } else {
                rest.push_back(*it);
            }
        }
    
        pnpgen = new PNPGenerator(generateUUID());
        Place *p = pnpgen->pnp.pinit;

        for(std::vector<pnpgen_ros::PNPActionArray>::const_iterator it = plan.actions.begin();
                it != plan.actions.end(); ++it) {
            // Add fork and join transitions in case of multiple concurrent actions
            Transition *tf, *tj; int y = 0;
            if(it->pnp_action_array.size() > 1) {
                // fork transition
                tf = pnpgen->pnp.addTransition("[]"); tf->setX(p->getX()+1); tf->setY(p->getY());
                pnpgen->pnp.connect(p,tf);
                tj = pnpgen->pnp.addTransition("[]");
            }
            
            // Add actions
            for(std::vector<pnpgen_ros::PNPAction>::const_iterator action = it->pnp_action_array.begin();
                action != it->pnp_action_array.end(); ++action) {
                // Insert an initial place connected with the fork in case of multiple concurrent actions
                if(it->pnp_action_array.size() > 1) {
                    // initial places of actions
                    Place *pi1 = pnpgen->pnp.addPlace("X",-1); pi1->setX(tf->getX()+1); pi1->setY(tf->getY()-y);
                    pnpgen->pnp.connect(tf,pi1);
                    p = pi1;
                    y-=3;
                }
                
                // Add action: timed or normal
                std::string name = action->name;
                for(std::vector<std::string>::const_iterator param = action->parameters.begin();
                    param != action->parameters.end(); ++param) {
                    // Create param string
                    name += "_"+(*param);
                }
                ROS_INFO_STREAM("Adding " << name << " action " << action->duration);
                Place *start = p;
                p = addAction(name, p, action->duration);
                
                if(after.find(action->name) != after.end()) {
                    std::vector<std::string> o;
                    o.push_back(after[action->name].condition);
                    o.push_back("(not "+after[action->name].condition+")");
                    std::vector<Place*> res = pnpgen->addSensingAction("testBla", p, o);
                    p = res[0];
                    if(after[action->name].recovery.pnp_action_array[0].name == "fail_plan") {
                        res[1]->setName("fail");
                    } else {
                        Transition *t = pnpgen->pnp.addTransition("[]"); t->setX(res[1]->getX()+1); t->setY(res[1]->getY());
                        pnpgen->pnp.connect(res[1], t); pnpgen->pnp.connect(t, start);
                    }
                }
                
                // Connect end place with join transition in case of multiple concurrent actions
                if(it->pnp_action_array.size() > 1) {
                    pnpgen->pnp.connect(p,tj);
                }
            }
            
            // Move join transtion after the actions and create an end place after the join in case of multiple concurrent actions
            if(it->pnp_action_array.size() > 1) {
                tj->setX(p->getX()+1); tj->setY(p->getY());
                p = pnpgen->pnp.addPlace("X",-1); p->setX(tj->getX()+1); p->setY(tj->getY()+1);
                pnpgen->pnp.connect(tj,p);
            }
        }
        
        // Set the name of last place to goal
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

