#include "pnpgen_ros/pnpgen.h"

namespace pnpgen_ros {
    PNPGenServer::PNPGenServer() {
        ros::NodeHandle n("~");
      
        service = n.advertiseService("generate_plan", &PNPGenServer::callback, this);
        ROS_INFO("Ready to generate plan.");
    }

    std::string PNPGenServer::createPNPFromLinearPlan(const pnpgen_ros::Plan& plan) {
        // Sorting execution rules based on timing. Multiple execution rules per action and timing are possible
        std::map<std::string, PNPExecutionRuleArray> before, after;
        std::vector<PNPExecutionRule> rest;
        for(std::vector<PNPExecutionRule>::const_iterator it=plan.execution_rules.pnp_execution_rule_array.begin();
            it != plan.execution_rules.pnp_execution_rule_array.end(); ++it) {
            if(it->timing == PNPExecutionRule::BEFORE) {
                before[it->action_name].pnp_execution_rule_array.push_back(*it);
            } else if (it->timing == PNPExecutionRule::AFTER) {
                after[it->action_name].pnp_execution_rule_array.push_back(*it);
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
                
                /***********************************************************************************************************
                 Recovery behaviours to be executed before an action has been started.
                ************************************************************************************************************/
                int y = 3;
                std::vector<Transition*> skip;
                if(before.find(action->name) != before.end()) {
                    int cnt = 0;
                    for(std::vector<PNPExecutionRule>::const_iterator er = before[action->name].pnp_execution_rule_array.begin();
                        er != before[action->name].pnp_execution_rule_array.end(); ++er) { // For each 'after' rule of that action
                        // Add a sensing action with two outcomes: user specified | 'not' user specified
                        std::vector<Place*> res = addBinarySensingAction("before"+num_to_str<int>(cnt)+action->name, er->condition, p);
                        p = res[0]; // Success place
                        // Error handling of specific keywords
                        if(er->recovery.pnp_action_array[0].name == "fail_plan") {
                            res[1]->setName("fail");
                        } else if(er->recovery.pnp_action_array[0].name == "restart_action"){
                            ROS_WARN("Restarting an action before it has been started is not supported. No recovery added.");
                        } else if(er->recovery.pnp_action_array[0].name == "restart_plan"){
                            Transition *t = pnpgen->pnp.addTransition("[]"); t->setX(res[1]->getX()+1); t->setY(res[1]->getY());
                            pnpgen->pnp.connect(res[1], t); pnpgen->pnp.connect(t, pnpgen->pnp.pinit);
                        } else if(er->recovery.pnp_action_array[0].name == "skip_action"){
                            Transition *t = pnpgen->pnp.addTransition("[]"); t->setX(res[1]->getX()+1); t->setY(res[1]->getY());
                            skip.push_back(t);
                        } else { // Custom recovery plan
                            Place *re_p = res[1];
                            re_p->setX(p->getX()); re_p->setY(re_p->getY()+y);
                            for(std::vector<PNPAction>::const_iterator re = er->recovery.pnp_action_array.begin();
                                re != er->recovery.pnp_action_array.end(); ++re) {
                                // Add recovery action: timed or normal
                                cout << "Adding custom recovery action" << endl;
                                re_p = addAction(*re, re_p);
                            }
                            Transition *t2 = pnpgen->pnp.addTransition("[]"); t2->setX(re_p->getX()+1); t2->setY(re_p->getY());
                            pnpgen->pnp.connect(re_p, t2); pnpgen->pnp.connect(t2, p);
                        }
                    y += 4; cnt++;
                    }
                }
                /***********************************************************************************************************/
                
                /***********************************************************************************************************
                 Add action: timed or normal
                ************************************************************************************************************/
                Place *start = p; // Save starting place for possible recovery
                p = addAction(*action, p);
                // Handel skipping the action
                for(std::vector<Transition*>::const_iterator s = skip.begin(); s != skip.end(); ++s) {
                    pnpgen->pnp.connect(*s, p);
                }
                /***********************************************************************************************************/
                
                /***********************************************************************************************************
                 Recovery behaviours to be executed after an action has been completed.
                ************************************************************************************************************/
                if(after.find(action->name) != after.end()) {
                    int cnt = 0;
                    for(std::vector<PNPExecutionRule>::const_iterator er = after[action->name].pnp_execution_rule_array.begin();
                        er != after[action->name].pnp_execution_rule_array.end(); ++er) { // For each 'after' rule of that action
                        // Add a sensing action with two outcomes: user specified | 'not' user specified
                        std::vector<Place*> res = addBinarySensingAction("after"+num_to_str<int>(cnt)+action->name, er->condition, p);
                        p = res[0]; // Success place
                        // Error handling of specific keywords
                        if(er->recovery.pnp_action_array[0].name == "fail_plan") {
                            res[1]->setName("fail");
                        } else if(er->recovery.pnp_action_array[0].name == "restart_action"){
                            Transition *t = pnpgen->pnp.addTransition("[]"); t->setX(res[1]->getX()+1); t->setY(res[1]->getY());
                            pnpgen->pnp.connect(res[1], t); pnpgen->pnp.connect(t, start);
                        } else if(er->recovery.pnp_action_array[0].name == "restart_plan"){
                            Transition *t = pnpgen->pnp.addTransition("[]"); t->setX(res[1]->getX()+1); t->setY(res[1]->getY());
                            pnpgen->pnp.connect(res[1], t); pnpgen->pnp.connect(t, pnpgen->pnp.pinit);
                        } else if(er->recovery.pnp_action_array[0].name == "skip_action"){
                            ROS_WARN("Skipping an action after it has been completed is not supported. No recovery added.");
                        } else { // Custom recovery plan
                            Place *re_p = res[1];
                            re_p->setX(start->getX()); re_p->setY(re_p->getY()+y);
                            for(std::vector<PNPAction>::const_iterator re = er->recovery.pnp_action_array.begin();
                                re != er->recovery.pnp_action_array.end(); ++re) {
                                // Add recovery action: timed or normal
                                cout << "Adding custom recovery action" << endl;
                                re_p = addAction(*re, re_p);
                            }
                            Transition *t2 = pnpgen->pnp.addTransition("[]"); t2->setX(re_p->getX()+1); t2->setY(re_p->getY());
                            pnpgen->pnp.connect(re_p, t2); pnpgen->pnp.connect(t2, p);
                        }
                    y += 4; cnt++;
                    }
                }
                /***********************************************************************************************************/
                
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
        
        // Apply during execution rules
        for(std::vector<PNPExecutionRule>::const_iterator du = rest.begin(); du != rest.end(); ++du) {
            std:: string recovery = "";
            if(du->recovery.pnp_action_array[0].name == "fail_plan"
                    || du->recovery.pnp_action_array[0].name == "restart_plan"
                    || du->recovery.pnp_action_array[0].name == "restart_action"
                    || du->recovery.pnp_action_array[0].name == "skip_action") {
                recovery = du->recovery.pnp_action_array[0].name + " ";
            } else {
                for(std::vector<PNPAction>::const_iterator a = du->recovery.pnp_action_array.begin(); 
                    a != du->recovery.pnp_action_array.end(); ++a) {
                    recovery += generateActionName(a->name, a->parameters);
                    if(a->duration > 0)
                        recovery += "|" + num_to_str<int>(a->duration);
                    recovery += ";";
                }
            }
            cout << "Adding execution rule: " << du->action_name << " " << du->condition << " " << recovery.substr(0, recovery.size()-1) << endl;
            pnpgen->addER(du->action_name, du->condition, recovery.substr(0, recovery.size()-1));
        }
        pnpgen->applyExecutionRules();
    
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

    Place* PNPGenServer::addAction(PNPAction action, Place *p) {
        std::string name = generateActionName(action.name, action.parameters);
        ROS_INFO_STREAM("Adding " << name << " action " << action.duration);
        if (action.duration>0) {
            Place *poa; // place to save in the stack
            p = pnpgen->pnp.addTimedAction(name,p,action.duration,&poa);
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

