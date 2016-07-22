# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 09:42:29 2016

@author: Christian Dondrup
"""

import rospy
import rosservice
from pnp_msgs.srv import PNPRegisterServer, PNPUnregisterServer


def get_plugin_server_service_name(service_type):
    try:
        return rosservice.rosservice_find(service_type)[0]
    except IndexError:
        if not rospy.core.is_shutdown_requested():
            rospy.logwarn("No service found. Retrying in 1 second")
            rospy.sleep(1.)
            return get_plugin_server_service_name(service_type)
            
def unregister_plugin_client(name):
    s = rospy.ServiceProxy(get_plugin_server_service_name(PNPUnregisterServer._type), PNPUnregisterServer)
    try:            
        s.wait_for_service(timeout=1.)
        s(name)
    except rospy.ROSException:
        rospy.logwarn("Unregestireing unsuccessful. '%s' might still be registered with server." % name)

def register_plugin_client(name):
    s = rospy.ServiceProxy(get_plugin_server_service_name(PNPRegisterServer._type), PNPRegisterServer)
    try:
        s.wait_for_service(timeout=1.)
        s(name)
    except rospy.ROSException:
        rospy.logwarn("Something went horribly wrong when trying to register the '%s' client. Did the server die?" % name)
            