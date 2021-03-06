# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 09:42:29 2016

@author: Christian Dondrup
"""

import rospy
import rosservice
import socket
from pnp_msgs.srv import PNPRegisterServer, PNPUnregisterServer


def custom_rosservice_find(service_type):
    """
    Lookup services by service_type. Exteing functionality of rosservice.rosservice_find
    @param service_type: type of service to find
    @type  service_type: str
    @return: list of service names that use service_type
    @rtype: [str]
    """
    master = rosservice._get_master()
    matches = []
    try:
        _, _, services = master.getSystemState()
        for s, l in services:
            try:
                t = rosservice.get_service_type(s)
            except rosservice.ROSServiceIOException as e:
                rospy.logdebug(e)
            else:
                if t == service_type:
                    matches.append(s)
    except socket.error:
        raise rosservice.ROSServiceIOException("Unable to communicate with master!")
    return matches

def find_service_by_type(service_type):
    while True:
        try:
            return custom_rosservice_find(service_type)[0]
        except IndexError:
            rospy.logwarn("No service with type '%s' found. Retrying in 1 second. Enable debug mode for more information." % service_type)
            if rospy.core.is_shutdown_requested():
                return
            rospy.sleep(1.)

def unregister_plugin_client(name):
    s = rospy.ServiceProxy(find_service_by_type(PNPUnregisterServer._type), PNPUnregisterServer)
    try:
        s.wait_for_service(timeout=1.)
        s(name)
    except rospy.ROSException:
        rospy.logwarn("Unregistering unsuccessful. '%s' might still be registered with server." % name)

def register_plugin_client(name):
    s = rospy.ServiceProxy(find_service_by_type(PNPRegisterServer._type), PNPRegisterServer)
    try:
        s.wait_for_service(timeout=1.)
        s(name)
    except rospy.ROSException:
        rospy.logwarn("Something went horribly wrong when trying to register the '%s' client. Did the server die?" % name)
