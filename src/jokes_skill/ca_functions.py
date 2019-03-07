#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Sergio Gonzalez Diaz"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Sergio Gonzalez Diaz"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Sergio Gonzalez Diaz"
__email__ = "sergigon@ing.uc3m.es"
__status__ = "Development"

import time
import rospy

from std_msgs.msg import String, Int16, Empty
from interaction_msgs.msg import CA
from common_msgs.msg import KeyValuePair as kvpa

############################### Giving info CA functions ###############################
def makeCA_info(etts_text=None, language='es', gesture=None, image_url=None, tablet_type=None, priority=1, duration=0, emitter='', type_ca=''):
    '''
    Creates info CA.
    '''

    msg = CA()
    msg.type = "robot_giving_info"
    msg.ca_name = str(rospy.get_rostime().nsecs)
    msg.duration = duration
    msg.priority = priority
    msg.emitter = emitter

    if etts_text:
        kvp = kvpa()
        kvp.key = "etts_text"
        kvp.value = etts_text
        msg.values.append(kvp)

        kvp = kvpa()
        kvp.key = "etts_language"
        kvp.value = language
        msg.values.append(kvp)

    if gesture:
        kvp = kvpa()
        kvp.key = "gesture"
        kvp.value = gesture
        msg.values.append(kvp)

    if image_url and tablet_type:
        kvp = kvpa()
        kvp.key = "tablet_type"
        kvp.value = tablet_type
        msg.values.append(kvp)

        kvp = kvpa()
        kvp.key = "tablet_url"
        kvp.value = image_url
        msg.values.append(kvp)

    rospy.logdebug("Created CA %s info %s" % (type_ca, msg.ca_name))

    return msg

def makeCA_etts_info(etts_text, language='es', priority=1, duration=0, emitter=''):
    '''
    Creates etts info CA.
    '''

    msg = makeCA_info(etts_text=etts_text, language=language, priority=priority, duration=duration, emitter=emitter, type_ca='etts')

    return msg

def makeCA_tablet_info(image_url, tablet_type, priority=1, duration=0, emitter=''):
    '''
    Creates tablet info CA.
    '''

    msg = makeCA_info(image_url=image_url, tablet_type=tablet_type, priority=priority, duration=duration, emitter=emitter, type_ca='tablet')

    return msg

def makeCA_defaultImage(duration=0, priority=1, emitter=""):
    '''
    Creates default tablet image CA.
    '''

    msg = makeCA_tablet_info(image_url='image/default_images/default_2.png', tablet_type='image', priority=priority, duration=duration, emitter=emitter, type_ca='tablet default')

    return msg

def makeCA_gesture_info(gesture, priority=1, duration=0, emitter=''):
    '''
    Creates gesture info CA.
    '''

    msg = makeCA_info(gesture=gesture, priority=priority, duration=duration, emitter=emitter, type_ca='gesture')

    return msg
#======================================================================================#

############################### Asking info CA functions ###############################
def makeCA_question(answer_type, answer_id='', answer_time=0, answer_attempts=2, grammar=None,
    etts_text=None, language='es', image_url=None, tablet_type=None, gesture=None, menu_value=None, menu_type=None,
    duration=0, priority=1, emitter="", type_ca=''):
    '''
    Creates question CA.
    '''

    msg = CA()
    msg.type = "robot_asking_for_info"
    msg.ca_name = str(rospy.get_rostime().nsecs)
    msg.duration = duration
    msg.priority = priority
    msg.emitter = emitter

    ### Giving info ###
    if etts_text:
        kvp = kvpa()
        kvp.key = "etts_text"
        kvp.value = str(etts_text)
        msg.values.append(kvp)

        kvp = kvpa()
        kvp.key = "etts_language"
        kvp.value = language
        msg.values.append(kvp)

    if image_url and tablet_type:
        kvp = kvpa()
        kvp.key = "tablet_url"
        kvp.value = image_url
        msg.values.append(kvp)

        kvp = kvpa()
        kvp.key = "tablet_type"
        kvp.value = tablet_type
        msg.values.append(kvp)

    if gesture:
        kvp = kvpa()
        kvp.key = "gesture"
        kvp.value = gesture
        msg.values.append(kvp)

    if menu_value and menu_type:
        kvp = kvpa()
        kvp.key = "menu_value"
        kvp.value = str(menu_value)
        msg.values.append(kvp)

        kvp = kvpa()
        kvp.key = "menu_type"
        kvp.value = str(menu_type)
        msg.values.append(kvp)
    #=================#

    ## Listen answer ##
    kvp = kvpa()
    kvp.key = "answer_type"
    kvp.value = answer_type
    msg.values.append(kvp)

    if grammar:
        kvp = kvpa()
        kvp.key = "grammar"
        kvp.value = grammar
        msg.values.append(kvp)

    kvp = kvpa()
    kvp.key = "answer_id"
    kvp.value = answer_id
    msg.values.append(kvp)

    if answer_time != 0:
        kvp = kvpa()
        kvp.key = "answer_time"
        kvp.value = str(answer_time)
        msg.values.append(kvp)

    kvp = kvpa()
    kvp.key = "answer_attempts"
    kvp.value = str(answer_attempts)
    msg.values.append(kvp)
    #=================#

    rospy.logdebug("Creating CA %s question %s" % (type_ca, msg.ca_name))

    return msg

def makeCA_ASR_question(etts_text, grammar, answer_id, language='es', image_url=None, tablet_type=None, gesture=None,
    answer_time=10, answer_attempts=2,
    duration=0, priority=1, emitter=""):
    '''
    Creates etts question CA.
    '''

    msg = makeCA_question(answer_type='ASR', grammar=grammar, answer_id=answer_id, answer_time=answer_time, answer_attempts=answer_attempts,
        etts_text=etts_text, language=language, image_url=image_url, tablet_type=tablet_type, gesture=gesture,
        duration=duration, priority=priority, emitter=emitter, type_ca='ASR')

    return msg

def makeCA_tablet_question(menu_value, menu_type, etts_text=None, language='es', gesture=None,
    answer_time=0, answer_attempts=2,
    duration=0, priority=1, emitter=""):
    '''
    Creates tablet question CA.
    '''

    msg = makeCA_question(answer_type='tablet_menu', answer_id='tablet_answer', answer_time=answer_time, answer_attempts=answer_attempts,
        etts_text=etts_text, language=language, gesture=gesture, menu_value=menu_value, menu_type=menu_type,
        duration=duration, priority=priority, emitter=emitter, type_ca='tablet')

    return msg

def makeCA_touch_question(etts_text=None, language='es', image_url=None, tablet_type=None, gesture=None,
    answer_time=10, answer_attempts=2,
    duration=0, priority=1, emitter=""):
    '''
    Creates touch question CA.
    '''

    msg = makeCA_question(answer_type='touch', answer_id='touch_answer', answer_time=answer_time, answer_attempts=answer_attempts,
        etts_text=etts_text, language=language, image_url=image_url, tablet_type=tablet_type, gesture=gesture,
        duration=duration, priority=priority, emitter=emitter, type_ca='touch')

    return msg

def makeCA_ASR_listen(grammar, answer_id, answer_time=10, answer_attempts=1,
    duration=0, priority=1, emitter=""):
    '''
    Creates listen question CA. Only listens, does not give info.
    '''

    msg = makeCA_question(answer_type='ASR', grammar=grammar, answer_id=answer_id, answer_time=answer_time, answer_attempts=answer_attempts,
        duration=duration, priority=priority, emitter=emitter, type_ca='ASR listen')

    return msg
#======================================================================================#

######################### Giving info and waiting CA functions #########################
def makeCA_say_and_wait(etts_text, grammar, answer_id, language='es', image_url= None, tablet_type=None, gesture=None, answer_time=6, duration=0, priority=1, emitter=""):
    '''
    Creates say and wait CA. Says info and waits for info, if given.
    '''

    msg = CA()
    msg.type = "robot_gives_info_and_waits"
    msg.ca_name = str(rospy.get_rostime().nsecs)
    msg.duration = duration
    msg.priority = priority
    msg.emitter = emitter

    ### Giving info ###
    kvp = KeyValuePair()
    kvp.key = "etts_text"
    kvp.value = str(etts_text)
    msg.values.append(kvp)

    kvp = KeyValuePair()
    kvp.key = "etts_language"
    kvp.value = language
    msg.values.append(kvp)

    if image_url and tablet_type:
        kvp = kvpa()
        kvp.key = "tablet_type"
        kvp.value = tablet_type
        msg.values.append(kvp)

        kvp = kvpa()
        kvp.key = "tablet_url"
        kvp.value = image_url
        msg.values.append(kvp)

    if gesture:
        kvp = kvpa()
        kvp.key = "gesture"
        kvp.value = gesture
        msg.values.append(kvp)
    #=================#

    ## Listen answer ##
    kvp = KeyValuePair()
    kvp.key = "answer_type"
    kvp.value = "ASR"
    msg.values.append(kvp)

    kvp = KeyValuePair()
    kvp.key = "answer_id"
    kvp.value = answer_id               
    msg.values.append(kvp)

    kvp = KeyValuePair()
    kvp.key = "answer_time"
    kvp.value = str(answer_time)                
    msg.values.append(kvp)

    kvp = KeyValuePair()
    kvp.key = "grammar"
    kvp.value = grammar
    msg.values.append(kvp)
    #=================#

    rospy.logdebug("Creating CA say and wait %s" % msg.ca_name)
    
    return msg
#======================================================================================#

############################### Deactivation CA functions ##############################
def deactivateCA(ca_name):
    '''
    Creates deactivation CA.
    '''

    msg = String()
    msg.data = ca_name

    rospy.logdebug("Creating CA deactivation: %s" % ca_name)

    return msg
#======================================================================================#