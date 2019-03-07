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

# Skill libraries
from skill.skill import Skill, ActionlibException, CONDITIONAL
import rospkg
import rospy
import roslib
import importlib
import actionlib

# Messages
import jokes_skill.msg
from std_msgs.msg import String, Empty
from interaction_msgs.msg import CA
from hri_manager.key_value_pairs import to_dict

# Local libraries
from xml_reader import XMLReader
from ca_functions import *
from exceptions_lib import PauseException, ErrorException

# Skill variables
# Package name
pkg_name = 'jokes_skill'
# Skill name (declare this only if the name is different of 'pkg_name')
skill_name = "jokes_skill"


class JokesSkill(Skill):
    """
    Jokes skill class.
    """

    # Feedback and result of this skill
    _feedback = jokes_skill.msg.JokesFeedback()
    _result = jokes_skill.msg.JokesResult()
    _emitter = 'jokes_ca'

    # Constants

    # Params constants
    _LANG_PARAM = 'context/user/language'
    _USER_NAME_PARAM = 'context/user/name'


    def __init__(self):
        """
        Init method.
        """

        # Class variables
        self._as = None # SimpleActionServer variable
        self._list_ca = [] # List of sended CAs
        
        self.init_variables() # Init variables

        # Local paths
        rospack = rospkg.RosPack()
        self._pkg_path = rospack.get_path(pkg_name) + '/' # Package path
        self._data_path = self._pkg_path + 'data/' # Data path

        # Tablet paths
        #self._icons_path = 'image/weather/' # Icons path

        # XMl reader
        self._xml_reader = XMLReader(self._data_path)

        # jokes variables
        #self.location, self.forecast_type, self.date = '', '', ''

        # init the skill
        Skill.__init__(self, skill_name, CONDITIONAL)

    def init_variables(self):
        """
        Initialization of variables.
        """

        # Goal control variables
        self._goal_exec = False # Indicates if goal is being handled
        self._pause_requested = False # Indicates if pause is requested
        self._pause = False # Indicates if pause
        self._step = '' # Indicates new process step
        self._exec_out = True # Indicates goal exec loop exit
        
        # Goal varaibles
        self._max_time,  self._time_run = 0, 0 # Max time to handle the goal; time running handling goal
        self._t0 = 0 # Variables to update self._time_run
        self._number_plays, self._i_plays = 0, 0 # Max number of plays to handle the goal; plays handling goal
        self._limit_method = '' # Indicates limit method selected (max time, plays or both)
        self._time_question = 10000 # Time to make a proactivity question

        # CA variables
        self._answer_received = '' # CA answer received

######################### Skill Callbacks #########################
    def response_callback(self, recog_response): ## No modify ##
        """
        Receive the response from an asking for info CA.
        """

        if(recog_response.emitter == self._emitter):
            values = to_dict(recog_response.values)
            ans = values['answer_value'].lower() # Turn it into lower case

            rospy.logdebug("Received response: " + ans)

            if ans in ["communicationproblems", "no_response"]: # If there's some problem or user not response
                self._answer_received = 'Error'
            else:
                self._answer_received = ans

        else:
            rospy.logwarn('Response not expected')

    def pause_exec(self): ## No modify ##
        """
        Modify the variable self._pause if goal is being handled, when a pause is requested.
        """

        if(self._goal_exec and not self._pause): # Goal being handled and not in pause
            self._pause_requested = True
            rospy.logdebug('Pause requested')
        elif(self._pause):
            rospy.logdebug('Already in pause')
        else: # Goal NOT being handled
            rospy.logwarn('Goal not being handled')

    def resume_exec(self): ## No modify ##
        """
        Modify the variable self._pause, when a resume is requested.
        """

        self._pause = False
        self._feedback.app_status = 'resume_ok'
        self._as.publish_feedback(self._feedback)

    def create_msg_srv(self):
        """
        Callback called when skill is started.

        @raise rospy.ROSException: if the service is inactive.
        """
        rospy.loginfo("Start requested")

        # publishers and subscribers
        self.ca_pub = rospy.Publisher(
            "hri_manager/ca_activations", CA, queue_size=1) # CA publisher
        self.ca_deactivation_pub = rospy.Publisher(
            "hri_manager/ca_deactivations", String, queue_size=1) # CA deactivation publisher

        self.sub_response = rospy.Subscriber(
            "hri_manager/response", CA, self.response_callback) # CA response

        #self.sub_response = rospy.Subscriber(robot + "hri_manager/response", CA, self.response_callback) # CA subscriber

        # servers and clients

        # If actionlib server has not been initialized
        if not self._as:
            self._as = actionlib.SimpleActionServer(skill_name, jokes_skill.msg.JokesAction, execute_cb=self.execute_cb, auto_start=False)
            # start the action server
            self._as.start()

    def shutdown_msg_srv(self):
        """
        Callback called when skill is stopped.
        """

        # publishers and subscribers
        # FIXME: do not unregister publishers because a bug in ROS
        # self.__test_pub.unregister()

        rospy.loginfo("Stop requested")
        # servers and clients
        self.sub_response.unregister()

        # Cancel goal
        self._as.preempt_request = True
#=================================================================#

######################### Skill Functions #########################
    def stop_etts(self): ## No modify ##
        """
        Stops active etts CA.
        """

        rospy.logdebug()
        ca_info = makeCA_etts_info(etts_text='\\pause=10', emitter=self._emitter)
        self.ca_pub.publish(ca_info)
        ca_deactivation_msg = deactivateCA(ca_info.ca_name)
        self.ca_deactivation_pub.publish(ca_deactivation_msg)

    def deactivate_ca_list(self): ## No modify ##
        """
        Deactivate all CAs in the list.
        """

        for ca_name in self._list_ca:
            msg = deactivateCA(ca_name)
            self.ca_deactivation_pub.publish(msg)
        self._list_ca = []

        # Stop etts
        self.stop_etts()

    def update_list_ca(self, list_ca): ## No modify ##
        """
        Update the list of active CAs.

        @param ca_list: List of CAs.
        """

        for ca_name in list_ca:
            rospy.logdebug('Adding CA: %s to CA list' % ca_name)
        self._list_ca.extend(list_ca)

    def wait_ca_finish(self, ca_name, max_time=60): ## No modify ##
        """
        Enters a loop until the CA has finished, or a exception has been called.

        @param ca_name: Name of the CA.
        @param max_time: Max time to for the CA.
        """

        rospy.logdebug('Waiting CA %s to finish' % ca_name)
        t0, t1 = time.time(), time.time()
        #while(t1-t0<max_time and ca_finished):
        while(t1-t0<max_time):
            # Check exceptions
            self.exception_check()
            rospy.sleep(1)
            t1=time.time()
        rospy.logdebug('CA finished')

    def wait_ca_answer(self, ca_info): ## No modify ##
        """
        Enters a loop until the CA has received an answer, or a exception has been called.

        @param ca_info: CA message.
        """

        rospy.logdebug('Waiting CA %s response' % ca_info.ca_name)
        values = to_dict(ca_info.values)
        t0, t1 = time.time(), time.time()
        while(self._answer_received == '' and t1-t0<(int(values['answer_time'])+4) * int(values['answer_attempts'])):
            rospy.sleep(1)
            t1=time.time()
        rospy.logdebug('Response received')

    def exception_check(self): ## No modify ##
        """
        Checks if an exception has been asked. It can be by a preempt request or by a pause request.
        """

        # Raise exceptions
        ############# State Preempted checking #############
        # If goal is in Preempted state (that is, there    #
        # is a goal in the queue or the actual goal is     #
        # cancelled), the exception is activated.          #
        ####################################################
        if(self._as.is_preempt_requested()):
            rospy.logwarn("Preempt requested")
            raise ActionlibException
        
        ############### State Pause checking ###############
        if(self._pause_requested):
            self._pause_requested = False
            self._pause = True
            rospy.logwarn('Raising Pause Exception')
            raise PauseException

    def pause_wait(self): ## No modify ##
        """
        Pause loop. Finishes when goal is resumed or cancelled.
        """

        if(self._pause and not self._as.is_preempt_requested()):
            rospy.loginfo('Start waiting')
            # Send feedback
            self._feedback.app_status = 'pause_ok'
            self._as.publish_feedback(self._feedback)
            # Wait loop
            while(self._pause and not self._as.is_preempt_requested()):
                rospy.logdebug('waiting...')
                rospy.sleep(1)

    def update_percentage(self): ## No modify ##
        """
        Update the skill percentage.
        """
        percentage_plays, percentage_time = 0, 0

        if(self._limit_method == 'both' or self._limit_method == 'plays'):
            percentage_plays = int((float(self._i_plays)/float(self._number_plays))*100)
            rospy.logdebug('percentage plays: %s' % percentage_plays)
        if(self._limit_method == 'both' or self._limit_method == 'time'):
            percentage_time = int((float(self._time_run)/float(self._max_time))*100)
            rospy.logdebug('percentage time: %s' % percentage_time)

        percentage = percentage_plays if percentage_plays > percentage_time else percentage_time
        percentage = percentage if percentage<100 else 100
        rospy.loginfo('percentage: %s' % percentage)
        self._feedback.percentage_completed = percentage

    def get_param(self, param_name): ## No modify ##
        """
        Get the param from the context manager params server.
        """

        # Language
        if(param_name == 'langauge'):
            try:
                language = rospy.get_param(self._LANG_PARAM) # Get langauge
            except KeyError:
                language = 'es'
                rospy.logwarn("Language not found. Using language %s" % language)
            return language

        # User name
        elif(param_name == 'user_name'):
            try:
                user_name = rospy.get_param(self._USER_NAME_PARAM) # Get user name
            except KeyError:
                user_name = ''
                rospy.logwarn("User name not found. Using empty name")
            return user_name

        # Unknow param name
        else:
            rospy.logerr("Param name '%s' not found. %s" % param_name)
            return -1
#=================================================================#

######################### Timer Functions #########################
    def start_timer(self): ## No modify ##
        """
        Starts timer. Update t0 and initialize time_run.
        """

        self._t0 = time.time()
        self._time_run = 0

    def restart_timer(self): ## No modify ##
        """
        Restart timer. Update t0.
        """

        self._t0 = time.time()

    def update_timer(self): ## No modify ##
        """
        Update timer. Update time_run.
        """

        self._time_run += time.time() - self._t0
#=================================================================#

########################## Goal Handling ##########################
    def goal_handler(self, goal):
        """
        Goal handler.

        Checks if the goal is appropriate. If True, it configures the variables for the goal execution.

        @param goal: Goal received.

        @return goal_accepted: True if goal is accepted, else, False.
        """

        # Fill goal variables
        # -- Skill command -- #
        skill_command_vec = goal.skill_command.split('/') # Divides goal by fields
        if(len(skill_command_vec)<2):
            rospy.logerr('NOT enough fields')
            return False
        self._type = skill_command_vec[0] # saying, joke
        self._categories = skill_command_vec[1] # short, long
        # -- Max time -- #
        self._max_time = goal.max_time
        # -- Number plays -- #
        self._number_plays = goal.number_plays
        
        # Check proactivity
        if(goal.proactivity == 1):
            self._time_question = 120 # Get time question
        elif(goal.proactivity == 2):
            self._time_question = 90
        elif(goal.proactivity == 3):
            self._time_question = 60
        elif(goal.proactivity == 4):
            self._time_question = 30
        elif(goal.proactivity == 5):
            self._time_question = 10
        else:
            rospy.logerr('Proactivity level not specified')
            return False

        # Check goal variables
        # -- Check type -- #
        if(self._type != 'joke' and self._type != 'saying'):
            rospy.logerr('Type NOT accepted')
            return False
        # -- Check categories -- #
        self._categories = self._categories.split(' ') # Divides goal by fields
        for category in self._categories:
            if(category != 'short'
                and category != 'medium'
                and category != 'long'
                and category != 'cliche'
                and category != 'jaimito'
                and category != 'robots'
                and category != 'dirty'):
                rospy.logerr('Category \'%s\' NOT accepted' % category)
                return False

        # Check max_time and number_plays
        if(self._max_time>0 and self._number_plays>0):
            self._limit_method = 'both'
        elif(self._max_time>0):
            self._limit_method = 'time'
        elif(self._number_plays>0):
            self._limit_method = 'plays'
        else:
            rospy.logerr('max_time and number_plays NOT accepted. Specify one of them')
            return False

        # Goal accepted
        rospy.loginfo('Goal accepted')
        return True

    def execute_cb(self, goal):
        """
        Callback of the node. Activated when a goal is received

        @param goal: jokes_skill goal.
        """

        # Init skill variables
        self.init_variables()

        # Init result and feedback
        # -- Result default values -- #
        self._result.skill_result = self._result.SUCCESS # Success
        # -- Feedback default values -- #
        self._feedback.app_status = 'start_ok'
        self._feedback.percentage_completed = 0
        self._feedback.engagement = True
        # -- Publish initial feedback -- #
        self._as.publish_feedback(self._feedback)

        ####################### Skill active #######################
        if self._status == self.RUNNING:
            print('\n')
            rospy.loginfo("RUNNING...")
            ##################### Process goal #####################
            rospy.loginfo('Goal: %s' % goal)
            if(not self.goal_handler(goal)): # Goal NOT correct
                rospy.logerr('Goal NOT correct')
                self._result.skill_result = self._result.ERROR # Error
                self._exec_out = True # Exits the loop
            else:
                ################## Init variables ##################
                self._goal_exec = True # Goal execution starts
                self._step = 'Search_info'
                n_questions = 1 # Variable to indicate the number of continue questions
                #==================================================#
                self._exec_out = False # Enters the loop
            #======================================================#
                
            ###################### Exec loop #######################
            self.start_timer()
            while(not self._exec_out and self._feedback.percentage_completed<100):
                try:
                    # Wait loop
                    self.pause_wait() # It pauses if it asked

                    # Exception check
                    self.exception_check() # If requested, it raises exception, else, it continues

                    # Get params
                    language = self.get_param('langauge')
                    user_name = self.get_param('user_name')
                    # Empty feedback status
                    self._feedback.app_status = ''

                    # Restart timer
                    self.restart_timer()

                    # Step 0
                    if(self._step=='Search_info'):
                        rospy.loginfo("Search_info")
                        # Make step 0 stuff
                        # ...
                        # Next step
                        self._step = 'Step1'

                    # Step 1
                    elif(self._step=='Step1'):
                        rospy.loginfo("Step1")
                        # Make step 1 stuff
                        # ...
                        # Next step
                        self._step = 'StepFinal'

                    # Step error
                    else:
                        rospy.logwarn("Step '%s' is not specified in the code" % self._step)
                        raise ActionlibException # Cancel the goal

                    self.update_timer() # Update self._time_run

                    # Final step
                    if(self._step=='StepFinal'):
                        rospy.loginfo("Final Step")

                        # Update limit variables
                        self._i_plays += 1
                        self.update_percentage()
                        # Next step
                        self._step = 'Step0'

                        # Exit question
                        if(self._feedback.percentage_completed<100):
                            # Asks when a time has passed
                            if(self._time_run > self._time_question * n_questions):
                                # Continue question
                                rospy.loginfo('Asking to continue')
                                etts_text, grammar, answer_id = self._xml_reader.GetQuestion('continue', language, user_name)
                                ca_info = makeCA_ASR_question(etts_text=etts_text, language=language, grammar = grammar, answer_id = answer_id, emitter=self._emitter)
                                self.ca_pub.publish(ca_info)
                                self.update_list_ca([ca_info.ca_name])
                                # Wait answer
                                self.wait_ca_answer(ca_info)
                                # Answer received
                                if(self._answer_received == 'si'): # Continue
                                    self._feedback.engagement = True
                                elif(self._answer_received == 'no'): # Stops skill
                                    # Send CA info
                                    etts_text = self._xml_reader.GetExpression('exit', language, user_name)
                                    ca_info = makeCA_etts_info(etts_text=etts_text, language=language, emitter=self._emitter)
                                    self.ca_pub.publish(ca_info)
                                    self.update_list_ca([ca_info.ca_name])
                                    # Wait finish CA
                                    self.wait_ca_finish(ca_info.ca_name, max_time=7)
                                    raise ActionlibException # Cancel the goal
                                else: # Continue but changes engagement
                                    self._feedback.engagement = False
                                # Update number of questions
                                n_questions += 1

                    # Exception check
                    self.exception_check() # If requested, it raises exception, else, it continues

                #################### Exceptions ####################
                ### Preempted or cancel:
                except ActionlibException:
                    rospy.logwarn('Preempted or cancelled')
                    # Feedback
                    if(self._status == self.STOPPED):
                        self._feedback.app_status = 'stop_ok'
                    else:
                        self._feedback.app_status = 'cancel_ok'
                    # Result
                    self._result.skill_result = self._result.FAIL # Preempted
                    # Exec loop variable
                    self._exec_out = True
                ### Error
                except ErrorException as e:
                    rospy.logerr(e)
                    # Feedback
                    self._feedback.app_status = 'error'
                    # Result
                    self._result.skill_result = self._result.ERROR # Error
                    # Exec loop variable
                    self._exec_out = True
                ### Pause
                except PauseException:
                    rospy.logwarn('Paused')
                    rospy.loginfo('Next step: %s' % self._step)
                    # Exec loop variable
                    self._exec_out = False
                #=================== Exceptions ===================#
                # Deactivate ca list
                self.deactivate_ca_list()
                
                # Publish feedback at the end of loop
                self._as.publish_feedback(self._feedback)

            #===================== Exec loop ======================#
            self._goal_exec = False # Goal execution finished
            print('\n')
        #==================== Skill active ========================#

        ##################### Skill NOT active #####################
        else:
            rospy.logwarn("STOPPED")
            rospy.logwarn("Cannot send a goal when the skill is stopped")
            self._result.skill_result = self._result.FAIL # Error
        #==========================================================#
        
        #### Result and feedback sending and goal status update ####
        if self._result.skill_result == self._result.SUCCESS:
            rospy.logdebug("setting goal to succeeded")
            self._feedback.app_status = 'completed_ok'
            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)
        else:
            rospy.logdebug("setting goal to preempted")
            self._feedback.app_status = 'completed_fail'
            self._as.publish_feedback(self._feedback)
            self._as.set_preempted(self._result)
        rospy.loginfo("#############################")
        rospy.loginfo("######## Result sent ########")
        rospy.loginfo("#############################")
        #==========================================================#
#=================================================================#


if __name__ == '__main__':
    try:
        # start the node
        rospy.init_node(skill_name, log_level=rospy.DEBUG)
        rospy.loginfo('[' + pkg_name + ': ' + skill_name + ']')

        # create and spin the node
        node = JokesSkill()
        rospy.sleep(1)
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass