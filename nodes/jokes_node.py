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
import actionlib

# Messages
import jokes_skill.msg
from std_msgs.msg import String, Empty
from interaction_msgs.msg import CA, CaResult

# Local libraries
from interaction_utils.ca_functions import *
from context_manager.params_lib import get_param
from hri_manager.key_value_pairs import to_dict
from jokes_skill.xml_reader import XMLReader
from jokes_skill.exceptions_lib import PauseException, ErrorException

# Libraries
import random

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

    ## @brief Init method
    def __init__(self):

        # Class variables
        self._as = None # SimpleActionServer variable
        self._list_ca = [] # List of sended CAs
        
        self.init_variables() # Init variables

        # Local paths
        rospack = rospkg.RosPack()
        self._pkg_path = rospack.get_path(pkg_name) + '/' # Package path
        self._data_path = self._pkg_path + 'data/' # Data path

        # Tablet paths
        self._icons_path = 'image/jokes/' # Icons path
        self._default_image = self._icons_path + 'chiquito.gif'

        # XMl reader
        self._xml_reader = XMLReader(self._data_path)

        # Jokes variables
        self._type = ''
        self._categories = []

        # init the skill
        Skill.__init__(self, skill_name, CONDITIONAL)

    ## @brief Initialization of variables
    def init_variables(self):

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
        self._response = {'result':'', 'ca_name':'', 'values':{}} # CA answer received
        self._active_ca = ''

######################### Skill Callbacks #########################
    ## @brief Receive the response from an asking for info CA
    #  @param recog_response: CA response
    def response_callback(self, recog_response): ## No modify ##

        # Checks if response corresponds to skill CAs
        if(recog_response.emitter == self._emitter): # Skill CA
            try:
                values = to_dict(recog_response.values)
                self._response = {'result':recog_response.result, 'ca_name':recog_response.ca_name.replace('ca_', ''), 'values':values}
                # Update error
                if self._response['result'] in ["communicationproblems", "no_response"]: # If there's some problem or user not response
                    response.update({'result':'Error'})
            except AttributeError as e:
                rospy.logerr(e)
                self._response = {'result':'', 'ca_name':'', 'values':{}}
            except KeyError as e:
                rospy.logerr(e)
                self._response = {'result':'', 'ca_name':'', 'values':{}}

        else: # Different CA
            rospy.logwarn('CA response not expected')

    ## @brief Pause callback
    #  Modify the variable self._pause if goal is being handled, when a pause is requested
    def pause_exec(self): ## No modify ##

        if(self._goal_exec and not self._pause): # Goal being handled and not in pause
            self._pause_requested = True
            rospy.logdebug('Pause requested')
        elif(self._pause):
            rospy.logdebug('Already in pause')
        else: # Goal NOT being handled
            rospy.logwarn('Goal not being handled')

    ## @brief Pause callback
    #  Modify the variable self._resume, when a resume is requested
    def resume_exec(self): ## No modify ##

        self._pause = False
        self._feedback.app_status = 'resume_ok'
        self._as.publish_feedback(self._feedback)

    ## @brief Start callback
    #  Callback called when skill is started
      #  @raise rospy.ROSException: If the service is inactive
    def create_msg_srv(self):
        
        rospy.loginfo("Start requested")

        # publishers and subscribers
        self.ca_pub = rospy.Publisher(
            "hri_manager/ca_activations", CA, queue_size=1) # CA publisher
        self.ca_deactivation_pub = rospy.Publisher(
            "hri_manager/ca_deactivations", String, queue_size=1) # CA deactivation publisher

        self.sub_response = rospy.Subscriber(
            "hri_manager/response", CaResult, self.response_callback) # CA response

        # servers and clients

        # If actionlib server has not been initialized
        if not self._as:
            self._as = actionlib.SimpleActionServer(skill_name, jokes_skill.msg.JokesAction, execute_cb=self.execute_cb, auto_start=False)
            # start the action server
            self._as.start()

    ## @brief Stop callback
    #  Callback called when skill is stopped
    def shutdown_msg_srv(self):

        # publishers and subscribers
        # FIXME: do not unregister publishers because a bug in ROS
        # self.__test_pub.unregister()

        rospy.loginfo("Stop requested")
        # servers and clients
        self.sub_response.unregister()

        # Cancel goal
        self._as.preempt_request = True

        self._feedback.app_status = 'stop_ok'
        self._as.publish_feedback(self._feedback)
#=================================================================#

######################### Skill Functions #########################
    ## @brief Stops active etts CA
    def stop_etts(self): ## No modify ##

        rospy.logdebug('Stop etts')
        ca_info = makeCA_etts_info(etts_text='\\pause=10', emitter=self._emitter)
        self.ca_pub.publish(ca_info)

     ## @brief Deactivate active CA
    def deactivate_ca(self): ## No modify ##

        if(self._active_ca != ''): ## CA active
            msg = deactivateCA(self._active_ca)
            self.ca_deactivation_pub.publish(msg)
            self._active_ca = ''
            #self.stop_etts()

    ## @brief Waits CA response
    #  Enters a loop until the CA has received a response,
    #  some time has passed or a exception has been called.
    #  @param ca_info: CA message
        #  @param max_time: Max time for the CA to finish, If 0, no max time
        #    If question is waited, max time will be calculated with
        #    answer_time and answer_attempts
        #  @param force_max_time: If true, not waits ca to stop, but uses only max_time
    def wait_ca_response(self, ca_info, max_time=0, force_max_time=False): ## No modify ##

        rospy.logdebug('Waiting CA %s response' % ca_info.ca_name)
        self._active_ca = ca_info.ca_name
        # If answer, max_time will be answer_time*answer_attempts+ktime
        ca_info_values = to_dict(ca_info.values)
        if('answer_time' and 'answer_attempts' in ca_info_values):
            max_time = (int(ca_info_values['answer_time'])+10) * int(ca_info_values['answer_attempts'])

        t0 = time.time()
        t = -1
        ca_name = 'forced' if(force_max_time) else ca_info.ca_name
        # Wait loop
        while(t<max_time and self._response['ca_name']!=ca_name):
            self.exception_check() # Check exceptions
            rospy.sleep(0.2)
            t = time.time()-t0 if(max_time!=0) else -1
        # Checks if response has been received
        if(self._response['ca_name']==ca_info.ca_name):
            rospy.logdebug('Response received')
            self._active_ca = ''
        else:
            rospy.logwarn('Response not received. Deactivating CA %s' % ca_info.ca_name)
            self.deactivate_ca()

    ## @brief Checks if an exception has been requested
    #  The exception request can be a preempt request or a pause request
      #  @raise ActionlibException: Preempt requested
      #  @raise PauseException: Pause requested
    def exception_check(self): ## No modify ##

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

    ## @brief Pause loop
    #  Finishes when goal is resumed or cancelled
    def pause_wait(self): ## No modify ##

        if(self._pause and not self._as.is_preempt_requested()):
            rospy.loginfo('Start waiting')
            # Send feedback
            self._feedback.app_status = 'pause_ok'
            self._as.publish_feedback(self._feedback)
            # Wait loop
            while(self._pause and not self._as.is_preempt_requested()):
                rospy.logdebug('waiting...')
                rospy.sleep(1)

    ## @brief Update the skill percentage
    def update_percentage(self): ## No modify ##

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
#=================================================================#

######################### Timer Functions #########################
    ## @brief Starts timer
    #  Update t0 and initialize time_run
    def start_timer(self): ## No modify ##

        self._t0 = time.time()
        self._time_run = 0

    ## @brief Restarts timer
    #  Update t0
    def restart_timer(self): ## No modify ##

        self._t0 = time.time()

    ## @brief Updates timer
    #  Update time_run
    def update_timer(self): ## No modify ##

        self._time_run += time.time() - self._t0
#=================================================================#

########################## Goal Handling ##########################
    ## @brief Searchs the expressions and questions list depending on the goal
      #  @return etts_list: List of etts texts
      #  @return gesture_list: List of gestures
    def search_info(self, language, user_name):

        # Search new joke
        if(self._type == 'joke'):
            item_list = self._xml_reader.GetListJokes(self._categories, language, user_name)
        # Search new saying
        elif(self._type == 'saying'):
            item_list = self._xml_reader.GetListSayings(self._categories, language, user_name)

        return item_list

    ## @brief Show info handler
    #  @param etts_text: Etts text to display
    #  @param gesture: Gesture to display
      #  @return ca_info
      #  @return max_wait
    def show_info_handler(self, item, language):

        max_wait = 100
        # Error found
        if(item==-1):
            if(self._type == 'joke'):
                etts_text = 'Creo que me he quedado sin chistes. Sorry'
            elif(self._type == 'saying'):
                etts_text = 'Creo que me he quedado sin refranes. Sorry'
            gesture = None
            max_wait = 7
        else:
            etts_text = item['etts']
            gesture = item['gesture']
            if('long' in self._categories):
                max_wait = 80
            elif('medium' in self._categories):
                max_wait = 40
            elif('short' in self._categories):
                max_wait = 20

        rospy.logdebug('etts_text: %s, language: %s, gesture: %s, image_url: %s, tablet_type: %s' % (etts_text, language, gesture, self._default_image, 'image'))
        ca_info = makeCA_info(etts_text=etts_text, language=language, gesture=gesture, tablet_url=self._default_image, tablet_type='image', emitter=self._emitter)
        self.ca_pub.publish(ca_info)

        return ca_info, max_wait

    ## @brief Goal handler
    #  Checks if the goal is appropriate.
    #  If True, it configures the variables for the goal execution
    #  @param goal: Goal received
      #  @return goal_accepted: True if goal is accepted, else False
    def goal_handler(self, goal):

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
        self._categories = self._categories.split('|') # Divides categories by fields
        category_list = ['short', 'medium', 'long', 'cliche', 'jaimito', 'robots', 'dirty']
        for category in self._categories:
            if(category not in category_list and category != 'random'):
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

    ## @brief Callback of the node
    #  Activated when a goal is received
    #  @param goal: skill goal
    def execute_cb(self, goal):

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

                    # Initialization
                    # -- Get params -- #
                    language = get_param('language')
                    user_name = get_param('user_name')
                    # -- Empty feedback status -- #
                    self._feedback.app_status = ''

                    # Restart timer
                    self.restart_timer()

                    # Step 0
                    if(self._step=='Search_info'):
                        rospy.loginfo("Search_info")

                        # Search_info
                        item_list = self.search_info(language, user_name)

                        # Next step
                        self._step = 'Select_info'

                    # Step 1
                    if(self._step=='Select_info'):
                        rospy.loginfo("Select_info")

                        if(len(item_list) == 0):
                            item = -1
                            rospy.logwarn('No more info')
                        else:
                            # Chose random
                            index = random.randint(0, len(item_list)-1)
                            item = item_list.pop(index)

                        # Next step
                        self._step = 'Show_info'

                    # Step 2
                    elif(self._step=='Show_info'):
                        rospy.loginfo("Show_info")

                        # Show info
                        ca_name, max_wait = self.show_info_handler(item, language)
                        # Wait finish CA
                        self.wait_ca_response(ca_name, max_time=max_wait)
                        if(item==-1): # Error
                            raise ActionlibException # Cancel the goal
                        else:
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
                        self._step = 'Select_info'

                        # Exit question
                        if(self._feedback.percentage_completed<100):
                            # Asks when a time has passed
                            if(self._time_run > self._time_question * n_questions):
                                # Continue question
                                rospy.loginfo('Asking to continue')
                                etts_text, grammar, answer_id = self._xml_reader.GetQuestion('continue', language, user_name)
                                ca_info = makeCA_ASR_question(etts_text=etts_text, language=language, grammar = grammar, answer_id = answer_id, emitter=self._emitter)
                                self.ca_pub.publish(ca_info)
                                self.wait_ca_response(ca_info) # Wait response
                                # Answer
                                if('answer_value' in self._response['values']): # Answer received
                                    if(self._response['values']['answer_value'] == 'si'): # Continue
                                        self._feedback.engagement = True
                                    elif(self._response['values']['answer_value'] == 'no'): # Stops skill
                                        # Send CA info
                                        etts_text = self._xml_reader.GetExpression('exit', language, user_name)
                                        ca_info = makeCA_etts_info(etts_text=etts_text, language=language, emitter=self._emitter)
                                        self.ca_pub.publish(ca_info)
                                        self.wait_ca_response(ca_info, max_time=6) # Wait response
                                        raise ActionlibException # Cancel the goal
                                    else: # Continue but changes engagement
                                        self._feedback.engagement = False
                                else: # Answer not received
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
                self.deactivate_ca()
                
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

        # Default image
        ca_info = makeCA_defaultImage(emitter=self._emitter)
        self.ca_pub.publish(ca_info)

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