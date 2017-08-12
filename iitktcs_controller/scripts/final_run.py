#!/usr/bin/env python

import roslib; roslib.load_manifest('iitktcs_controller')
import rospy
import smach
from smach_ros import IntrospectionServer
from smach_ros import ServiceState
from operator import itemgetter

# Import services.
from json_maker.srv import *
from iitktcs_msgs_srvs.srv import *

# Import messages.
from iitktcs_msgs_srvs.msg import *

# Import custom python files.
from open_gripper import publisher
from present_time import present_time_in_millisec

# Import data.
import time

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['start'],
                             input_keys     =['start_time',
                                              'objects',
                                              'objects_attempt',
                                              'debug'],
                             output_keys    =['start_time',
                                              'objects',
                                              'objects_attempt'])
    def execute(self, userdata):
        model_names = rospy.search_param('/ARC17_OBJECT_NAMES')
        userdata.objects = rospy.get_param(model_names)
        for i in range(0, len(userdata.objects)):
            print userdata.objects[i]
        for i in range(0, len(userdata.objects)):
            userdata.objects_attempt.append(0)
        userdata.start_time = rospy.get_rostime().secs
        if (userdata.debug == True):
            test = present_time_in_millisec().getTime()
            print "Test is : {}".format(test)
        print "System started at {} secs.".format(userdata.start_time)
        return 'start'

class Success(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['success',
                                              'drop_pose_view'],
                             input_keys     =['bin_available_objects',
                                              'bin_target_objects',
                                              'bin',
                                              'no_of_bins',
                                              'target_object',
                                              'picked_objects',
                                              'temp_objects',
                                              'non_target_object',
                                              'temp_object_placement',
                                              'packing_box_objects',
                                              'objects_attempt'],
                             output_keys    =['bin',
                                              'non_target_object',
                                              'remove_non_target_object',
                                              'temp_objects',
                                              'objects_attempt'])
    def execute(self, userdata):
        if(userdata.non_target_object==False):
            index = -1
            for i in range(0,len(userdata.packing_box_objects)):
                for j in range(0,len(userdata.packing_box_objects[i])):
                    if (userdata.target_object == userdata.packing_box_objects[i][j]):
                        index = i
                        break
            userdata.picked_objects[index].append(userdata.target_object)
            userdata.bin_available_objects[userdata.bin].remove(userdata.target_object)
            userdata.bin_target_objects[userdata.bin].remove(userdata.target_object)
            userdata.remove_non_target_object = 0
            return 'drop_pose_view'
        elif(userdata.non_target_object==True):
            userdata.bin_available_objects[userdata.bin].remove(userdata.target_object)
            userdata.bin_available_objects[userdata.temp_object_placement].append(userdata.target_object)
            userdata.objects_attempt[userdata.target_object - 1] = 0
            userdata.non_target_object=False
            return 'success'

class target_object_check(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['present',
                                              'check_next_bin',
                                              'absent'],
                             input_keys     =['bin_target_objects',
                                              'bin',
                                              'no_of_bins',
                                              'attempt_at_last',
                                              'objects_attempt',
                                              'debug'],
                             output_keys    =['bin_target_objects',
                                              'bin',
                                              'attempt_at_last',
                                              'objects_attempt'])
    def execute(selfself, userdata):
        if (userdata.debug == True):
            print "Target object before attempt check is : {}".format(userdata.bin_target_objects)
        temp_bin_target = []
        for i in range(0, len(userdata.bin_target_objects[userdata.bin])):
            if (userdata.objects_attempt[userdata.bin_target_objects[userdata.bin][i] - 1] > 1):
                if (userdata.debug == True):
                    print "Inside attempt at last. {}".format(userdata.bin_target_objects[userdata.bin][i])
                userdata.attempt_at_last[userdata.bin].append(userdata.bin_target_objects[userdata.bin][i])
            else:
                if (userdata.debug == True):
                    print "Else loop . {}".format(userdata.bin_target_objects[userdata.bin][i])
                temp_bin_target.append(userdata.bin_target_objects[userdata.bin][i])
        if (userdata.debug == True):
            print "Objects to attempt at last : {}".format(userdata.attempt_at_last)
        # for i in range(0, len(userdata.attempt_at_last[userdata.bin])):
        #     if userdata.attempt_at_last[userdata.bin][i] in userdata.bin_target_objects[userdata.bin]:
        #         userdata.bin_target_objects[userdata.bin].remove(userdata.attempt_at_last[userdata.bin][i])
        userdata.bin_target_objects[userdata.bin] = temp_bin_target
        if (userdata.debug == True):
            print "Target object after attempt check is : {}".format(userdata.bin_target_objects)

        if(len(userdata.bin_target_objects[userdata.bin])==0):
            if(userdata.bin<(userdata.no_of_bins-1)):
                if (userdata.debug == True):
                    print "Bin is ======== {}".format(userdata.bin)
                userdata.bin+=1
                return 'check_next_bin'
            else:
                if (userdata.debug == True):
                    print "Bin is ******** {}".format(userdata.bin)
                return 'absent'
        else:
            if (userdata.debug == True):
                print "Bin is @@@@@@@@@ {}".format(userdata.bin)
            return 'present'

class failed_to_move_target_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['octomap_publisher',
                                              'target_object_list_check',
                                              'reattempt'],
                             input_keys     =['bin_available_objects',
                                              'bin_target_objects',
                                              'object_info',
                                              'reattempt_objects',
                                              'bin',
                                              'no_of_bins',
                                              'non_target_object',
                                              'partial_occlusion',
                                              'object_model_fitting',
                                              'debug'],
                             output_keys    =['bin',
                                              'target_object',
                                              'object_model_fitting'])
    def execute(self, userdata):
        if(userdata.non_target_object==False):
            userdata.reattempt_objects[userdata.bin].append(userdata.object_info[0].obj_info_vect[0].id.data)
            # userdata.bin_available_objects[userdata.bin].remove(userdata.object_info[0].id.data)
            userdata.bin_target_objects[userdata.bin].remove(userdata.object_info[0].obj_info_vect[0].id.data)
            userdata.object_info.pop(0)
            if (userdata.debug == True):
                print "Failed to move target object : {}".format(userdata.reattempt_objects)
            if len(userdata.object_info) != 0:
                userdata.object_model_fitting = []
                userdata.object_model_fitting.append(userdata.object_info[0].obj_info_vect[0])
                userdata.target_object = userdata.object_model_fitting[0].id.data
                return 'octomap_publisher'
            else:
                userdata.bin+=1
                if(userdata.bin >= userdata.no_of_bins):
                    return 'reattempt'
                else:
                    return 'target_object_list_check'
        elif(userdata.non_target_object==True and userdata.partial_occlusion[userdata.bin] == True):
            userdata.reattempt_objects[userdata.bin].append(userdata.object_info[0].obj_info_vect[0].id.data)
            userdata.bin_target_objects[userdata.bin].remove(userdata.object_info[0].obj_info_vect[0].id.data)
            userdata.object_info.pop(0)
            if (userdata.debug == True):
                print "Failed to move target object : {}".format(userdata.reattempt_objects)
            if (len(userdata.object_info) != 0):
                userdata.non_target_object = True
                userdata.object_model_fitting = []
                userdata.object_model_fitting.append(userdata.object_info[0].obj_info_vect[len(userdata.object_info[0])-1])
                userdata.target_object = userdata.object_model_fitting[0].id.data
                if (userdata.bin == 0):
                    userdata.temp_object_placement = 1
                else:
                    userdata.temp_object_placement = 0
                return 'octomap_publisher'
            else:
                userdata.bin += 1
                if (userdata.bin >= userdata.no_of_bins):
                    return 'reattempt'
                else:
                    return 'target_object_list_check'
        elif(userdata.non_target_object==True and userdata.partial_occlusion[userdata.bin] == False):
            userdata.object_info.pop(0)
            if len(userdata.object_info) != 0:
                userdata.object_model_fitting = []
                userdata.object_model_fitting.append(userdata.object_info[0].obj_info_vect[0])
                userdata.target_object = userdata.object_model_fitting[0].id.data
                return 'octomap_publisher'
            else:
                userdata.bin+=1
                if(userdata.bin >= userdata.no_of_bins):
                    return 'reattempt'
                else:
                    return 'target_object_list_check'

class reattempt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['target_object_list_check',
                                              'remove_non_target_object',
                                              'rest'],
                             input_keys     =['bin_available_objects',
                                              'bin_target_objects',
                                              'reattempt_objects',
                                              'bin',
                                              'no_of_bins',
                                              'remove_non_target_object',
                                              'partial_occlusion',
                                              'partial_occlusion_detected',
                                              'attempt_at_last',
                                              'objects_attempt',
                                              'debug'],
                             output_keys    =['bin_available_objects',
                                              'bin_target_objects',
                                              'reattempt_objects',
                                              'bin',
                                              'remove_non_target_object',
                                              'objects_attempt',
                                              'attempt_at_last'])
    def execute(self, userdata):
        size = 0
        attempt_last = 0
        bin_partial_occlusion = False
        if (userdata.debug == True):
            print "Reattempt object list is {}".format(userdata.reattempt_objects)
            print "Bin available object list {}".format(userdata.bin_available_objects)
            print "Bin target object list {}".format(userdata.bin_target_objects)
        for i in range(0,len(userdata.reattempt_objects)):
            if(len(userdata.reattempt_objects[i])>=1):
                size = 1
        for i in range(0,len(userdata.bin_target_objects)):
            if(len(userdata.bin_target_objects[i])>=1):
                size = 1
        for i in range(0,len(userdata.attempt_at_last)):
            if(len(userdata.attempt_at_last[i])>=1):
                attempt_last = 1
        if (userdata.debug == True):
            print "Partial Occlusion for bins are : {}".format(userdata.partial_occlusion_detected)
        for i in range(0, len(userdata.partial_occlusion_detected)):
            if(userdata.partial_occlusion_detected[i]==True):
                bin_partial_occlusion = True
        if (userdata.debug == True):
            print "Flag for partial occlusion is : {}".format(bin_partial_occlusion)
        if(size == 0 and attempt_last == 0):
            return 'rest'
        elif(size == 0 and attempt_last == 1):
            userdata.remove_non_target_object = 1
            userdata.bin = 0
            for i in range(0,len(userdata.attempt_at_last)):
                for j in range(0,len(userdata.attempt_at_last[i])):
                    userdata.objects_attempt[userdata.attempt_at_last[i][j]-1] = 0
            for i in range(0,len(userdata.bin_target_objects)):
                userdata.bin_target_objects[i] += userdata.attempt_at_last[i]
            userdata.attempt_at_last = [[],[]]
            if (userdata.debug == True):
                print userdata.bin_available_objects
                print userdata.bin_target_objects
                print userdata.attempt_at_last
            return 'target_object_list_check'
        elif(userdata.remove_non_target_object == 1 and bin_partial_occlusion == False):
            userdata.remove_non_target_object = 1
            userdata.bin = 0
            for i in range(0,len(userdata.bin_target_objects)):
                # userdata.bin_available_objects[i] += userdata.reattempt_objects[i]
                userdata.bin_target_objects[i] += userdata.reattempt_objects[i]
            userdata.reattempt_objects = [[],[]]
            if (userdata.debug == True):
                print userdata.bin_available_objects
                print userdata.bin_target_objects
                print userdata.reattempt_objects
            return 'remove_non_target_object'
        else:
            userdata.remove_non_target_object = 1
            userdata.bin = 0
            for i in range(0,len(userdata.bin_target_objects)):
                # userdata.bin_available_objects[i] += userdata.reattempt_objects[i]
                userdata.bin_target_objects[i] += userdata.reattempt_objects[i]
            userdata.reattempt_objects = [[],[]]
            if (userdata.debug == True):
                print userdata.bin_available_objects
                print userdata.bin_target_objects
                print userdata.reattempt_objects
            return 'target_object_list_check'

class Failure(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['failure',
                                              'reattempt'],
                             input_keys     =['bin_available_objects',
                                              'bin_target_objects',
                                              'bin',
                                              'no_of_bins',
                                              'target_object',
                                              'reattempt_objects',
                                              'non_target_object',
                                              'debug'],
                             output_keys    =['bin',
                                              'non_target_object'])
    def execute(self, userdata):
        if(userdata.non_target_object==False):
            userdata.reattempt_objects[userdata.bin].append(userdata.target_object)
            # userdata.bin_available_objects[userdata.bin].remove(userdata.target_object)
            userdata.bin_target_objects[userdata.bin].remove(userdata.target_object)
            if (len(userdata.bin_target_objects[userdata.bin]) == 0 and userdata.bin < userdata.no_of_bins):
                userdata.bin = userdata.bin+1
                if (userdata.bin >= userdata.no_of_bins):
                    return 'reattempt'
        userdata.non_target_object = False
        return 'failure'

class remove_non_target_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['bin_view'],
                             input_keys     =['no_of_bins',
                                              'bin_target_objects',
                                              'bin',
                                              'temp_object_placement',
                                              'non_target_state_initiated',
                                              'non_target_object',
                                              'debug'],
                             output_keys    =['bin',
                                              'temp_object_placement',
                                              'non_target_state_initiated',
                                              'non_target_object'])
    def execute(self, userdata):
        userdata.non_target_object          = True
        userdata.non_target_state_initiated = True
        size = []
        for i in range (0,(userdata.no_of_bins)):
            size.append(len(userdata.bin_target_objects[i]))
        if (userdata.debug == True):
            print size
        index_max                           = size.index(max(size))
        index_min                           = size.index(min(size))
        if (index_max == index_min):
            userdata.temp_object_placement  = (userdata.no_of_bins-1)
            userdata.bin                    = 0
        else:
            userdata.temp_object_placement  = index_min
            userdata.bin                    = index_max
        if (userdata.debug == True):
            print "Bin to remove from is : {}".format(userdata.bin)
            print "Place to keep the object at : {}".format(userdata.temp_object_placement)
        return 'bin_view'

class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['complete'])
    def execute(self,userdata):
        return 'complete'

def main():

    # Bin angles of bin view.
    bin_angles = [[-0.0109823,  -1.56805,   1.37091,    -1.11577,   -1.58023,   0.0018583],
                  [-0.570878,   -1.5398,    1.35223,    -1.16178,   -1.71734,   -0.532907]]
    # Non-Target Object Placement.
    temp_object_placement_joints=[[[0.0233202,  -1.15814,   1.70757,    -2.07361,   -1.57921,   0.800493],[0.0625764,   -1.34026,   1.96991,    -2.1541,    -1.575,     0.839319]],
                                   [[-0.552563, -1.35272,   1.98701,    -2.16316,   -1.60329,   0.224726],[-0.462898,   -1.10364,   1.63139,    -2.05489,   -1.60244,   0.314815]]]
    objects_per_drop_location   = [[0,  0],[0,  0]]
    # Drop joint angles
    drop_joints = [[[1.99343, -1.3337, 1.38803, -1.34644, -1.43215, 0.337836],
                    [1.94427, -0.922341, 1.6076, -2.18562, -1.58184, 0.279748],
                    [1.9556, -0.952392, 1.66262, -2.21049, -1.5805, 0.290858],
                    [1.96379, -0.972878, 1.69945, -2.22701, -1.57964, 0.299044]],
                   [[1.45534, -1.58904, 1.66518, -1.38629, -1.59178, -0.177695],
                    [1.48963, -1.0651, 1.87965, -2.34867, -1.59029, -0.153011],
                    [1.48561, -1.11222, 1.9614, -2.38317, -1.58956, -0.157099],
                    [1.48387, -1.1312, 1.99376, -2.39655, -1.58937, -0.158825]],
                   [[0.880835, -1.46296, 1.54236, -1.43419, -1.73241, -0.697552],
                    [0.983552, -1.0081, 1.78884, -2.37154, -1.60666, -0.689648],
                    [0.955249, -1.05428, 1.87122, -2.40844, -1.60536, -0.717962],
                    [0.925003, -1.09855, 1.94898, -2.44257, -1.60403, -0.748302]],
                   [[-2.17591, -1.49156, 1.54613, -1.3822, -1.75844, -0.567945],
                    [-2.13699, -0.974254, 1.71744, -2.28123, -1.58937, -0.566254],
                    [-2.16083, -1.01754, 1.79526, -2.31598, -1.58939, -0.590369],
                    [-2.1971, -1.07567, 1.89796, -2.36063, -1.58967, -0.626715]],
                   [[-1.37183, -1.46513, 1.50598, -1.33244, -1.54755, 0.212478],
                    [-1.43616, -0.9789, 1.72331, -2.28588, -1.55003, 0.143358],
                    [-1.42754, -1.03873, 1.83023, -2.33339, -1.54886, 0.151807],
                    [-1.41657, -1.10424, 1.94466, -2.38264, -1.54741, 0.162582]]]
    # Drop joint view angles
    drop_joints_view = [[1.99343, -1.3337, 1.38803, -1.34644, -1.43215, 0.337836],
                       [1.45534, -1.58904, 1.66518, -1.38629, -1.59178, -0.177695],
                       [0.880835, -1.46296, 1.54236, -1.43419, -1.73241, -0.697552],
                       [-2.17591, -1.49156, 1.54613, -1.3822, -1.75844, -0.567945],
                       [-1.37183, -1.46513, 1.50598, -1.33244, -1.54755, 0.212478]]

    objects_per_box_drop_location = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
    # Rest pose joint angles
    rest_joints = [-0.0712016,  -2.71543,   2.6089, -2.8294,    -1.49305,   -0.0444363]

    # Initialize ROS node and smach variables.
    rospy.init_node('arc_smach_state_machine')

    sm = smach.StateMachine(['succeeded', 'aborted', 'preempted'])

    # Userdata
    sm.userdata.bin_available_objects       = []
    sm.userdata.bin_target_objects          = [[],[]]
    sm.userdata.packing_box_objects         = []
    sm.userdata.picked_objects              = [[],[],[],[],[]]
    sm.userdata.reattempt_objects           = [[],[]]
    sm.userdata.remove_non_target_object    = 0
    sm.userdata.missing_objects             = []
    sm.userdata.bin                         = 0
    sm.userdata.no_of_bins                  = 2
    sm.userdata.target_object               = -1
    sm.userdata.reattempt_single_goal       = 0
    sm.userdata.reattempt_retrieval_max     = 1
    sm.userdata.reattempt_retrieval         = 0
    sm.userdata.remove_object               = False
    sm.userdata.object_position             = UR5PoseGoalRequest()
    sm.userdata.grasp_location              = GripperPoseGoalRequest()
    sm.userdata.gripper_objects             = [20,1,17,25,14,11,22,30,31,32,38,40,16,9,23,24,15,13]
    sm.userdata.attempt_at_last             = [[],[]]
    sm.userdata.objects_attempt             = []
    sm.userdata.valley_drop_request         = UR5PoseGoalRequest()
    sm.userdata.drop_pose_sequence          = 0
    sm.userdata.blacklisted_object          = []
    # Debugging Mode
    sm.userdata.debug                       = True
    # For Non-target object.
    sm.userdata.non_target_object           = False
    sm.userdata.non_target_state_initiated  = False
    sm.userdata.partial_occlusion           = [False,False]
    sm.userdata.partial_occlusion_detected  = [False,False]
    sm.userdata.temp_object_placement       = sm.userdata.no_of_bins
    sm.userdata.object_model_fitting        = []
    sm.userdata.temp_objects                = []
    sm.userdata.start_time                  = 0

    # Create and start the introspection server
    sis = IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    with sm:

        # Start State.
        smach.StateMachine.add('START', Start(),
                               transitions={'start' :   'READ_OBJECT_LIST_FROM_JSON'})

        # JSON Reader State
        def read_object_list_from_json_request_cb(userdata, request):
            bin_index = get_target_bin_objectsRequest()
            userdata.state_start_time = present_time_in_millisec().getTime()
            return bin_index
        def read_object_list_from_json_response_cb(userdata, response):
            userdata.state_stop_time = present_time_in_millisec().getTime()
            print "Time taken for JSON Reader State is {} microsec.".format(userdata.state_stop_time - userdata.state_start_time)
            print " "
            for i in range(0,len(response.ids_available)):
                userdata.bin_available_objects.append(list(response.ids_available[i].ids.data))
            for i in range(0,len(response.ids_target)):
                userdata.packing_box_objects.append(list(response.ids_target[i].ids.data))
            userdata.task = response.task.data
            target = []
            for i in range(0,len(userdata.packing_box_objects)):
                for j in range(0,len(userdata.packing_box_objects[i])):
                    target.append(userdata.packing_box_objects[i][j])
            if (userdata.debug == True):
                print "Target is : {}".format(target)
            for i in range(0,len(userdata.bin_available_objects)):
                for j in range(0,len(target)):
                    if target[j] in userdata.bin_available_objects[i]:
                        userdata.bin_target_objects[i].append(target[j])
            temp_bin_available = [[],[]]
            for i in range(0,len(userdata.bin_available_objects)):
                for j in range(0,len(userdata.bin_available_objects[i])):
                    if not userdata.bin_available_objects[i][j] in userdata.blacklisted_object:
                        temp_bin_available[i].append(userdata.bin_available_objects[i][j])
            userdata.bin_available_objects = temp_bin_available
            temp_bin_target = [[],[]]
            for i in range (0,len(userdata.bin_target_objects)):
                for j in range (0,len(userdata.bin_target_objects[i])):
                    if not userdata.bin_target_objects[i][j] in userdata.blacklisted_object:
                        temp_bin_target[i].append(userdata.bin_target_objects[i][j])
            userdata.bin_target_objects = temp_bin_target
            if (userdata.debug == True):
                print userdata.bin_target_objects
                print "Task is : {}".format(userdata.task)
                print ("Available")
                for i in range(0,len(userdata.bin_available_objects)):
                    print userdata.bin_available_objects[i]
                print ("Target")
                for i in range(0,len(userdata.bin_target_objects)):
                    print userdata.bin_target_objects[i]
            return 'succeeded'
        smach.StateMachine.add('READ_OBJECT_LIST_FROM_JSON',
                               ServiceState('/iitktcs/pick_object_service', get_target_bin_objects,
                                            request_cb  =read_object_list_from_json_request_cb,
                                            input_keys  =['bin_available_objects',
                                                          'bin_target_objects',
                                                          'packing_box_objects',
                                                          'task',
                                                          'state_start_time',
                                                          'state_stop_time',
                                                          'debug',
                                                          'blacklisted_object'],
                                            response_cb =read_object_list_from_json_response_cb,
                                            output_keys =['bin_available_objects',
                                                          'bin_target_objects',
                                                          'packing_box_objects',
                                                          'task',
                                                          'state_start_time',
                                                          'state_stop_time']),
                               transitions={'succeeded'             :   'TARGET_OBJECT_LIST_CHECK',
                                            'aborted'               :   'READ_OBJECT_LIST_FROM_JSON'},
                               remapping={'bin_available_objects'   :   'bin_available_objects',
                                          'bin_target_objects'      :   'bin_target_objects',
                                          'task'                    :   'task'})

        # Target Object Check State
        smach.StateMachine.add('TARGET_OBJECT_LIST_CHECK', target_object_check(),
                               transitions={'present'               :   'BIN_VIEW_POSE',
                                            'check_next_bin'        :   'TARGET_OBJECT_LIST_CHECK',
                                            'absent'                :   'REATTEMPT'},
                               remapping={'bin_target_objects'      :   'bin_target_objects',
                                          'bin'                     :   'bin'})

        # Initial Bin View Pose State.
        def single_goal_request_cb(userdata, request):
            pose = UR5GoalRequest()
            if (userdata.debug == True):
                print "Bin is {}".format(userdata.bin)
            for i in range(0, 6):
                pose.goal.data.append(bin_angles[userdata.bin][i])
            if(userdata.remove_object == True):
                pose.flag_remove_object.data = True
            else:
                pose.flag_remove_object.data = False
            userdata.state_start_time = present_time_in_millisec().getTime()
            return pose
        def single_goal_response_cb(userdata, response):
            userdata.state_stop_time = present_time_in_millisec().getTime()
            print "Time taken for Bin View State is {} microsec.".format(userdata.state_stop_time-userdata.state_start_time)
            print " "
            time.sleep(1)
            userdata.remove_object = False
            if(response.success.data == True):
                return 'succeeded'
            else:
                return 'aborted'
        smach.StateMachine.add('BIN_VIEW_POSE',
                               ServiceState('/iitktcs/motion_planner/single_goal', UR5Goal,
                                            request_cb  =single_goal_request_cb,
                                            input_keys  =['bin',
                                                          'bin_target_objects',
                                                          'remove_object',
                                                          'state_start_time',
                                                          'state_stop_time',
                                                          'debug'],
                                            response_cb =single_goal_response_cb,
                                            output_keys =['remove_object',
                                                          'state_start_time',
                                                          'state_stop_time']),
                               transitions = {'succeeded'  :   'OBJECT_DETECT_SERVICE',
                                              'aborted'    :   'BIN_VIEW_POSE'})

        # Object Detection State
        def object_detect_service_request_cb(userdata, request):
            # For target objects.
            if(userdata.non_target_object==False):
                object_det                          =computer_vision_stowing_pickingRequest()
                object_det.task.data                =userdata.task
                object_det.bin_id.data              =userdata.bin
                object_det.ids_available.data       =userdata.bin_available_objects[userdata.bin]
                object_det.ids_target.data          =userdata.bin_target_objects[userdata.bin]
                # for i in range (0,len(userdata.bin_target_objects[userdata.bin])):
                #     if (userdata.objects_attempt[userdata.bin_target_objects[userdata.bin][i]-1] > 1):
                #         print "Inside attempt at last. {}".format(userdata.bin_target_objects[userdata.bin][i])
                #         userdata.attempt_at_last[userdata.bin].append(userdata.bin_target_objects[userdata.bin][i])
                #     else:
                #         print "Else loop . {}".format(userdata.bin_target_objects[userdata.bin][i])
                #         object_det.ids_target.data.append(userdata.bin_target_objects[userdata.bin][i])
                # print "Objects to attempt at last : {}".format(userdata.attempt_at_last)
                # for i in range (0,len(userdata.attempt_at_last[userdata.bin])):
                #     if userdata.attempt_at_last[userdata.bin][i] in userdata.bin_target_objects[userdata.bin]:
                #         userdata.bin_target_objects[userdata.bin].remove(userdata.attempt_at_last[userdata.bin][i])
                if (userdata.debug == True):
                    print "Objects to attempt at last : {}".format(userdata.attempt_at_last)
                    print "Target objects in bin {} are {}".format(userdata.bin, userdata.bin_target_objects[userdata.bin])
                userdata.state_start_time = present_time_in_millisec().getTime()
                return object_det
            # For non-target objects.
            elif(userdata.non_target_object==True):
                object_det                          =computer_vision_stowing_pickingRequest()
                object_det.task.data                =userdata.task
                object_det.bin_id.data              =userdata.bin
                object_det.ids_available.data       =userdata.bin_available_objects[userdata.bin]
                temp_target                         = []
                for i in range(0,len(userdata.bin_available_objects[userdata.bin])):
                    if not userdata.bin_available_objects[userdata.bin][i] in userdata.bin_target_objects[userdata.bin]:
                        temp_target.append(userdata.bin_available_objects[userdata.bin][i])
                # New Changes.
                for i in range(0,len(userdata.attempt_at_last[userdata.bin])):
                    if userdata.attempt_at_last[userdata.bin][i] in temp_target:
                        temp_target.remove(userdata.attempt_at_last[userdata.bin][i])
                for i in range(0,len(userdata.reattempt_objects[userdata.bin])):
                    if userdata.reattempt_objects[userdata.bin][i] in temp_target:
                        temp_target.remove(userdata.reattempt_objects[userdata.bin][i])
                for i in range(0,len(temp_target)):
                    if not (userdata.objects_attempt[temp_target[i] - 1] > 1):
                        object_det.ids_target.data.append(temp_target[i])
                if (userdata.debug == True):
                    print userdata.bin_available_objects[userdata.bin]
                    print userdata.bin_target_objects[userdata.bin]
                    print "Target objects in bin {} are {}".format(userdata.bin, object_det.ids_target.data)
                userdata.state_start_time = present_time_in_millisec().getTime()
                return object_det
        def object_detect_service_response_cb(userdata, response):
            userdata.state_stop_time = present_time_in_millisec().getTime()
            print "Time taken for Object Detection State is {} microsec.".format(userdata.state_stop_time-userdata.state_start_time)
            print " "
            userdata.object_info                                    = response.object_info
            userdata.bin_cloud                                      = response.bin_cloud
            userdata.partial_occlusion[userdata.bin]                = False
            # For target objects.
            if(userdata.non_target_object==False):
                # Sort on the basis of length of sub-list.
                userdata.object_info.sort(lambda x,y: cmp(len(x.obj_info_vect), len(y.obj_info_vect)))
                temp_available_object_ids   = []
                temp_unavailable_object_ids = []
                if (userdata.debug == True):
                    print "For bin {} length of target object is {}".format(userdata.bin,len(userdata.object_info))
                for i in range(0,len(userdata.object_info)):
                    if (userdata.debug == True):
                        print "Object id {} and object is {}.".format(userdata.object_info[i].obj_info_vect[0].id.data,userdata.objects[userdata.object_info[i].obj_info_vect[0].id.data-1])
                    temp_available_object_ids.append(userdata.object_info[i].obj_info_vect[0].id.data)
                for i in range(0,len(userdata.bin_target_objects[userdata.bin])):
                    if not userdata.bin_target_objects[userdata.bin][i] in temp_available_object_ids:
                        temp_unavailable_object_ids.append(userdata.bin_target_objects[userdata.bin][i])
                if (userdata.debug == True):
                    print "temp_unavailable_object_ids are : {}".format(temp_unavailable_object_ids)
                for i in range(0,len(temp_unavailable_object_ids)):
                    userdata.reattempt_objects[userdata.bin].append(temp_unavailable_object_ids[i])
                    # userdata.bin_available_objects[userdata.bin].remove(temp_unavailable_object_ids[i])
                    userdata.bin_target_objects[userdata.bin].remove(temp_unavailable_object_ids[i])
                if len(userdata.object_info) != 0:
                    for i in range (0,len(userdata.object_info)):
                        if(len(userdata.object_info[i].obj_info_vect)>1):
                            userdata.partial_occlusion_detected[userdata.bin] = True
                        if (userdata.debug == True):
                            print "Partial occlusion for bins after update is : {}".format(userdata.partial_occlusion_detected[userdata.bin])
                            print "************"
                            for j in range (0,len(userdata.object_info[i].obj_info_vect)):
                                if (j == 0):
                                    print "Object {} is occluded by :".format(userdata.objects[userdata.object_info[i].obj_info_vect[j].id.data-1])
                                else:
                                    print userdata.objects[userdata.object_info[i].obj_info_vect[j].id.data-1]
                            print "@@@@@@@@@@@@"
                    if(len(userdata.object_info[0].obj_info_vect)>1):
                        userdata.partial_occlusion[userdata.bin] = True
                    if(userdata.partial_occlusion[userdata.bin] == False):
                        for i in range(0,len(userdata.object_info)):
                            if (len(userdata.object_info[i].obj_info_vect)>1):
                                userdata.reattempt_objects[userdata.bin].append(userdata.object_info[i].obj_info_vect[0].id.data)
                                userdata.bin_target_objects[userdata.bin].remove(userdata.object_info[i].obj_info_vect[0].id.data)
                        index_to_remove = len(userdata.object_info)
                        for i in range(0,len(userdata.object_info)):
                            if (userdata.debug == True):
                                print "Initial size for target {} is {}.".format(userdata.object_info[i].obj_info_vect[0].id.data,len(userdata.object_info[i].obj_info_vect))
                            if (len(userdata.object_info[i].obj_info_vect)>1):
                                if (userdata.debug == True):
                                    print "Test {}".format(i)
                                index_to_remove = i
                                break
                        if (userdata.debug == True):
                            for i in range(0, len(userdata.object_info)):
                                # print userdata.objects(userdata.object_info[i].obj_info_vect[0].id.data-1)
                                print userdata.object_info[i].obj_info_vect[0].id.data
                        userdata.object_info = userdata.object_info[0:index_to_remove]
                        if (userdata.debug == True):
                            for i in range(0, len(userdata.object_info)):
                                print "To pick."
                                # print userdata.objects(userdata.object_info[i].obj_info_vect[0].id.data-1)
                                print "Size for target {} is {}.".format(userdata.object_info[i].obj_info_vect[0].id.data,len(userdata.object_info[i].obj_info_vect))
                            print "Model ID is {}".format(userdata.object_info[0].obj_info_vect[0].id.data)
                        userdata.object_model_fitting = []
                        userdata.object_model_fitting.append(userdata.object_info[0].obj_info_vect[0])
                        userdata.target_object = userdata.object_model_fitting[0].id.data
                        userdata.objects_attempt[userdata.target_object - 1] += 1
                        if (userdata.debug == True):
                            print ("Target object is : {}".format(userdata.objects[userdata.target_object - 1]))
                        return 'succeeded'
                    else:
                        #userdata.partial_occlusion[userdata.bin] = False
                        userdata.non_target_object=True
                        userdata.object_model_fitting = []
                        userdata.object_model_fitting.append(userdata.object_info[0].obj_info_vect[len(userdata.object_info[0].obj_info_vect)-1])
                        userdata.objects_attempt[userdata.object_info[0].obj_info_vect[0].id.data - 1] += 1
                        userdata.target_object = userdata.object_model_fitting[0].id.data
                        if(userdata.bin == 0):
                            userdata.temp_object_placement = 1
                        else:
                            userdata.temp_object_placement = 0
                        return 'succeeded'
                else:
                    userdata.partial_occlusion_detected[userdata.bin] = False
                    userdata.bin+=1
                    if (userdata.bin >= (userdata.no_of_bins) or userdata.non_target_state_initiated == True):
                        userdata.remove_non_target_object = 1
                        return 'preempted'
                    else:
                        return 'aborted'
            # For non-target objects and full occlusion.
            elif(userdata.non_target_object==True):
                if len(userdata.object_info) != 0:
                    if (userdata.debug == True):
                        print ("In fully occluded non-target object.")
                    userdata.object_model_fitting = []
                    userdata.object_model_fitting.append(userdata.object_info[0].obj_info_vect[0])
                    userdata.target_object = userdata.object_model_fitting[0].id.data
                    userdata.objects_attempt[userdata.object_info[0].obj_info_vect[0].id.data - 1] += 1
                    if (userdata.debug == True):
                        print ("Target object is : {}".format(userdata.objects[userdata.target_object - 1]))
                    return 'succeeded'
                else:
                    userdata.bin+=1
                    if (userdata.bin >= (userdata.no_of_bins)):
                        userdata.remove_non_target_object = 1
                        return 'preempted'
                    else:
                        return 'aborted'
        smach.StateMachine.add('OBJECT_DETECT_SERVICE',
                               ServiceState('/iitktcs/computer_vision_picking', computer_vision_stowing_picking,
                                            request_cb  =object_detect_service_request_cb,
                                            input_keys  =['ids_available',
                                                          'task',
                                                          'ids_target',
                                                          'bin_id',
                                                          'object_info',
                                                          'bin_cloud',
                                                          'bin',
                                                          'no_of_bins',
                                                          'task',
                                                          'bin_available_objects',
                                                          'bin_target_objects',
                                                          'target_object',
                                                          'reattempt_objects',
                                                          'non_target_object',
                                                          'remove_non_target_object',
                                                          'non_target_state_initiated',
                                                          'object_model_fitting',
                                                          'partial_occlusion',
                                                          'objects',
                                                          'partial_occlusion_detected',
                                                          'objects_attempt',
                                                          'attempt_at_last',
                                                          'temp_object_placement',
                                                          'state_start_time',
                                                          'state_stop_time',
                                                          'debug'],
                                            response_cb =object_detect_service_response_cb,
                                            output_keys =['object_info',
                                                          'bin_cloud',
                                                          'target_object',
                                                          'bin',
                                                          'remove_non_target_object',
                                                          'object_model_fitting',
                                                          'partial_occlusion',
                                                          'non_target_object',
                                                          'partial_occlusion_detected',
                                                          'objects_attempt',
                                                          'attempt_at_last',
                                                          'temp_object_placement',
                                                          'state_start_time',
                                                          'state_stop_time']),
                               transitions  ={'succeeded'       :   'POINT_CLOUD_PUBLISHER_FOR_OCTOMAP',
                                              'preempted'       :   'REATTEMPT',
                                              'aborted'         :   'TARGET_OBJECT_LIST_CHECK'
                                              },
                               remapping    ={'ids_available'   :   'ids_available',
                                              'task'            :   'task',
                                              'ids_target'      :   'ids_target',
                                              'bin_id'          :   'bin_id',
                                              'object_info'     :   'object_info',
                                              'bin_cloud'       :   'bin_cloud',
                                              'target_object'   :   'target_object'
                                              })

        #State to compensate the failure in approaching target object.
        smach.StateMachine.add('FAILED_TO_MOVE_TARGET_OBJECT', failed_to_move_target_object(),
                               transitions  ={'octomap_publisher'                   :   'POINT_CLOUD_PUBLISHER_FOR_OCTOMAP',
                                              'target_object_list_check'            :   'TARGET_OBJECT_LIST_CHECK',
                                              'reattempt'                           :   'REATTEMPT'},
                               remapping    ={'object_info'                         :   'object_info'})

        #State to publish static point cloud for octomap.
        def point_cloud_publisher_request_cb(userdata, request):
            point_clouds                = static_point_cloudRequest()
            #print point_clouds
            #### To Send Empty Bin Cloud
            #point_clouds.bin_cloud      = userdata.bin_cloud
            #point_clouds.object_cloud   = userdata.object_model_fitting[0].roi
            #point_clouds.flag.data      = True
            userdata.state_start_time = present_time_in_millisec().getTime()
            return point_clouds
        def point_cloud_publisher_response_cb(userdata, response):
            userdata.state_stop_time = present_time_in_millisec().getTime()
            print "Time taken for Point Cloud Publisher State is {} microsec.".format(userdata.state_stop_time-userdata.state_start_time)
            print " "
            return 'succeeded'
        smach.StateMachine.add('POINT_CLOUD_PUBLISHER_FOR_OCTOMAP',
                               ServiceState('/iitktcs/utils/octomap', static_point_cloud,
                                            request_cb =point_cloud_publisher_request_cb,
                                            input_keys =['object_info',
                                                         'bin_cloud',
                                                         'object_model_fitting',
                                                         'state_start_time',
                                                         'state_stop_time',
                                                         'debug'],
                                            response_cb =point_cloud_publisher_response_cb,
                                            output_keys =['state_start_time',
                                                          'state_stop_time']),
                               transitions  ={'succeeded'       :   'POSE_ESTIMATE_SERVICE',
                                              'aborted'         :   'FAILED_TO_MOVE_TARGET_OBJECT'},
                               remapping    ={'object_info'     :   'object_info',
                                              'bin_cloud'       :   'bin_cloud'})

        #Pose Estimation State
        def pose_estimate_service_request_cb(userdata, request):
            if (userdata.debug == True):
                print ("Target object is : {}".format(userdata.objects[userdata.target_object - 1]))
            pose_estimate                               =poseRequest()
            pose_estimate.object_info                   =userdata.object_model_fitting
            pose_estimate.bin_id.data                   =userdata.bin
            userdata.state_start_time                   = present_time_in_millisec().getTime()
            return pose_estimate
        def pose_estimate_service_response_cb(userdata, response):
            userdata.state_stop_time                    = present_time_in_millisec().getTime()
            print "Time taken for Pose Estimation State is {} microsec.".format(userdata.state_stop_time-userdata.state_start_time)
            print " "
            #print response
            userdata.object_position.object_pose                =response.pose
            userdata.object_position.axis                       =response.axis
            userdata.object_position.axis1                      =response.axis1
            userdata.object_position.angle                      =response.angle
            userdata.object_position.normal                     =response.normal
            userdata.object_position.centroid                   =response.centroid
            userdata.object_position.end_effector_link.data     ="Manish"
            userdata.object_position.object_id.data             =userdata.target_object
            userdata.object_position.bin_id.data                =userdata.bin
            #print userdata.object_position
            if(0):
                return 'preempted'
            else:
                return 'succeeded'
        smach.StateMachine.add('POSE_ESTIMATE_SERVICE',
                               ServiceState('/iitktcs/estimate_pose', pose,
                                            request_cb  =pose_estimate_service_request_cb,
                                            input_keys  =['object_model_fitting',
                                                          'id',
                                                          'roi',
                                                          'object_position',
                                                          'target_object',
                                                          'bin',
                                                          'objects',
                                                          'state_start_time',
                                                          'state_stop_time',
                                                          'debug'],
                                            response_cb =pose_estimate_service_response_cb,
                                            output_keys =['state_start_time',
                                                          'state_stop_time']),
                               transitions={'succeeded'     :   'ROBOT_POSE_GOAL',
                                            'preempted'     :   'GRASP_LOCATION_ESTIMATE_SERVICE',
                                            'aborted'       :   'FAILED_TO_MOVE_TARGET_OBJECT'},
                               remapping={'object_position' :   'object_position',
                                          'target_object'   :   'target_object'})

        # Grasp Location Estimation State
        def grasp_location_estimate_service_request_cb(userdata, request):
            grasp_pose                      =GraspPoseRequest()
            grasp_pose.obj_cloud            =userdata.object_model_fitting[0].roi
            grasp_pose.obj_provided.data    =True
            grasp_pose.bin_cloud            =userdata.bin_cloud
            grasp_pose.bin_provided.data    =True
            return grasp_pose
        def grasp_location_estimate_service_response_cb(userdata, response):
            print "Grasp handle found : {}".format(response.valid_handle_found.data)
            if(response.valid_handle_found.data==True):
                print "********************************************************************"
                # print response
                print "Handles are : {}".format(len(response.grasp_handles))
                print "********************************************************************"
                for i in range(0,len(response.grasp_handles)):
                    temp_grasp_location = grasp_pose()
                    temp_grasp_location.centroid    = response.grasp_handles[i].center.position
                    temp_grasp_location.axis        = response.grasp_handles[i].axis
                    temp_grasp_location.normal      = response.grasp_handles[i].normal
                    userdata.grasp_location.grasp_poses.append(temp_grasp_location)
                userdata.grasp_location.object_id.data  = userdata.target_object
                userdata.grasp_location.object_pose            = userdata.object_position.object_pose
                print "Moving towards Gripper Pose Goal to pick the object."
                return 'succeeded'
            else:
                print "Moving towards Suction Pose Goal to pick the object."
                return 'aborted'
        smach.StateMachine.add('GRASP_LOCATION_ESTIMATE_SERVICE',
                               ServiceState('iitktcs/grasping/GraspPose', GraspPose,
                                            request_cb  =grasp_location_estimate_service_request_cb,
                                            input_keys  =['grasp_location',
                                                          'object_info',
                                                          'bin_cloud',
                                                          'target_object',
                                                          'object_position',
                                                          'object_model_fitting',
                                                          'debug'],
                                            response_cb =grasp_location_estimate_service_response_cb,
                                            output_keys =['grasp_location']),
                               transitions={'succeeded'     : 'GRIPPER_POSE_GOAL',
                                            'aborted'       : 'ROBOT_POSE_GOAL'},
                               remapping={'object_position' : 'object_position',
                                          'target_object'   : 'target_object',
                                          'bin_cloud'       : 'bin_cloud',
                                          'grasp_location'  : 'grasp_location'})

        # Gripper Pose Goal State.
        def gripper_pose_goal_request_cb(userdata,request):
            print len(userdata.grasp_location.grasp_poses)
            #print (userdata.grasp_location)
            return userdata.grasp_location
        def gripper_pose_goal_response_cb(userdata,response):
            print "$$$$$$$$$$$$$$$$$$$$$$$$$$"
            print response
            if(response.success.data==True):
                return 'succeeded'
            else:
                return 'aborted'
        smach.StateMachine.add('GRIPPER_POSE_GOAL',
                               ServiceState('iitktcs/motion_planner/gripper_pose_goal', GripperPoseGoal,
                                            request_cb  =gripper_pose_goal_request_cb,
                                            input_keys  =['grasp_location',
                                                          'debug'],
                                            response_cb =gripper_pose_goal_response_cb,
                                            output_keys =[]),
                               transitions={'succeeded'     :   'ROBOT_GRIPPER_RETRIEVAL',
                                            'aborted'       :   'ROBOT_POSE_GOAL'},
                               remapping={'grasp_location'  :   'grasp_location'})

        # Suction Pose Goal State.
        def pose_goal_request_cb(userdata,request):
            #print userdata.object_position
            userdata.state_start_time = present_time_in_millisec().getTime()
            return userdata.object_position
        def pose_goal_response_cb(userdata,response):
            userdata.state_stop_time = present_time_in_millisec().getTime()
            print "Time taken for Suction Pose Goal State is {} microsec.".format(userdata.state_stop_time-userdata.state_start_time)
            print " "
            if (userdata.debug == True):
                print "Inside Suction Pose Goal."
                print "Response is {}".format(response)
            if(response.success.data==True):
                return 'succeeded'
            else:
                return 'aborted'
        smach.StateMachine.add('ROBOT_POSE_GOAL',
                               ServiceState('/iitktcs/motion_planner/suction_pose_goal', UR5PoseGoal,
                                            request_cb=pose_goal_request_cb,
                                            input_keys=['object_position',
                                                        'state_start_time',
                                                        'state_stop_time',
                                                        'debug'],
                                            response_cb =pose_goal_response_cb,
                                            output_keys =['state_start_time',
                                                          'state_stop_time']),
                               transitions={'succeeded' :   'ROBOT_SUCTION_START',
                                            'aborted'   :   'FAILURE'},
                               remapping={})

        # Robot Suction Start State.
        # Service call.
        def robot_suction_start_request_cb(userdata,request):
            suction_request = gripper_suction_controllerRequest()
            suction_request.vacuum_cleaner.data = 1
            userdata.state_start_time = present_time_in_millisec().getTime()
            return suction_request
        def robot_suction_start_response_cb(userdata,response):
            userdata.state_stop_time = present_time_in_millisec().getTime()
            print "Time taken for Suction Start State is {} microsec.".format(userdata.state_stop_time-userdata.state_start_time)
            print " "
            time.sleep(1)
            return 'succeeded'
        smach.StateMachine.add('ROBOT_SUCTION_START',
                               ServiceState('/iitktcs/gripper_suction_controller', gripper_suction_controller,
                                            request_cb  =robot_suction_start_request_cb,
                                            input_keys  =['state_start_time',
                                                          'state_stop_time',
                                                          'debug'],
                                            response_cb =robot_suction_start_response_cb,
                                            output_keys =['state_start_time',
                                                          'state_stop_time']),
                               transitions={'succeeded'     :   'ROBOT_RETRIEVAL',
                                            'aborted'       :   'ROBOT_FAILURE_SUCTION_STOP'},
                               remapping={})


        # # Back to bin centroid view position.
        # def single_goal_request_cb(userdata, request):
        #     pose = UR5GoalRequest()
        #     for i in range(0, 6):
        #         pose.goal.data.append(bin_centroid_angles[userdata.bin][i])
        #     return pose
        # def single_goal_response_cb(userdata, response):
        #     print response
        #     if(response.success.data == 0):
        #         userdata.reattempt_single_goal+=1
        #         if(userdata.reattempt_single_goal == 3):
        #             userdata.reattempt_single_goal = 0
        #             return 'aborted'
        #         else:
        #             return 'preempted'
        #     else:
        #         userdata.reattempt_single_goal = 0
        #         return 'succeeded'
        # smach.StateMachine.add('BACK_TO_BIN_VIEW_POSE',
        #                        ServiceState('/iitktcs/motion_planner/single_goal', UR5Goal,
        #                                     request_cb  =single_goal_request_cb,
        #                                     input_keys  =['bin',
        #                                                   'reattempt_single_goal'],
        #                                     response_cb = single_goal_response_cb,
        #                                     output_keys =['reattempt_single_goal']),
        #                        transitions = {'succeeded'   :   'DROP_POSE',
        #                                       'preempted'   :   'BACK_TO_BIN_VIEW_POSE',
        #                                       'aborted'     :   'ROBOT_FAILURE_SUCTION_STOP'})

        # Retrieval state
        def robot_retrieval_request_cb(userdata, request):
            retrieval_request = RetrievalRequest()
            retrieval_request.bin_id.data = userdata.bin
            userdata.state_start_time = present_time_in_millisec().getTime()
            return retrieval_request
        def robot_retrieval_response_cb(userdata, response):
            userdata.state_stop_time = present_time_in_millisec().getTime()
            print "Time taken for Retrieval State is {} microsec.".format(userdata.state_stop_time-userdata.state_start_time)
            print " "
            if (userdata.debug == True):
                print response
            if(response.success.data == False or response.object_present.data == False):
            #     print "This is a bug. Fix it for **************"
                userdata.reattempt_retrieval+=1
                if(userdata.reattempt_retrieval>=(userdata.reattempt_retrieval_max)):
                    userdata.reattempt_retrieval = 0
                    return 'aborted'
                else:
                    return 'preempted'
            # elif(response.success.data == True and response.object_present.data == False):
            #     print "This is a bug. Fix it for **************"
            else:
                userdata.reattempt_retrieval = 0
                return 'succeeded'
        smach.StateMachine.add('ROBOT_RETRIEVAL',
                               ServiceState('/iitktcs/motion_planner/retrieve_from_bin', Retrieval,
                                            request_cb  =robot_retrieval_request_cb,
                                            input_keys  =['bin',
                                                          'reattempt_retrieval',
                                                          'reattempt_retrieval_max',
                                                          'state_start_time',
                                                          'state_stop_time',
                                                          'debug'],
                                            response_cb =robot_retrieval_response_cb,
                                            output_keys =['reattempt_retrieval',
                                                          'state_start_time',
                                                          'state_stop_time']),
                               transitions  = {'succeeded'      :   'DROP_POSE',
                                               'preempted'      :   'ROBOT_RETRIEVAL',
                                               'aborted'        :   'ROBOT_FAILURE_SUCTION_STOP'})

        # Retrieval State For Gripper
        def robot_retrieval_request_cb(userdata, request):
            retrieval_request = RetrievalRequest()
            retrieval_request.bin_id.data = userdata.bin
            return retrieval_request
        def robot_retrieval_response_cb(userdata, response):
            if (userdata.debug == True):
                print response
            if(response.success.data == False or response.object_present.data == False):
                userdata.reattempt_retrieval+=1
                if(userdata.reattempt_retrieval>=(userdata.reattempt_retrieval_max)):
                    userdata.reattempt_retrieval = 0
                    return 'aborted'
                else:
                    return 'preempted'
            else:
                userdata.reattempt_retrieval = 0
                return 'succeeded'
        smach.StateMachine.add('ROBOT_GRIPPER_RETRIEVAL',
                               ServiceState('/iitktcs/motion_planner/retrieve_from_bin', Retrieval,
                                            request_cb  =robot_retrieval_request_cb,
                                            input_keys  =['bin',
                                                          'reattempt_retrieval',
                                                          'reattempt_retrieval_max',
                                                          'debug'],
                                            response_cb =robot_retrieval_response_cb,
                                            output_keys =['reattempt_retrieval']),
                               transitions  = {'succeeded'      :   'DROP_POSE_GRIPPER',
                                               'preempted'      :   'ROBOT_GRIPPER_RETRIEVAL',
                                               'aborted'        :   'FAILURE'})

        # Suction Go to tote position to drop the object.
        def single_goal_request_cb(userdata, request):
            pose = UR5GoalRequest()
            if (userdata.debug == True):
                print "Target object is : {}".format(userdata.target_object)
            if(userdata.non_target_object==False):
                index = -1
                for i in range(0, len(userdata.packing_box_objects)):
                    for j in range(0, len(userdata.packing_box_objects[i])):
                        if (userdata.target_object == userdata.packing_box_objects[i][j]):
                            index = i
                            break
                for i in range(0, 6):
                    pose.goal.data.append(drop_joints[index][userdata.drop_pose_sequence][i])
                if (userdata.debug == True):
                    print "Drop joint is : {}".format(drop_joints[index][userdata.drop_pose_sequence])
                pose.object_present.data = True
                if (userdata.drop_pose_sequence == 1):
                    pose.flag_remove_object.data    = True
                if (userdata.debug == True):
                    print "Pose is : {}".format(pose)
                userdata.state_start_time = present_time_in_millisec().getTime()
                return pose
            elif(userdata.non_target_object==True):
                index = -1
                # This has to be set one for the drop pose to go to a single drop location.
                userdata.drop_pose_sequence = 1
                for i in range(0,len(objects_per_drop_location[userdata.temp_object_placement])):
                    if(objects_per_drop_location[userdata.temp_object_placement][i]<1):
                        index = i
                        break
                if(index == -1):
                    index = 0
                    objects_per_drop_location[userdata.temp_object_placement] = [0,0]
                for i in range(0, 6):
                    pose.goal.data.append(temp_object_placement_joints[userdata.temp_object_placement][index][i])
                pose.flag_remove_object.data    = True
                pose.object_present.data        = True
                objects_per_drop_location[userdata.temp_object_placement][index]+=1
                userdata.state_start_time = present_time_in_millisec().getTime()
                return pose
        def single_goal_response_cb(userdata, response):
            userdata.state_stop_time = present_time_in_millisec().getTime()
            print "Time taken for Drop Pose State {} is {} microsec.".format(userdata.drop_pose_sequence+1,userdata.state_stop_time-userdata.state_start_time)
            print " "
            index = -1
            for i in range(0, len(userdata.packing_box_objects)):
                for j in range(0, len(userdata.packing_box_objects[i])):
                    if (userdata.target_object == userdata.packing_box_objects[i][j]):
                        index = i
                        break
            if (userdata.debug == True):
                print "Inside Suction Drop Pose Response CB."
                print "Response is {}".format(response)
            if (response.success.data == True):
                if(userdata.drop_pose_sequence == 0):
                    for i in range(0,len(objects_per_box_drop_location[index])):
                        if (objects_per_box_drop_location[index][i]<1):
                            userdata.drop_pose_sequence = i+1
                            objects_per_box_drop_location[index][i]+=1
                            print "Drop pose seq is : {}.".format(userdata.drop_pose_sequence)
                            break
                    return 'preempted'
                elif(userdata.drop_pose_sequence > 0 and userdata.drop_pose_sequence < 4):
                    print "Packing box seq is : {}".format(objects_per_box_drop_location[index])
                    if (userdata.drop_pose_sequence == 3):
                        objects_per_box_drop_location[index] = [0, 0, 0]
                    userdata.drop_pose_sequence = 0
                    return 'succeeded'
            else:
                return 'aborted'
        smach.StateMachine.add('DROP_POSE',
                               ServiceState('/iitktcs/motion_planner/single_goal', UR5Goal,
                                            request_cb  =single_goal_request_cb,
                                            input_keys  =['bin',
                                                          'target_object',
                                                          'packing_box_objects',
                                                          'temp_object_placement',
                                                          'non_target_object',
                                                          'drop_pose_sequence',
                                                          'state_start_time',
                                                          'state_stop_time',
                                                          'debug'],
                                            response_cb =single_goal_response_cb,
                                            output_keys =['drop_pose_sequence',
                                                          'state_start_time',
                                                          'state_stop_time']),
                               transitions = {'succeeded'   :   'ROBOT_SUCTION_STOP',
                                              'preempted'   :   'DROP_POSE',
                                              'aborted'     :   'ROBOT_FAILURE_SUCTION_STOP'})

        # Gripper Go to tote position to drop the object.
        def single_goal_request_cb(userdata, request):
            pose = UR5GoalRequest()
            print "Target object is : {}".format(userdata.target_object)
            if(userdata.non_target_object==False):
                index = -1
                for i in range(0, len(userdata.packing_box_objects)):
                    for j in range(0, len(userdata.packing_box_objects[i])):
                        if (userdata.target_object == userdata.packing_box_objects[i][j]):
                            index = i
                            break
                for i in range(0, 6):
                    pose.goal.data.append(drop_joints[index][i])
                pose.flag_remove_object.data    = True
                pose.object_present.data        = True
                return pose
            elif(userdata.non_target_object==True):
                index = -1
                for i in range(0,len(objects_per_drop_location[userdata.temp_object_placement])):
                    if(objects_per_drop_location[userdata.temp_object_placement][i]<1):
                        index = i
                        break
                if(index == -1):
                    index = 0
                    objects_per_drop_location[userdata.temp_object_placement] = [0,0]
                for i in range(0, 6):
                    pose.goal.data.append(temp_object_placement_joints[userdata.temp_object_placement][index][i])
                pose.flag_remove_object.data    = True
                pose.object_present.data        = True
                objects_per_drop_location[userdata.temp_object_placement][index]+=1
                return pose
        def single_goal_response_cb(userdata, response):
            print "Inside Gripper Drop Pose Response CB."
            print "Response is {}".format(response)
            if (response.success.data == True):
                publisher([0,2,0])
                publisher([0,2,1])
                # msg1 = Int16MultiArray()
                # msg2 = Int16MultiArray()
                # msg1.data = [0,2,0]
                # pub.publish(msg1)
                # msg2.data = [0,2,1]
                # pub.publish(msg2)
                return 'succeeded'
            else:
                return 'aborted'
        smach.StateMachine.add('DROP_POSE_GRIPPER',
                               ServiceState('/iitktcs/motion_planner/single_goal', UR5Goal,
                                            request_cb  =single_goal_request_cb,
                                            input_keys  =['bin',
                                                          'target_object',
                                                          'packing_box_objects',
                                                          'temp_object_placement',
                                                          'non_target_object',
                                                          'debug'],
                                            response_cb =single_goal_response_cb,
                                            output_keys =[]
                                            ),
                               transitions = {'succeeded'  :   'SUCCESS',
                                              'aborted'    :   'FAILURE'})

        # Robot Suction Stop State.
        def robot_suction_stop_request_cb(userdata,request):
            suction_request                     =gripper_suction_controllerRequest()
            suction_request.vacuum_cleaner.data =0
            userdata.state_start_time = present_time_in_millisec().getTime()
            return suction_request
        def robot_suction_stop_response_cb(userdata,response):
            userdata.state_stop_time = present_time_in_millisec().getTime()
            print "Time taken for Suction Stop state is {} microsec.".format(userdata.state_stop_time-userdata.state_start_time)
            print " "
            time.sleep(1)
            userdata.remove_object = False
            #stop = raw_input("Press any keys + Enter or Enter to stop. ")
            #print stop
            return 'succeeded'
        smach.StateMachine.add('ROBOT_SUCTION_STOP',
                               ServiceState('/iitktcs/gripper_suction_controller', gripper_suction_controller,
                                            request_cb  =robot_suction_stop_request_cb,
                                            input_keys  =['state_start_time',
                                                          'state_stop_time',
                                                          'debug'],
                                            response_cb =robot_suction_stop_response_cb,
                                            output_keys =['remove_object',
                                                          'state_start_time',
                                                          'state_stop_time']),
                               transitions={'succeeded'     :   'SUCCESS',
                                            'aborted'       :   'FAILURE'},
                               remapping={})

        # Robot Failure Suction Failure Stop State.
        def robot_failure_suction_stop_request_cb(userdata,request):
            suction_request                     =gripper_suction_controllerRequest()
            suction_request.vacuum_cleaner.data =0
            userdata.state_start_time = present_time_in_millisec().getTime()
            return suction_request
        def robot_failure_suction_stop_response_cb(userdata,response):
            userdata.state_stop_time = present_time_in_millisec().getTime()
            print "Time taken for Suction Failure Stop state is {} microsec.".format(userdata.state_stop_time-userdata.state_start_time)
            print " "
            time.sleep(3)
            userdata.remove_object = True
            return 'succeeded'
        smach.StateMachine.add('ROBOT_FAILURE_SUCTION_STOP',
                               ServiceState('/iitktcs/gripper_suction_controller', gripper_suction_controller,
                                            request_cb  =robot_failure_suction_stop_request_cb,
                                            input_keys  =['state_start_time',
                                                          'state_stop_time',
                                                          'debug'],
                                            response_cb =robot_failure_suction_stop_response_cb,
                                            output_keys =['remove_object',
                                                          'state_start_time',
                                                          'state_stop_time']),
                               transitions={'succeeded'     :   'FAILURE',
                                            'aborted'       :   'FAILURE'},
                               remapping={})


        # Success State
        smach.StateMachine.add('SUCCESS', Success(),
                               transitions={'success'               :   'JSON_WRITTER_STATE',
                                            'drop_pose_view'        :   'DROP_POSE_VIEW'},
                               remapping={'bin_available_objects'   :   'bin_available_objects',
                                          'bin_target_objects'      :   'bin_target_objects',
                                          'bin'                     :   'bin',
                                          'target_object'           :   'target_object'})

        # Single goal to come up.
        def single_goal_request_cb(userdata, request):
            pose = UR5GoalRequest()
            index = -1
            for i in range(0, len(userdata.packing_box_objects)):
                for j in range(0, len(userdata.packing_box_objects[i])):
                    if (userdata.target_object == userdata.packing_box_objects[i][j]):
                        index = i
                        break
            for i in range(0, 6):
                pose.goal.data.append(drop_joints_view[index][i])
            if (userdata.debug == True):
                print "Drop joint view is : {}".format(drop_joints_view[index])
            pose.object_present.data = False
            pose.flag_remove_object.data    = False
            if (userdata.debug == True):
                print "Pose is : {}".format(pose)
            userdata.state_start_time = present_time_in_millisec().getTime()
            return pose
        def single_goal_response_cb(userdata, response):
            userdata.state_stop_time = present_time_in_millisec().getTime()
            print "Time taken for Drop Pose View State {} is {} microsec.".format(userdata.drop_pose_sequence+1,userdata.state_stop_time-userdata.state_start_time)
            print " "
            if (response.success.data == True):
                return 'succeeded'
            else:
                return 'aborted'
        smach.StateMachine.add('DROP_POSE_VIEW',
                               ServiceState('/iitktcs/motion_planner/single_goal', UR5Goal,
                                            request_cb  =single_goal_request_cb,
                                            input_keys  =['bin',
                                                          'target_object',
                                                          'packing_box_objects',
                                                          'temp_object_placement',
                                                          'non_target_object',
                                                          'drop_pose_sequence',
                                                          'state_start_time',
                                                          'state_stop_time',
                                                          'debug'],
                                            response_cb =single_goal_response_cb,
                                            output_keys =['drop_pose_sequence',
                                                          'state_start_time',
                                                          'state_stop_time']),
                               transitions = {'succeeded'   :   'JSON_WRITTER_STATE',
                                              'aborted'     :   'DROP_POSE_VIEW'})


        # JSON Writter State
        def json_writter_request_cb(userdata,request):
            json_writter_request        = write_object_pick_statusRequest()
            for i in range(0,len(userdata.bin_available_objects)):
                bin_available_ids = custom_ids()
                bin_available_ids.ids.data = userdata.bin_available_objects[i]
                json_writter_request.ids_available.append(bin_available_ids)
            for i in range(0,len(userdata.picked_objects)):
                box_available_ids = custom_ids()
                box_available_ids.ids.data = userdata.picked_objects[i]
                json_writter_request.ids_picked.append(box_available_ids)
            if (userdata.debug == True):
                print "JSON Writter Request is : {}".format(json_writter_request)
            userdata.state_start_time = present_time_in_millisec().getTime()
            return json_writter_request
        def json_writter_response_cb(userdata,response):
            userdata.state_stop_time = present_time_in_millisec().getTime()
            print "Time taken for JSON Writter State is {} microsec.".format(userdata.state_stop_time-userdata.state_start_time)
            print " "
            if (len(userdata.bin_target_objects[userdata.bin]) == 0 and userdata.bin < userdata.no_of_bins):
                userdata.bin = userdata.bin+1
                if (userdata.bin >= userdata.no_of_bins):
                    return 'preempted'
            return 'succeeded'
        smach.StateMachine.add('JSON_WRITTER_STATE',
                               ServiceState('/iitktcs/pick_object_status_service', write_object_pick_status,
                                            request_cb  =json_writter_request_cb,
                                            input_keys  =['bin_target_objects',
                                                          'bin_available_objects',
                                                          'picked_objects',
                                                          'bin',
                                                          'no_of_bins',
                                                          'state_start_time',
                                                          'state_stop_time',
                                                          'debug'],
                                            response_cb =json_writter_response_cb,
                                            output_keys =['bin',
                                                          'state_start_time',
                                                          'state_stop_time']),
                               transitions={'succeeded'     :   'TARGET_OBJECT_LIST_CHECK',
                                            'preempted'     :   'REATTEMPT',
                                            'aborted'       :   'JSON_WRITTER_STATE'},
                               remapping={})

        # Failure State
        smach.StateMachine.add('FAILURE', Failure(),
                               transitions={'failure'               :   'TARGET_OBJECT_LIST_CHECK',
                                            'reattempt'             :   'REATTEMPT'},
                               remapping={'bin_available_objects'   :   'bin_available_objects',
                                          'bin_target_objects'      :   'bin_target_objects',
                                          'bin'                     :   'bin',
                                          'target_object'           :   'target_object'})

        # Reattempt state.
        smach.StateMachine.add('REATTEMPT', reattempt(),
                               transitions  ={'target_object_list_check'    :   'TARGET_OBJECT_LIST_CHECK',
                                              'remove_non_target_object'    :   'REMOVE_NON_TARGET_OBJECT',
                                              'rest'                        :   'REST_POSE'},
                               remapping    ={})

        # Remove non-target object.
        smach.StateMachine.add('REMOVE_NON_TARGET_OBJECT', remove_non_target_object(),
                               transitions  ={'bin_view'                    :   'BIN_VIEW_POSE'},
                               remapping    ={})

        # Robot Rest position.
        def single_goal_request_cb(userdata, request):
            pose = UR5GoalRequest()
            for i in range(0, 6):
                pose.goal.data.append(rest_joints[i])
            return pose
        smach.StateMachine.add('REST_POSE',
                               ServiceState('/iitktcs/motion_planner/single_goal', UR5Goal,
                                            request_cb =single_goal_request_cb,
                                            input_keys =['bin']),
                               transitions = {'succeeded'  :   'FINISH',
                                              'aborted'    :   'REST_POSE'})

        # Finish
        smach.StateMachine.add('FINISH', Finish(),
                               transitions={'complete'      :   'succeeded'})
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    main()
