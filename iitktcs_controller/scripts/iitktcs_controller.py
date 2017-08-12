#!/usr/bin/env python

import roslib; roslib.load_manifest('iitktcs_controller')
import rospy
import smach
from smach_ros import IntrospectionServer
from smach_ros import ServiceState
from open_gripper import publisher
from operator import itemgetter

# Import services.
from json_maker.srv import *
from iitktcs_msgs_srvs.srv import *

# Import messages.
from iitktcs_msgs_srvs.msg import *

# Import data.
import time

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['start'],
                             input_keys     =['start_time',
                                              'objects',
                                              'objects_attempt'],
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
        print "System started at {} secs.".format(userdata.start_time)
        return 'start'

class Success(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['success'],
                             input_keys     =['bin_available_objects',
                                              'bin_target_objects',
                                              'bin',
                                              'no_of_bins',
                                              'target_object',
                                              'picked_objects',
                                              'temp_objects',
                                              'non_target_object',
                                              'temp_object_placement',
                                              'packing_box_objects'],
                             output_keys    =['bin',
                                              'non_target_object',
                                              'remove_non_target_object',
                                              'temp_objects'])
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
            return 'success'
        elif(userdata.non_target_object==True):
            userdata.bin_available_objects[userdata.bin].remove(userdata.target_object)
            userdata.bin_available_objects[userdata.temp_object_placement].append(userdata.target_object)
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
                                              'objects_attempt'],
                             output_keys    =['bin_target_objects',
                                              'bin',
                                              'attempt_at_last',
                                              'objects_attempt'])
    def execute(selfself, userdata):
        print "Target object before attempt check is : {}".format(userdata.bin_target_objects)

        temp_bin_target = []
        for i in range(0, len(userdata.bin_target_objects[userdata.bin])):
            if (userdata.objects_attempt[userdata.bin_target_objects[userdata.bin][i] - 1] > 1):
                print "Inside attempt at last. {}".format(userdata.bin_target_objects[userdata.bin][i])
                userdata.attempt_at_last[userdata.bin].append(userdata.bin_target_objects[userdata.bin][i])
            else:
                print "Else loop . {}".format(userdata.bin_target_objects[userdata.bin][i])
                temp_bin_target.append(userdata.bin_target_objects[userdata.bin][i])
        print "Objects to attempt at last : {}".format(userdata.attempt_at_last)
        # for i in range(0, len(userdata.attempt_at_last[userdata.bin])):
        #     if userdata.attempt_at_last[userdata.bin][i] in userdata.bin_target_objects[userdata.bin]:
        #         userdata.bin_target_objects[userdata.bin].remove(userdata.attempt_at_last[userdata.bin][i])
        userdata.bin_target_objects[userdata.bin] = temp_bin_target
        print "Target object after attempt check is : {}".format(userdata.bin_target_objects)

        if(len(userdata.bin_target_objects[userdata.bin])==0):
            if(userdata.bin<(userdata.no_of_bins-1)):
                print "Bin is ======== {}".format(userdata.bin)
                userdata.bin+=1
                return 'check_next_bin'
            else:
                print "Bin is ******** {}".format(userdata.bin)
                return 'absent'
        else:
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
                                              'object_model_fitting'],
                             output_keys    =['bin',
                                              'target_object',
                                              'object_model_fitting'])
    def execute(self, userdata):
        if(userdata.non_target_object==False):
            userdata.reattempt_objects[userdata.bin].append(userdata.object_info[0].obj_info_vect[0].id.data)
            # userdata.bin_available_objects[userdata.bin].remove(userdata.object_info[0].id.data)
            userdata.bin_target_objects[userdata.bin].remove(userdata.object_info[0].obj_info_vect[0].id.data)
            userdata.object_info.pop(0)
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
                                              'objects_attempt'],
                             output_keys    =['bin_available_objects',
                                              'bin_target_objects',
                                              'reattempt_objects',
                                              'bin',
                                              'remove_non_target_object',
                                              'objects_attempt'])
    def execute(self, userdata):
        size = 0
        attempt_last = 0
        bin_partial_occlusion = False
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
        print "Partial Occlusion for bins are : {}".format(userdata.partial_occlusion_detected)
        for i in range(0, len(userdata.partial_occlusion_detected)):
            if(userdata.partial_occlusion_detected[i]==True):
                bin_partial_occlusion = True
        print "Flag for partial occlusion is : {}".format(bin_partial_occlusion)
        if(size == 0 and attempt_last == 0):
            return 'rest'
        elif(size == 0 and attempt_last == 1):
            userdata.remove_non_target_object = 1
            userdata.bin = 0
            for i in range(0,len(userdata.attempt_at_last)):
                for i in range(0,len(userdata.attempt_at_last[i])):
                    userdata.objects_attempt[userdata.attempt_at_last[i][j]-1] = 0
            for i in range(0,len(userdata.bin_target_objects)):
                userdata.bin_target_objects[i] += userdata.attempt_at_last[i]
            userdata.attempt_at_last = [[],[]]
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
                                              'non_target_object'],
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
                                              'non_target_object'],
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
        print size
        index_max                           = size.index(max(size))
        index_min                           = size.index(min(size))
        if (index_max == index_min):
            userdata.temp_object_placement  = (userdata.no_of_bins-1)
            userdata.bin                    = 0
        else:
            userdata.temp_object_placement  = index_min
            userdata.bin                    = index_max
        print "Bin to remove from is : {}".format(userdata.bin)
        print "Place to keep the object at : {}".format(userdata.temp_object_placement)
        return 'bin_view'

class Finish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['complete'])
    def execute(self,userdata):
        return 'complete'

def main():
    # objects = ["Toilet_Brush",          "Avery_Binder",         "Balloons",                 "Band_Aid_Tape",#4
    #            "Bath_Sponge",           "Black_Fashion_Gloves", "Burts_Bees_Baby_Wipes",    "Colgate_Toothbrush_4PK",#8
    #            "Composition_Book",      "Crayons",              "Duct_Tape",                "Epsom_Salts",#12
    #            "Expo_Eraser",           "Fiskars_Scissors",     "Flashlight",               "Glue_Sticks",#16
    #            "Hand_Weight",           "Hanes_Socks",          "Hinged_Ruled_Index_Cards", "Ice_Cube_Tray",#20
    #            "Irish_Spring_Soap",     "Laugh_Out_Loud_Jokes", "Marbles",                  "Measuring_Spoons",#24
    #            "Mesh_Cup",              "Mouse_Traps",          "Pie_Plates",               "Plastic_Wine_Glass",#28
    #            "Poland_Spring_Water",   "Reynolds_Wrap",        "Robots_DVD",               "Robots_Everywhere",#32
    #            "Scotch_Sponges",        "Speed_Stick",          "White_Facecloth",          "Table_Cloth",#36
    #            "Tennis_Ball_Container", "Ticonderoga_Pencils",  "Tissue_Box",               "Windex"]#40

    # Temp Objects Drop Position ID.
    # drop_id = [-1, -1, -1, -1,#4
    #            -1, -1, -1, -1,#8
    #            -1, -1, -1, -1,#12
    #            -1, -1, -1, -1,#16
    #            -1, -1, -1, -1,#20
    #            -1, -1, -1, -1,#24
    #            -1, -1, -1, -1,#28
    #            -1, -1, -1, -1,#32
    #            -1, -1, -1, -1,#36
    #            -1, -1, -1, -1,#40
    #            ]

    # Bin angles of bin view.
    bin_angles = [[-0.0109823, -1.56805, 1.37091, -1.11577, -1.58023, 0.0018583],
                  [-0.570878, -1.5398, 1.35223, -1.16178, -1.71734, -0.532907]]
    # Non-target object temp. position.
    temp_pose = [-1.53369, -1.49637, 1.7334, -1.757, -1.49645, -0.297381]
    # Non-Target Object Placement.
    temp_object_placement_joints =[[[0.0233202, -1.15814, 1.70757, -2.07361, -1.57921, 0.800493],[0.0625764, -1.34026, 1.96991, -2.1541, -1.575, 0.839319]],
                                   [[-0.552563, -1.35272, 1.98701, -2.16316, -1.60329, 0.224726],[-0.462898, -1.10364, 1.63139, -2.05489, -1.60244, 0.314815]]]
    objects_per_drop_location = [[0,0],[0,0]]
    # Drop joint angles
    drop_joints = [[1.82685, -1.43837, 1.63587, -1.52443, -1.52538, 0.265151],
                   [1.39877, -1.57822, 1.78443, -1.53032, -1.62971, -0.150361],
                   [0.858659, -1.42812, 1.63514, -1.58902, -1.75076, -0.680978],
                   [-2.07378, -1.5265, 1.74689, -1.61961, -1.67912, -0.439727],
                   [-1.2148, -1.36933, 1.56644, -1.58345, -1.51837, 0.40552]]
    # Failure Drop Joint Angles
    failure_drop_joints =  [[1.796, -1.15715, 1.749, -2.10737, -1.53445, 0.230635],
                            [1.35947, -1.16626, 1.89799, -2.23129, -1.56127, -0.206432],
                            [0.962412, -1.07528, 1.73768, -2.15932, -1.59198, -0.602013],
                            [-1.9343, -1.09591, 1.75377, -2.15534, -1.71637, -0.44812],
                            [-1.32455, -1.05343, 1.68856, -2.0705, -1.65106, 0.162223]]
    # Rest pose joint angles
    rest_joints = [-0.0712016, -2.71543, 2.6089, -2.8294, -1.49305, -0.0444363]

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
            return bin_index
        def read_object_list_from_json_response_cb(userdata, response):
            for i in range(0,len(response.ids_available)):
                userdata.bin_available_objects.append(list(response.ids_available[i].ids.data))
            for i in range(0,len(response.ids_target)):
                userdata.packing_box_objects.append(list(response.ids_target[i].ids.data))
            userdata.task = response.task.data
            target = []
            for i in range(0,len(userdata.packing_box_objects)):
                for j in range(0,len(userdata.packing_box_objects[i])):
                    target.append(userdata.packing_box_objects[i][j])
            print "Target is : {}".format(target)
            for i in range(0,len(userdata.bin_available_objects)):
                for j in range(0,len(target)):
                    if target[j] in userdata.bin_available_objects[i]:
                        userdata.bin_target_objects[i].append(target[j])
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
                                                          'task'],
                                            response_cb =read_object_list_from_json_response_cb,
                                            output_keys =['bin_available_objects',
                                                          'bin_target_objects',
                                                          'packing_box_objects',
                                                          'task']),
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
            print "Bin is {}".format(userdata.bin)
            for i in range(0, 6):
                pose.goal.data.append(bin_angles[userdata.bin][i])
            if(userdata.remove_object == True):
                pose.flag_remove_object.data = True
            else:
                pose.flag_remove_object.data = False
            return pose
        def single_goal_response_cb(userdata, response):
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
                                                          'remove_object'],
                                            response_cb =single_goal_response_cb,
                                            output_keys =['remove_object']),
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
                print "Objects to attempt at last : {}".format(userdata.attempt_at_last)
                print "Target objects in bin {} are {}".format(userdata.bin, userdata.bin_target_objects[userdata.bin])
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
                for i in range(0,len(temp_target)):
                    if not (userdata.objects_attempt[temp_target[i] - 1] > 2):
                        object_det.ids_target.data.append(temp_target[i])
                print userdata.bin_available_objects[userdata.bin]
                print userdata.bin_target_objects[userdata.bin]
                print "Target objects in bin {} are {}".format(userdata.bin, object_det.ids_target.data)
                return object_det
        def object_detect_service_response_cb(userdata, response):
            userdata.object_info                                    = response.object_info
            userdata.bin_cloud                                      = response.bin_cloud
            userdata.partial_occlusion[userdata.bin]                = False
            # For target objects.
            if(userdata.non_target_object==False):
                # Sort on the basis of length of sub-list.
                userdata.object_info.sort(lambda x,y: cmp(len(x.obj_info_vect), len(y.obj_info_vect)))
                temp_available_object_ids   = []
                temp_unavailable_object_ids = []
                print "For bin {} length of target object is {}".format(userdata.bin,len(userdata.object_info))
                for i in range(0,len(userdata.object_info)):
                    print "Object id {} and object is {}.".format(userdata.object_info[i].obj_info_vect[0].id.data,userdata.objects[userdata.object_info[i].obj_info_vect[0].id.data-1])
                    temp_available_object_ids.append(userdata.object_info[i].obj_info_vect[0].id.data)
                for i in range(0,len(userdata.bin_target_objects[userdata.bin])):
                    if not userdata.bin_target_objects[userdata.bin][i] in temp_available_object_ids:
                        temp_unavailable_object_ids.append(userdata.bin_target_objects[userdata.bin][i])
                print "temp_unavailable_object_ids are : {}".format(temp_unavailable_object_ids)
                for i in range(0,len(temp_unavailable_object_ids)):
                    userdata.reattempt_objects[userdata.bin].append(temp_unavailable_object_ids[i])
                    # userdata.bin_available_objects[userdata.bin].remove(temp_unavailable_object_ids[i])
                    userdata.bin_target_objects[userdata.bin].remove(temp_unavailable_object_ids[i])
                if len(userdata.object_info) != 0:
                    for i in range (0,len(userdata.object_info)):
                        if(len(userdata.object_info[i].obj_info_vect)>1):
                            userdata.partial_occlusion_detected[userdata.bin] = True
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
                            print "Initial size for target {} is {}.".format(userdata.object_info[i].obj_info_vect[0].id.data,len(userdata.object_info[i].obj_info_vect))
                            if (len(userdata.object_info[i].obj_info_vect)>1):
                                print "Test {}".format(i)
                                index_to_remove = i
                                break
                        for i in range(0, len(userdata.object_info)):
                            # print userdata.objects(userdata.object_info[i].obj_info_vect[0].id.data-1)
                            print userdata.object_info[i].obj_info_vect[0].id.data
                        userdata.object_info = userdata.object_info[0:index_to_remove]
                        for i in range(0, len(userdata.object_info)):
                            print "To pick."
                            # print userdata.objects(userdata.object_info[i].obj_info_vect[0].id.data-1)
                            print "Size for target {} is {}.".format(userdata.object_info[i].obj_info_vect[0].id.data,len(userdata.object_info[i].obj_info_vect))
                        print "Model ID is {}".format(userdata.object_info[0].obj_info_vect[0].id.data)
                        userdata.object_model_fitting = []
                        userdata.object_model_fitting.append(userdata.object_info[0].obj_info_vect[0])
                        userdata.target_object = userdata.object_model_fitting[0].id.data
                        userdata.objects_attempt[userdata.target_object - 1] += 1
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
                    print ("In fully occluded non-target object.")
                    userdata.object_model_fitting = []
                    userdata.object_model_fitting.append(userdata.object_info[0].obj_info_vect[0])
                    userdata.target_object = userdata.object_model_fitting[0].id.data
                    userdata.objects_attempt[userdata.object_info[0].obj_info_vect[0].id.data - 1] += 1
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
                                                          'temp_object_placement'],
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
                                                          'temp_object_placement']),
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
            return point_clouds
        smach.StateMachine.add('POINT_CLOUD_PUBLISHER_FOR_OCTOMAP',
                               ServiceState('/iitktcs/utils/octomap', static_point_cloud,
                                            request_cb =point_cloud_publisher_request_cb,
                                            input_keys =['object_info',
                                                         'bin_cloud',
                                                         'object_model_fitting']),
                               transitions  ={'succeeded'       :   'POSE_ESTIMATE_SERVICE',
                                              'aborted'         :   'FAILED_TO_MOVE_TARGET_OBJECT'},
                               remapping    ={'object_info'     :   'object_info',
                                              'bin_cloud'       :   'bin_cloud'})

        #Pose Estimation State
        def pose_estimate_service_request_cb(userdata, request):
            print ("Target object is : {}".format(userdata.objects[userdata.target_object - 1]))
            pose_estimate                               =poseRequest()
            pose_estimate.object_info                   =userdata.object_model_fitting
            pose_estimate.bin_id.data                   =userdata.bin
            return pose_estimate
        def pose_estimate_service_response_cb(userdata, response):
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
                                                          'objects'],
                                            response_cb =pose_estimate_service_response_cb,
                                            output_keys =[]),
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
                                                          'object_model_fitting'],
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
                                            input_keys  =['grasp_location'],
                                            response_cb =gripper_pose_goal_response_cb,
                                            output_keys =[]),
                               transitions={'succeeded'     :   'ROBOT_GRIPPER_RETRIEVAL',
                                            'aborted'       :   'ROBOT_POSE_GOAL'},
                               remapping={'grasp_location'  :   'grasp_location'})

        # Suction Pose Goal State.
        def pose_goal_request_cb(userdata,request):
            #print userdata.object_position
            return userdata.object_position
        def pose_goal_response_cb(userdata,response):
            print "Inside Suction Pose Goal."
            print "Response is {}".format(response)
            if(response.success.data==True):
                return 'succeeded'
            else:
                return 'aborted'
        smach.StateMachine.add('ROBOT_POSE_GOAL',
                               ServiceState('/iitktcs/motion_planner/suction_pose_goal', UR5PoseGoal,
                                            request_cb=pose_goal_request_cb,
                                            input_keys=['object_position'],
                                            response_cb =pose_goal_response_cb,
                                            output_keys =[]),
                               transitions={'succeeded' :   'ROBOT_SUCTION_START',
                                            'aborted'   :   'FAILURE'},
                               remapping={})

        # Robot Suction Start State.
        # Service call.
        def robot_suction_start_request_cb(userdata,request):
            suction_request = gripper_suction_controllerRequest()
            suction_request.vacuum_cleaner.data = 1
            return suction_request
        def robot_suction_start_response_cb(userdata,response):
            time.sleep(1)
            return 'succeeded'
        smach.StateMachine.add('ROBOT_SUCTION_START',
                               ServiceState('/iitktcs/gripper_suction_controller', gripper_suction_controller,
                                            request_cb  =robot_suction_start_request_cb,
                                            input_keys  =[''],
                                            response_cb =robot_suction_start_response_cb,
                                            output_keys =['']),
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
            return retrieval_request
        def robot_retrieval_response_cb(userdata, response):
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
                                                          'reattempt_retrieval_max'],
                                            response_cb =robot_retrieval_response_cb,
                                            output_keys =['reattempt_retrieval']),
                               transitions  = {'succeeded'      :   'DROP_POSE',
                                               'preempted'      :   'ROBOT_RETRIEVAL',
                                               'aborted'        :   'ROBOT_FAILURE_SUCTION_STOP'})

        # Retrieval State For Gripper
        def robot_retrieval_request_cb(userdata, request):
            retrieval_request = RetrievalRequest()
            retrieval_request.bin_id.data = userdata.bin
            return retrieval_request
        def robot_retrieval_response_cb(userdata, response):
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
                                                          'reattempt_retrieval_max'],
                                            response_cb =robot_retrieval_response_cb,
                                            output_keys =['reattempt_retrieval']),
                               transitions  = {'succeeded'      :   'DROP_POSE_GRIPPER',
                                               'preempted'      :   'ROBOT_GRIPPER_RETRIEVAL',
                                               'aborted'        :   'FAILURE'})


        # Suction Go to box position to drop the object.
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
                print "Drop joint is : {}".format(drop_joints[index])
                pose.object_present.data = True
                pose.flag_remove_object.data = False
                print "Pose is : {}".format(pose)
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
            time.sleep(3)
            print "Inside Suction Drop Pose Response CB."
            print "Response is {}".format(response)
            if (response.success.data == True):
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
                                                          'drop_pose_sequence'],
                                            response_cb =single_goal_response_cb,
                                            output_keys =['drop_pose_sequence']
                                            ),
                               transitions = {'succeeded'   :   'VALLEY_DETECTION',
                                              'aborted'     :   'ROBOT_FAILURE_SUCTION_STOP'})

        # Valley Detection Code.
        def valley_detection_request_cb(userdata, request):
            valley_request = detect_valleyRequest()
            index = -1
            for i in range(0, len(userdata.packing_box_objects)):
                for j in range(0, len(userdata.packing_box_objects[i])):
                    if (userdata.target_object == userdata.packing_box_objects[i][j]):
                        index = i
                        break
            valley_request.box_id.data = index
            print "Request to valley detection is {}".format(valley_request)
            return valley_request
        def valley_detection_response_cb(userdata, response):
            print response
            userdata.valley_drop_request.centroid       = response.valley[0].center.position
            userdata.valley_drop_request.axis           = response.valley[0].major_axis
            userdata.valley_drop_request.axis1          = response.valley[0].minor_axis
            return 'succeeded'
        smach.StateMachine.add('VALLEY_DETECTION',
                               ServiceState('/iitktcs/detect_valley', detect_valley,
                                            request_cb  =valley_detection_request_cb,
                                            input_keys  =['packing_box_objects',
                                                          'target_object',
                                                          'valley_drop_request'],
                                            response_cb =valley_detection_response_cb,
                                            output_keys =['valley_drop_request']
                                            ),
                               transitions = {'succeeded'   :   'DROP_TO_VALLEY',
                                              'aborted'     :   'VALLEY_FAILURE_DROP_TO_BOX_CENTRE'})

        # Drop to valley.
        def drop_to_valley_request_cb(userdata, request):
            drop_to_valley_request = userdata.valley_drop_request
            print "Request for drop to valley is {}".format(drop_to_valley_request)
            return drop_to_valley_request
        def drop_to_valley_response_cb(userdata, response):
            print response
            if(response.success.data == True):
                return 'succeeded'
            else:
                return 'aborted'
        smach.StateMachine.add('DROP_TO_VALLEY',
                               ServiceState('/iitktcs/motion_planner/valley_planning', UR5PoseGoal,
                                            request_cb  = drop_to_valley_request_cb,
                                            input_keys  = ['valley_drop_request'],
                                            response_cb = drop_to_valley_response_cb,
                                            output_keys = []
                                            ),
                               transitions = {'succeeded'   :   'ROBOT_SUCTION_STOP',
                                              'aborted'     :   'VALLEY_FAILURE_DROP_TO_BOX_CENTRE'})

        # Valley Detection Failure backup plan.
        def single_goal_request_cb(userdata, request):
            pose = UR5GoalRequest()
            print "Target object is : {}".format(userdata.target_object)
            index = -1
            for i in range(0, len(userdata.packing_box_objects)):
                for j in range(0, len(userdata.packing_box_objects[i])):
                    if (userdata.target_object == userdata.packing_box_objects[i][j]):
                        index = i
                        break
            for i in range(0, 6):
                pose.goal.data.append(failure_drop_joints[index][i])
            print "Drop joint is : {}".format(drop_joints[index])
            pose.object_present.data = True
            pose.flag_remove_object.data = True
            print "Pose is : {}".format(pose)
            return pose
        def single_goal_response_cb(userdata, response):
            print "Inside failed valley detection state."
            print "Response is {}".format(response)
            if (response.success.data == True):
                return 'succeeded'
            else:
                return 'aborted'
        smach.StateMachine.add('VALLEY_FAILURE_DROP_TO_BOX_CENTRE',
                               ServiceState('/iitktcs/motion_planner/single_goal', UR5Goal,
                                            request_cb=single_goal_request_cb,
                                            input_keys=['bin',
                                                        'target_object',
                                                        'packing_box_objects',
                                                        'temp_object_placement',
                                                        'non_target_object'],
                                            response_cb=single_goal_response_cb,
                                            output_keys=[]
                                            ),
                               transitions={'succeeded' : 'ROBOT_SUCTION_STOP',
                                            'aborted'   : 'ROBOT_FAILURE_SUCTION_STOP'})

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
                                                          'non_target_object'],
                                            response_cb =single_goal_response_cb,
                                            output_keys =[]
                                            ),
                               transitions = {'succeeded'  :   'SUCCESS',
                                              'aborted'    :   'FAILURE'})

        # Robot Suction Stop State.
        def robot_suction_stop_request_cb(userdata,request):
            suction_request                     =gripper_suction_controllerRequest()
            suction_request.vacuum_cleaner.data =0
            return suction_request
        def robot_suction_stop_response_cb(userdata,response):
            time.sleep(1)
            userdata.remove_object = False
            #stop = raw_input("Press any keys + Enter or Enter to stop. ")
            #print stop
            return 'succeeded'
        smach.StateMachine.add('ROBOT_SUCTION_STOP',
                               ServiceState('/iitktcs/gripper_suction_controller', gripper_suction_controller,
                                            request_cb  =robot_suction_stop_request_cb,
                                            input_keys  =[''],
                                            response_cb =robot_suction_stop_response_cb,
                                            output_keys =['remove_object']),
                               transitions={'succeeded'     :   'SUCCESS',
                                            'aborted'       :   'FAILURE'},
                               remapping={})

        # Robot Failure Suction Failure Stop State.
        def robot_failure_suction_stop_request_cb(userdata,request):
            suction_request                     =gripper_suction_controllerRequest()
            suction_request.vacuum_cleaner.data =0
            return suction_request
        def robot_failure_suction_stop_response_cb(userdata,response):
            time.sleep(3)
            userdata.remove_object = True
            return 'succeeded'
        smach.StateMachine.add('ROBOT_FAILURE_SUCTION_STOP',
                               ServiceState('/iitktcs/gripper_suction_controller', gripper_suction_controller,
                                            request_cb  =robot_failure_suction_stop_request_cb,
                                            input_keys  =[''],
                                            response_cb =robot_failure_suction_stop_response_cb,
                                            output_keys =['remove_object']),
                               transitions={'succeeded'     :   'FAILURE',
                                            'aborted'       :   'FAILURE'},
                               remapping={})


        # Success State
        smach.StateMachine.add('SUCCESS', Success(),
                               transitions={'success'               :   'JSON_WRITTER_STATE'},
                               remapping={'bin_available_objects'   :   'bin_available_objects',
                                          'bin_target_objects'      :   'bin_target_objects',
                                          'bin'                     :   'bin',
                                          'target_object'           :   'target_object'})

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
            print "JSON Writter Request is : {}".format(json_writter_request)
            return json_writter_request
        def json_writter_response_cb(userdata,response):
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
                                                          'no_of_bins'],
                                            response_cb =json_writter_response_cb,
                                            output_keys =['bin']),
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
