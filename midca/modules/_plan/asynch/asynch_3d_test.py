from midca import rosrun, plans
from midca import midcatime
import traceback
import math
import copy
import numpy as np

try:
    from geometry_msgs.msg import PointStamped
    from geometry_msgs.msg import Point
    from MIDCA.examples import ObjectDetector
    from std_msgs.msg import String
    from scipy.spatial import distance
except:
    pass  # if ROS is not installed, an error message will already have been generated.

END_COLOR_CODE = '\033[0m'
NOT_STARTED = 0
NS_COLOR_CODE = END_COLOR_CODE
IN_PROGRESS = 1
IP_COLOR_CODE = '\033[92m'
COMPLETE = 2
C_COLOR_CODE = '\033[94m'
FAILED = 3
F_COLOR_CODE = '\033[91m'

FEEDBACK_KEY = "code"
CMD_ID_KEY = "cmd_id"
POINT_TOPIC = "point_cmd"
LOC_TOPIC = "loc_cmd"
GRAB_TOPIC = "grabbing_cmd"
RAISE_TOPIC = "raise_cmd"
RELEASE_TOPIC = "release_cmd"
# set this to change output for all asynch actions.
verbose = 2

MAX_SIGHTING_LAG = 3.0
MAX_SIGHTING_WAIT = 5.0


def get_asynch_action(midcaAction):
    raise ArgumentException("midca action " + str(midcaAction) + " does not translate to a \
    valid asynchronous action.")


def asynch_plan(mem, midcaPlan):
    '''
    returns an asynchronous plan that corresponds to the given MIDCA plan.
    '''
    actions = []
    goals = midcaPlan.goals
    for midcaAction in midcaPlan.actions:
        if midcaAction.op == "communicate":
            actions.append(GraceCommunicate(mem, midcaAction))

        elif midcaAction.op == "clean":
            actions.append(GraceClean(mem, midcaAction))

        elif midcaAction.op == "dive":
            actions.append(GraceDive(mem, midcaAction))

        elif midcaAction.op == "raise":
            actions.append(GraceRaise(mem, midcaAction))

        elif midcaAction.op == "sensedepth":
            actions.append(GraceSense(mem, midcaAction))

        elif midcaAction.op == "recorddepth":
            actions.append(GraceSense(mem, midcaAction))

        else:
            if verbose >= 1:
                print "MIDCA action", midcaAction, "does not correspond to an asynch",
                "action. MIDCA will skip this action"
    return AsynchPlan(actions, goals)


class AsynchPlan(plans.Plan):
    '''
    subclass of MIDCA Plan class that uses asynchronous actions.
    '''

    def finished(self):
        '''
        overrides plan.finished(). Declares a plan complete if all its actions report
        complete or failed.
        '''
        for action in self.actions:
            if action.status != COMPLETE and action.status != FAILED:
                return False
        return True

    @property
    def status(self):
        '''
        property that returns the plan's status. This can be NOT_STARTED, IN_PROGRESS,
        FAILED, or COMPLETE. If any action fails, the plan is considered to have failed.
        The plan is complete when all actions are complete.
        '''
        status = COMPLETE
        for action in self.actions:
            if action.status == FAILED:
                return FAILED
            elif action.status == NOT_STARTED and status == COMPLETE:
                status = NOT_STARTED
            elif action.status == IN_PROGRESS:
                status = IN_PROGRESS
        return status

    def __str__(self):
        s = ""
        for action in self.actions:
            if action.status == NOT_STARTED:
                s += NS_COLOR_CODE
            elif action.status == IN_PROGRESS:
                s += IP_COLOR_CODE
            elif action.status == FAILED:
                s += F_COLOR_CODE
            elif action.status == COMPLETE:
                s += C_COLOR_CODE

            s += str(action) + " "
        return s[:-1] + END_COLOR_CODE


class AsynchAction:
    nextID = 0

    def __init__(self, mem, midcaAction, executeFunc, isComplete, blocks):
        self.status = NOT_STARTED
        self.mem = mem
        self.midcaAction = midcaAction
        self.executeFunc = executeFunc
        self.isComplete = isComplete
        self.blocks = blocks
        self.startTime = None
        self.id = AsynchAction.nextID
        AsynchAction.nextID += 1

    def execute(self):
        if not self.startTime:
            self.startTime = midcatime.now()
        self.status = IN_PROGRESS
        if not self.executeFunc:
            return
        try:
            self.executeFunc(self.mem, self.midcaAction, self.status)
        except:
            if verbose >= 2:
                print "Error executing action", self, ":\n", traceback.format_exc(),
                "\n\nAction assumed to be failed"
            self.status = FAILED

    def check_complete(self):
        if not self.startTime:
            self.startTime = midcatime.now()
        if not self.check_complete:
            return
        try:
            complete = self.isComplete(self.mem, self.midcaAction, self.status)
            if verbose >= 2 and not complete:
                print "Action", self, "not complete."
            if verbose >= 1 and complete:
                print "Action", self, "complete."
            if complete:
                self.status = COMPLETE
            return complete
        except:
            if verbose >= 1:
                print "Error checking completion status for action", self, " - Assuming \
                 failure:\n", traceback.format_exc()
            self.status = FAILED

    def ros_msg(self, topic, d):
        '''
        arg d should be a dictionary that contains the key/value pairs to be sent.
        '''
        sent = rosrun.send_msg(topic, rosrun.dict_as_msg)
        if not sent:
            if verbose >= 1:
                print "Unable to send msg; ", d, "on topic", topic, " Action", self,
                "assumed failed."
            self.status = FAILED

    def __str__(self):
        return str(self.midcaAction)


class GraceMidcaAct():
    def __init__(self):
        gracePath = '/home/pi/Desktop/Grace_Control'
        self.bottomDepth = 0
        self.gracePath = gracePath
        import sys
        sys.path.insert(0, gracePath)
        import GliderFunIPC
        self.interface = GliderFunIPC.GraceFun()

    def communicateDepth(self, depth):  # returns boolean telling if depth was sent out over xbee
        grace = self.interface
        return grace.sendXbeeMsg("Depth:" + str(depth))

    def dive(self):  # fills robot tank with water to make it sink
        grace = self.interface
        # grace.movePump(4)
        grace.moveMass(4)

    def rise(self):  # expells water from robot to make it float
        grace = self.interface
        # grace.movePump(98)
        grace.moveMass(98)

    def senseDepth(self):  # reads the pressure sensor and converts it to depth in meters
        grace = self.interface
        return grace.readDepth()

    def checkCommunicationAck(self):  # read a file output by program chechinkg for surface and return true or false
        # sending $%GO%$ over xbee will cause a file "Next_Dive_GO" to be produce with 1 in line one
        Acknowleged = False
        gracePath = self.gracePath + "/"
        try:
            f = open(gracePath + "Next_Dive_GO", 'r')
            Acknowleged = (1 == int(f.readline()))
            f.close()
            if Acknowleged:
                f = open(gracePath + "Next_Dive_GO", 'w')
                f.write("0")
                f.close()
        except:
            return False
        return Acknowleged


class GraceSense(AsynchAction):
    '''
    Grace action that senses depth
    '''

    def __init__(self, mem, midcaAction):
        #self.GraceAct = GraceMidcaAct()
        self.mem = mem
        self.action = midcaAction
        self.time = None
        self.skip = True
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
                              completionCheck, True)

    def implement_action(self):
        self.time = midcatime.now()
        pass

    def check_confirmation(self):
        if self.time:
            if (midcatime.now() - self.time) >= 10:
                if self.action.args[1] == "veryshallow":
                    self.mem.set(self.mem.ANOMALY, "slow")
                return True
        return False

class GraceClean(AsynchAction):
    '''
    Grace action that cleans itself
    '''

    def __init__(self, mem, midcaAction):
        #self.GraceAct = GraceMidcaAct()
        self.mem = mem
        self.action = midcaAction
        self.time = None
        self.skip = True
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
                              completionCheck, True)

    def implement_action(self):
        self.time = midcatime.now()
        pass

    def check_confirmation(self):
        if self.time:
            if (midcatime.now() - self.time) >= 10:
                return True
        return False


class GraceCommunicate(AsynchAction):
    '''
    Action that communicates it's depth to fumin
    '''

    def __init__(self, mem, midcaAction):
        #self.GraceAct = GraceMidcaAct()
        self.mem = mem
        self.time = None
        self.skip = True
        self.depth = None
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
                              completionCheck, True)

    def implement_action(self):
        self.time = midcatime.now()
        pass

    def check_confirmation(self):
        if self.time:
            if (midcatime.now() - self.time) >= 10:
                return True
        return False


class GraceRaise(AsynchAction):
    '''
    Action to make grace reach surface
    '''

    def __init__(self, mem, midcaAction):
        #self.GraceAct = GraceMidcaAct()
        self.mem = mem
        self.time = None
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
                              completionCheck, True)

    def implement_action(self):
        self.time = midcatime.now()
        pass

    def check_confirmation(self):
        if self.time:
            if (midcatime.now() - self.time) >= 10:
                return True
        return False


class GraceDive(AsynchAction):
    '''
    Makes grace dive
    '''

    def __init__(self, mem, midcaAction):
        #self.GraceAct = GraceMidcaAct()
        self.mem = mem
        self.time = None
        self.complete = False
        executeAction = lambda mem, midcaAction, status: self.implement_action()
        completionCheck = lambda mem, midcaAction, status: self.check_confirmation()
        AsynchAction.__init__(self, mem, midcaAction, executeAction,
                              completionCheck, True)

    def implement_action(self):
        self.time = midcatime.now()
        pass

    def check_confirmation(self):
        if self.time:
            if (midcatime.now() - self.time) >= 10:
                return True
        return False
