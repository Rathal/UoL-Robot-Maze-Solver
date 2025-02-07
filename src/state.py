import rospy
from std_msgs.msg import String
from time import sleep

class StateManager:
    def __init__(self):
        rospy.init_node('state_manager')
        self.state_sub = rospy.Subscriber('/state', String, self.stateCB)
        self.state_pub = rospy.Publisher('/state', String, queue_size=1)
        
        self.state = "Null"
        self.start = False
        self.list = []
        
        self.isSpinning = False
    

    def stateCB(self, state):
        string = state.data
        print 'Incoming: ', string
        if string.startswith('req_'):
            print 'Request'
            self.determineRequest(string)
        else: self.state = string

        if self.state == "Start Spin":
            self.isSpinning = True
        if self.isSpinning and self.state == "Waiting":
            self.state_pub.publish(String(data="Update Spin"))

        # self.list.append(self.state)
        # print 'List:'
        # for x in self.list:
        #     print(x)
        
    def determineRequest(self, request):
        approved = False
        if self.state == 'Initialised' or self.state == 'Waiting':
            approved = True
        elif "Obstacle" in self.state or self.state == "Red":
            if "Turn To" in request:
                if "90" in request or "180" in request or "270" in request or "360" in request or "0" in request:
                    approved = True      
        if (approved):
            print 'Request Approved'
            sleep(0.1)
            self.state_pub.publish(request.lstrip('req_'))
        else: print 'Request Denied'
    
    def boot(self):
        print 'Starting'
        if (self.start == False):
            self.start = True
            print 'Initialising State: "Initialised"'
            state = String()
            state.data = "Initialised"
            self.state_pub.publish(state)
            print "Published state: ", self.state


stateManager = StateManager()
sleep(1)
stateManager.boot()
rospy.spin()