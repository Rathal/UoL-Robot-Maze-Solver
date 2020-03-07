import rospy
from std_msgs.msg import Int16

class StateManager:
    def __init__(self):
        rospy.init_node('state_manager')
        self.state_sub = rospy.Subscriber('/state', Int16, self.stateCB)
        self.state_pub = rospy.Publisher('/state', Int16, queue_size=1)
        
        self.state = None
        self.start = False
    

    def stateCB(self, state):
        self.state = state
        print state
    
    def boot(self):
        print 'Starting'
        if (self.start == False):
            self.start = True
            print 'Initialising State: 0'
            self.state_pub.publish(0)


stateManager = StateManager()
stateManager.boot()
rospy.spin()