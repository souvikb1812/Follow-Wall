#!/usr/bin/env python

# ROS stuff
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
# other useful math tools
from tf.transformations import euler_from_quaternion
from math import atan2,sqrt,pi

 
# angle and distance constraints
# you can adjust them if necessary
angle_eps = 0.2
dis_eps = 0.01

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

pub = None

# Class that will be used to read and parse /odom topic
class odomReader:

    def __init__(self):

        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.x = None
        self.y = None
        self.theta = None

    # Function that will take care of input message from odom topic
    # This function will be called whenever new message is available to read
    # Subsequently odom topic is parsed to get (x,y,theta) coordinates 
    def newOdom(self, msg):
        # get x and y coordinates
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # convert quaternion to Euler angles
        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

# Class that is responsible to read and parse raw LaserScan data 
class scanReader:

    def __init__(self):
        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/scan", LaserScan, self.newScan)
        # divide laser scan data into 5 regions
        self.region = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
    # Function that will take care of input message from scan topic
    # This function will be called whenever new message is available to read
    # Subsequently scan topic is parsed to get region data: minimum distance to object from every sight 
    def newScan(self, msg):
        self.ranges = msg.ranges
        self.msg = msg
        self.region['left'] = min(self.ranges[60:100])
        self.region['fleft'] = min(self.ranges[20:60])
        self.region['front'] = min(self.ranges[0:20]+self.ranges[-20:])
        self.region['fright'] = min(self.ranges[300:340])
        self.region['right'] = min(self.ranges[260:300])
        
        #print "range[90]: ", msg.ranges[90]

state = 0
# divide robot motion in 3 scenario
state_dict = {
    0: 'go to point',
    1: 'wall detected',
    2: 'follow the wall',
}
# define initial scenario
state = 0


def main():
    global pub

    # initialize ROS node
    rospy.init_node("follow_wall")
    # run stop function when this node is killed
    rospy.on_shutdown(stop)
    rospy.sleep(0.5)

    # define the control velocity publisher of topic type Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    # initialize odom and scan objects
    # Use these objects to access robot position in space and scan information
    odom = odomReader()
    scan = scanReader()
    rospy.sleep(0.5)

    # initialize speed as Twist topic 
    speed = Twist()

    # set the loop frequency
    rate = rospy.Rate(1000)

    # Set the goal point 
    goal = Point()
    goal.x = 0.0
    goal.y = 5.0

    # distance maintained from wall
    d=0.6   

    state = 0 # setting initial state to 0


    while not rospy.is_shutdown():
        # initialize speed as Twist topic
        

        # TODO:

        # Decide what to do for the robot in each of these states:


        if state == 0:
            # go to point state. 
            '''
            Hint: 
                Here robot should go towards a point located behind the wall. When it encounters the wall
                it should change the state to "wall detected".

                should be bunch of if-else statements'''
            # find the x,y distance to the goal from current position
            inc_x=goal.x-odom.x
            inc_y=goal.y-odom.y

            # find the angle of the goal point to the initial point
            # where the robot starts to rotate on the global frame
            angle_to_goal=atan2(inc_y,inc_x)
            if (angle_to_goal<0):
                angle_to_goal+=2*pi

            # find the angle difference between breakthe robot's sensor(odometry)
            # and the one you found above
            angle_diff=(angle_to_goal-odom.theta)
            

            # find the Euclidean distance between the current robot's location
            # and the current destinations
            dist_diff=sqrt((inc_x*inc_x)+(inc_y*inc_y))

            # write an if-else statement which determine whether 
            # angular or linear velocity is applied to the robot
            # and when it reaches a goal point, update the initial point
            # and move on to the next goal point.
            
            if (angle_diff>0.1):

                speed.linear.x=0.0 # stopping linear movement
                speed.angular.z=0.3 # rotating anti-clockwise

            elif (angle_diff<-0.1):

                speed.linear.x=0.0 # stopping linear movement
                speed.angular.z=-0.3 # rotating clockwise
                
            # Move towards wall if distance from wall is more than 0.3
            elif(dist_diff>=0.2 and scan.region['front'] > 0.3):

                speed.linear.x=0.11 # setting linear speed
                speed.angular.z=0.0 # stopping rotation

            # Stop moving towards goal if distance between wall and 
            # turtlebot is less than or equal to 0.3
            elif (dist_diff>=0.2 and scan.region['front'] <= 0.3):

                speed.linear.x = 0.0 # stopping linear movement
                speed.angular.z = 0.0 # stopping rotation
                state=1
                
            
            print ("current state: ", state_dict[state])



        elif state == 1:
            # wall detected state. 
            '''
            Hint: 
                Here robot should turn right/left based on your choice. And, continue following the wall.
                Finally, do not forget to change the state!

                should be bunch of if-else statements

            '''
            # Turn left by rotating the turtlebot anti-clockwise if near wall
            if ((scan.region['front'] <= d and scan.region['fleft'] > d and scan.region['fright'] > d)or
            (scan.region['front'] <= d and scan.region['fleft'] > d and scan.region['fright'] <= d and scan.region['right'] <= d)or
            (scan.region['front'] <= d and scan.region['fleft'] <= d and scan.region['fright'] > d and scan.region['right'] > d) or
            (scan.region['front'] <= d and scan.region['fleft'] <= d and scan.region['fright'] <= d and scan.region['right'] <= d)):

                speed.angular.z=0.3 # rotating anti-clockwise
                state=2

            else:
                state=2

            print ("current state: ", state_dict[state])


        elif state==2:
            # wall following state. 
            '''
            Hint: 
                Here robot should go around the obstacle infinetily. Depending on the laserscan readings
                decide what to do for the robot.

                should be bunch of if-else statements
            '''
            # If wall is to the right move along the wall
            if (scan.region['front'] > d and scan.region['fleft'] > d and scan.region['fright'] <= d and scan.region['right'] <= d):
                
                speed.linear.x=0.11 # setting linear speed
                speed.angular.z=0.0 # stopping rotation

            # If the wall is too far to the right then move ahead while turning right by rotating clockwise
            elif ((scan.region['front'] > d and scan.region['fleft'] > d and scan.region['fright'] > d and scan.region['right'] > d)or
            (scan.region['front'] > d and scan.region['fleft'] <= d and scan.region['fright'] > d and scan.region['right'] > d)):
                
                speed.linear.x=0.11 # setting linear speed
                speed.angular.z=-0.3 # rotating clockwise
                state=1

            else:
                state=1


            print ("current state: ", state_dict[state])


        print (scan.region)
        pub.publish(speed)
        rate.sleep()

# call this function when you press CTRL+C to stop the robot
def stop():
    global pub
    speed = Twist()
    speed.linear.x = 0.0
    speed.angular.z = 0.0

    pub.publish(speed)

if __name__ == '__main__':
    main()
