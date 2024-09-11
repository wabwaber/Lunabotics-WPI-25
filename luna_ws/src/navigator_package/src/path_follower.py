from math import atan2, cos, sqrt, sin
import rospy
from std_msgs.msg import Float32, Int32, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PathFollower:
    #state variables
    current_path:Path
    curr_pos = (0,0)
    curr_heading = 0
    following = False
    next_pos = (0,0)
    next_index = 1
    index = 0
    goal_pos = (0,0)
    prev_icc = 0

    
    # set up outputs
    state = Int32()
    speed = Float32()

    #constants
    DRIVE_STRAIGHT = .015 #max delta heading to drive staright
    ICC_TURN = 1 # max delta heading to use ICC turn, makes sure robot point turns if goal is behind it
    ICC_SCALE_FACTOR = .9 #makes turns tighter than necessary to avoid runaway oscilation
    NORMAL_LOOK_AHEAD = .25
    FINAL_LOOK_AHEAD = .0625
    FINAL_PATH_LEN = 2
    DRIVE_SPEED = .75
    TURN_SPEED = .375
    HALF_DT_WIDTH = .5334 / 2
    ICC_HYST = .1 # 10cm
    



    #init
    def __init__(self):
        # subscribers
        self.path_sub = rospy.Subscriber('/jetson/nav_path', Path, self.path_cb)
        self.fused_pose_sub = rospy.Subscriber('/jetson/filtered_pose', PoseStamped, self.fused_pose_cb)
        self.e_stop_sub = rospy.Subscriber('/robot/e_stop', Bool, self.e_stop_cb)

        # publishers
        self.state_pub = rospy.Publisher('/drivetrain/state', Int32, queue_size = 10)
        self.speed_pub = rospy.Publisher('/drivetrain/drive', Float32, queue_size = 10)
        self.icc_pub = rospy.Publisher('/drivetrain/icc', Float32, queue_size = 10)
        self.target_pub = rospy.Publisher('/target', PoseStamped, queue_size = 10)



    # callback functions
    
    # disables path following TODO: Implement this with KBL
    def e_stop_cb(self, message:Bool):
        if(message.data):
            self.following = False
            self.state.data = 0
            self.state_pub.publish(self.state)
        else:
            self.following = True


    # Receives a path and starts following it
    def path_cb(self, path):
        self.current_path = path
        self.following = True
        self.index = len(path.poses) - 1
        self.extract_next_pos()
        # final_pos = path.poses[0]
        # self.goal_pos = (final_pos.pose.position.x, final_pos.pose.position.y)

    # Updates the pose of the robot
    def fused_pose_cb(self, pose_stamped):
        #extract x and y pos of robot
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        self.curr_pos = (x,y)

        #extract heading of robot
        q = pose_stamped.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.curr_heading = yaw


    # External functions
        
    def is_following(self):
        return self.following

    # The command to follow the path, needs to be continuously called
    def follow(self):
        if not self.following:
            return
        
        #dist_2_goal = self.euchlidean_distance(self.curr_pos, )


        #check euclidean distance
        dist = self.euchlidean_distance(self.curr_pos, self.next_pos)
        
        if dist < self.NORMAL_LOOK_AHEAD and self.index > self.FINAL_PATH_LEN:
            self.extract_next_pos()
            return
        elif dist < self.FINAL_LOOK_AHEAD:
            self.extract_next_pos()
            return
        
        #find the necessary heading to get to the next point
        dx = self.next_pos[0] - self.curr_pos[0]
        dy = self.next_pos[1] - self.curr_pos[1]

        target_heading = atan2(dy, dx)

        #find the needed heading change
        delta_heading = target_heading - self.curr_heading

        # constrain delta_heading to -pi < th < pi
        if delta_heading > 3.141592653589793:
            delta_heading -= 6.28318530718

        if delta_heading < -3.141592653589793:
            delta_heading += 6.28318530718


        # ask Ian for this proof
        r_icc = dist / (2*sin(delta_heading))
        r_icc *= self.ICC_SCALE_FACTOR

        #drive straight condition
        if(abs(delta_heading) < self.DRIVE_STRAIGHT):
            self.state.data = 1
            self.state_pub.publish(self.state)

            self.speed.data = self.DRIVE_SPEED
            self.speed_pub.publish(self.speed)

        #icc turning condition
        elif abs(delta_heading) < self.ICC_TURN and ((self.state.data == 3 and abs(r_icc) > self.HALF_DT_WIDTH + .05) or abs(r_icc) > self.HALF_DT_WIDTH + self.ICC_HYST):

            self.state.data = 3
            self.state_pub.publish(self.state)

            icc = Float32()
            icc.data = r_icc
            self.icc_pub.publish(icc)
            self.prev_icc = r_icc

            self.speed.data = self.DRIVE_SPEED
            self.speed_pub.publish(self.speed)

        #point turning condition
        #negative
        elif(delta_heading < 0):
            self.state.data = 2
            self.state_pub.publish(self.state)

            self.speed.data = -self.TURN_SPEED
            self.speed_pub.publish(self.speed)
        #positive
        else:
            self.state.data = 2
            self.state_pub.publish(self.state)

            self.speed.data = self.TURN_SPEED
            self.speed_pub.publish(self.speed)

    # helper functions
    def euchlidean_distance(self, pos_1, pos_2):
        dist = sqrt((pos_1[0] - pos_2[0])**2 + (pos_1[1] - pos_2[1])**2)
        return dist
    
    # sets next_pose to be the next pose in the path, also iterates index and checks if we are at end of path
    def extract_next_pos(self):
        #increment
        self.index -= 1

        #if we are at the end of the path, stop following and disable DT
        if self.index < 0:
            print("path done")
            self.following = False
            state = Int32()
            state.data = 1
            self.state_pub.publish(state)
            self.speed.data = 0
            self.speed_pub.publish(self.speed)
            return

        #extract next pose
        pose_stamped = self.current_path.poses[self.index]
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        self.next_pos = (x,y)

        pose_stamped.header.frame_id = "world"

        self.target_pub.publish(pose_stamped)




# main
if __name__ == '__main__':
    rospy.init_node("path_follower")
    follower = PathFollower()

    while not rospy.is_shutdown():
        if(follower.is_following()):
            follower.follow()
            rospy.sleep(.1)
        else:
            rospy.sleep(.125)



