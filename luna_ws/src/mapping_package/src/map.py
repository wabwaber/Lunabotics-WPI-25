import math
import numpy
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, PointStamped
from std_msgs.msg import Float32MultiArray, Bool

class Map_Node:
    # Constants
    GRID_CELL_SIZE_M = 0.08

    MIN_OCCURANCES = 10

    CELL_PADDING = 5

    MAP_WIDTH = 5.3

    MAP_HEIGHT = 3.8

    # Initialize occupancy grid message
    map_msg = OccupancyGrid()
    rviz_map_msg = OccupancyGrid()

    width = int(MAP_WIDTH / GRID_CELL_SIZE_M) # 86
    height = int(MAP_HEIGHT / GRID_CELL_SIZE_M) # 62

    # initialize grid with -1 (unknown)
    obstacle_grid = numpy.ndarray((height, width), buffer=numpy.zeros((width, height), dtype=numpy.int),dtype=numpy.int)
    driveable_grid = numpy.ndarray((height, width), buffer=numpy.zeros((width, height), dtype=numpy.int),dtype=numpy.int)

    # TF Broadcaster
    tf_broadcaster = tf.TransformBroadcaster()
    
    def __init__(self):
        # initialize node
        rospy.init_node('map')

        self.obstacle_grid.fill(int(0))
        self.driveable_grid.fill(int(0))

        # fill map_msg with the parameters
        self.map_msg.info.resolution = self.GRID_CELL_SIZE_M
        self.map_msg.info.width = self.width
        self.map_msg.info.height = self.height
        self.map_msg.data = [0] * (self.width * self.height)
        self.map_msg.header.frame_id = 'map'
        # set map origin [meters]
        self.map_msg.info.origin.position.x = 0.0
        self.map_msg.info.origin.position.y = 0.0

        # fill rviz_map_msg with the parameters
        self.rviz_map_msg.info.resolution = self.GRID_CELL_SIZE_M
        self.rviz_map_msg.info.width = self.width
        self.rviz_map_msg.info.height = self.height
        self.rviz_map_msg.data = [0] * (self.width * self.height)
        self.rviz_map_msg.header.frame_id = 'map'
        # set rviz_map origin [meters]
        self.rviz_map_msg.info.origin.position.x = 0.0
        self.rviz_map_msg.info.origin.position.y = 0.0

        # Publishers
        self.occ_pub = rospy.Publisher("/robot/map", OccupancyGrid, queue_size = 10)
        self.rviz_pub = rospy.Publisher("/robot/rviz_map", OccupancyGrid, queue_size = 10)
        self.change_pub = rospy.Publisher("robot/map_change", Bool, queue_size = 3)

        # Subscribers
        self.obstacle_sub = rospy.Subscriber('/map/obstacle', Float32MultiArray, self.set_obstacle_cb)
        self.click_sub = rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_cb)

        for i in range(self.width):
            self.pad_cell(0,i)
            self.pad_cell(self.height-1, i)

        for i in range(self.height):
            self.pad_cell(i, 0)
            self.pad_cell(i, self.width-1)

    # callback functions
            
    def set_obstacle_cb(self, obstacle_msg):
        data = obstacle_msg.data
        x = data[0]
        y = data[1]
        radius = data[2]

        grid_x, grid_y = self.world_to_grid(x, y)
        self.set_obstacle(x=grid_x, y=grid_y, radius=radius)

    def clicked_point_cb(self,click_msg):
        x = click_msg.point.x
        y = click_msg.point.y
        grid_x, grid_y = self.world_to_grid(x, y)

        self.obstacle_grid[grid_x][grid_y] = self.MIN_OCCURANCES + 1
        self.pad_cell(grid_x, grid_y)

    # takes grid cell dimensions
    def pad_cell(self, x, y):
        
        new_obstacle = False
        # Loop through the cells within the obstacle radius around the specified coordinate
        for i in range(x - self.CELL_PADDING, x + self.CELL_PADDING + 1):
            for j in range(y - self.CELL_PADDING, y + self.CELL_PADDING + 1):

                # Calculate the distance between the current cell and the center (x, y)
                distance = math.sqrt((i - x)**2 + (j - y)**2)

                # Check if the current cell is within the radius of the circle
                if distance <= self.CELL_PADDING:

                    # Check if the current cell is within the grid boundaries
                    if 0 <= i < len(self.driveable_grid) and 0 <= j < len(self.driveable_grid[0]):

                        # If the cell is within the circle, set its value to 10
                        if(self.driveable_grid[i][j] < 10):
                            new_obstacle = True
                            self.driveable_grid[i][j] = 10

        self.change_pub.publish(new_obstacle)


    def set_obstacle(self, x, y, radius): # size in m
        
        rad = math.floor(radius // self.GRID_CELL_SIZE_M)

        #this is a flag to tell if the drivable space is changed
        new_obstacle = False
        
        # Loop through the cells within the obstacle radius around the specified coordinate
        for i in range(x - rad, x + rad + 1):
            for j in range(y - rad, y + rad + 1):

                # Calculate the distance between the current cell and the center (x, y)
                distance = math.sqrt((i - x)**2 + (j - y)**2)

                # Check if the current cell is within the radius of the circle
                if distance <= rad:

                    # Check if the current cell is within the grid boundaries
                    if 0 <= i < len(self.obstacle_grid) and 0 <= j < len(self.obstacle_grid[0]):

                        # If the cell is within the circle, set its value to 100
                        if(self.obstacle_grid[i][j] < self.MIN_OCCURANCES):
                            self.obstacle_grid[i][j] += 1
                        
                        elif self.obstacle_grid[i][j] == self.MIN_OCCURANCES:
                            self.pad_cell(i, j)
                            self.obstacle_grid[i][j] += 1
                        
    def forget_potential_obstacles(self):

        for x in range(len(self.obstacle_grid)):
            for y in range(len(self.obstacle_grid[0])):
                if(0 < self.obstacle_grid[x][y] < self.MIN_OCCURANCES):
                    self.obstacle_grid[x][y] -= 1
        
    def world_to_grid(self, x, y):

        # Grid cell indices for (x, y) point
        grid_x = int(x / self.GRID_CELL_SIZE_M)
        grid_y = int(y / self.GRID_CELL_SIZE_M)
        return (grid_y, grid_x)
    
    def publish_map(self):
        
        # stamp current ros time to the message
        self.map_msg.header.stamp = rospy.Time.now()
        self.rviz_map_msg.header.stamp = rospy.Time.now()
        # build ros map message and publish
        for i in range(self.width*self.height):
            self.map_msg.data[i] = self.driveable_grid.flat[i]
            self.rviz_map_msg.data[i] = int(self.driveable_grid.flat[i] - 100 * (self.obstacle_grid.flat[i] / self.MIN_OCCURANCES))

        for i in range(self.width*self.height):
            if self.rviz_map_msg.data[i] == 0:
                self.rviz_map_msg.data[i] = 99

        self.occ_pub.publish(self.map_msg)
        self.rviz_pub.publish(self.rviz_map_msg)

        # Publish TF transform between "map" and "world" frame
        self.tf_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            rospy.Time.now(),
            "map",
            "world"
        )
        
# main function
if __name__ == '__main__':
    # Map update rate (defaulted to 5 Hz)
    rate = 5.0

    mapNode = Map_Node()
    loop_rate = rospy.Rate(rate)

    iterator = 0

    while not rospy.is_shutdown():
        loop_rate.sleep()
        mapNode.publish_map()

        iterator += 1

        if iterator > 8:
            mapNode.forget_potential_obstacles()
            iterator = 0
