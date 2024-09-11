from math import sqrt
import rospy
from queue import PriorityQueue
from std_msgs.msg import Float32, Int32, Bool
from nav_msgs.msg import OccupancyGrid, Path, GridCells
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped


class PathFinder:
    # state variables
    goal = (0,0)
    occupancy_grid:OccupancyGrid = OccupancyGrid()
    current_pose = (1,1)
    ready_for_path = False

    def __init__(self):
        #subscribers
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.nav_goal_cb)
        self.map_sub = rospy.Subscriber('/robot/map', OccupancyGrid, self.new_map_cb)
        self.fused_pose_sub = rospy.Subscriber('/jetson/filtered_pose', PoseStamped, self.fused_pose_cb)
        self.clicked_point_sub = rospy.Subscriber('/robot/map_change', Bool, self.new_obstacle_cb)

        #publishers
        self.path_pub = rospy.Publisher('/jetson/nav_path', Path, queue_size = 1)
        # self.cells_pub = rospy.Publisher('/a_star_debug', GridCells, queue_size = 10)

        # prevents a div 0 error if map isn't updated in time
        self.occupancy_grid.info.resolution = 1
    

    # callback functions
    def nav_goal_cb(self, nav_goal:PoseStamped):
        point = nav_goal.pose.position
        resolution = self.occupancy_grid.info.resolution

        self.goal = (int(point.x / resolution), int(point.y / resolution))

        #sets the generate path flag to true, will generate when a new map is available
        self.ready_for_path = True
    
    def new_obstacle_cb(self, message:Bool):
        
        # sets the generate path flag to true, will generate when a new map is available
        if(message.data):
            self.ready_for_path = True

    def new_map_cb(self, grid:OccupancyGrid):
        # update internal map
        self.occupancy_grid = grid

        #Check if a path needs to be generated, and generates one
        if self.ready_for_path:
            if self.is_drivable(self.goal):
                print("drivable")
                path = self.a_star()
                if path is None:
                    print("no path")
                    path = Path()

            else:
                print("not drivable")
                path = Path()
            self.path_pub.publish(path)
            
            self.ready_for_path = False

    
    def fused_pose_cb(self, pose:PoseStamped):
        # Updates internal pose data
        point = pose.pose.position
        resolution = self.occupancy_grid.info.resolution

        self.current_pose = (int(point.x / resolution), int(point.y / resolution))
        
    #helper functions

    #is_drivable takes a tuple representing x and y cell numbers in the occupancy grid
    def is_drivable(self, postion):
        if postion[0] < 0 or postion[1] < 0 or postion[0] >= self.occupancy_grid.info.width or postion[1] >= self.occupancy_grid.info.height:
            return False
        
        grid = self.occupancy_grid.data
        index = postion[1] * self.occupancy_grid.info.width + postion[0]

        if grid[index] < 10:
            return True
        return False
    
    #neighbors_of_8 will return all adjacent drivable cells, unless position is undrivable, in which case it will return all adjacent cells on the map
    def neighbors_of_8(self, position):
        x = position[0]
        y = position[1]
        neighbors = []
        in_drivable = self.is_drivable(position)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                new_pos = (x+dx, y+dy, dx, dy)
                if (dx,dy) != (0,0):
                    #if nt currently in drivable space
                    if self.is_drivable(new_pos) or not in_drivable:
                        neighbors.append(new_pos)
        
        return neighbors
    
    # Euchlid_dist returns the eeuchlidean distance between 2 points
    def euchlid_dist(self, start, end):
        return sqrt((start[0] - end[0]) **2 + (start[1] - end[1])**2)

    # Plans a pathe from self.current_pose to self.goal_pose
    def a_star(self):

        # gridCells stuff
        # gc = GridCells()
        # gc.cell_width = .08
        # gc.cell_height = .08
        # gc.header.frame_id = "world"
        
        frontier = PriorityQueue()

        frontier.put((0, self.current_pose))

        came_from = {}
        cost_to = {}
        dir_to = {}


        came_from[self.current_pose] = None
        cost_to[self.current_pose] = 0
        dir_to[self.current_pose] = (0,0)

        while not frontier.empty():
            current = frontier.get()[1]

            if current == self.goal:
                break

            current_dir = dir_to[current]



            for next in self.neighbors_of_8(current):
                next_pos = (next[0], next[1])
                next_dir = (next[2], next[3])

                # gridCells stuff
                # point = Point()
                # point.x = next_pos[0] * .08
                # point.y = next_pos[1] * .08
                # gc.cells.append(point)
                # self.cells_pub.publish(gc)

                # calculate cost to reach this cell
                new_cost = cost_to[current] + self.euchlid_dist(current, next_pos)

                # punish paths that turn
                if(next_dir != current_dir):
                    new_cost += .5
                
                
                if next_pos not in cost_to or new_cost < cost_to[next_pos]:
                    #add/update a cell in frontier
                    cost_to[next_pos] = new_cost
                    dir_to[next_pos] = next_dir
                    
                    # If cell is drivable, use distance to goal as herustic
                    if self.is_drivable(next_pos):
                        heuristic = self.euchlid_dist(next_pos, self.goal)

                    # If cell is non-drivable, make herustic really big and constant so the algorithm does dijkstras until it escapes
                    else:
                        heuristic = 1000
                    
                    
                    priority = new_cost + heuristic
                    

                    frontier.put((priority, next_pos))
                    came_from[next_pos] = current
        
        # reconstruct the path back
        current = self.goal
        path = Path()
        path.header.frame_id = "world"
        
        try:
            while current != self.current_pose:
                # convert cell location to pose in world frame
                pose = PoseStamped()

                resolution = self.occupancy_grid.info.resolution

                pose.pose.position.x = current[0] * resolution + resolution / 2
                pose.pose.position.y = current[1] * resolution + resolution / 2

                path.poses.append(pose)
                current = came_from[current]
            
            return path
        except KeyError: # if  the path fails reconstruction, I.E. there is no path
            return None


# main
if __name__ == '__main__':
    rospy.init_node("pathfinder")
    PathFinder()
    rospy.spin()

