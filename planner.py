from mapUtilities import *
from a_star import *
from probabilistic_road_map import *
import time

POINT_PLANNER=0; TRAJECTORY_PLANNER=1; ASTAR_PLANNER=2; PRM_PLANNER=3

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)

        self.costMap=None
        self.initTrajectoryPlanner()
        
        return self.trajectory_planner(startPose, endPose, self.type)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):
        
        #### If using the map, you can leverage on the code below originally implemented for A*
        self.m_utilities=mapManipulator(laser_sig=0.4)    
        self.costMap=self.m_utilities.make_likelihood_field()

        # List of obstacles to plot (in x and y coordiantes)
        self.obstaclesList = np.array(self.m_utilities.getAllObstacles())     

        # List of obstacles for PRM, in cell indices
        self.obstaclesListCell = np.array(self.m_utilities.getAllObstaclesCell())  

        
    def trajectory_planner(self, startPoseCart, endPoseCart, type):

        # Create function for calculating total cost using Euclidean distance
        def calculate_total_path_cost(path):
            total_cost=0
            for i in range(len(path)-1):
                x1,y1=path[i]
                x2,y2=path[i+1]
                total_cost+=sqrt((x2-x1)**2+(y2-y1)**2)
            return total_cost
        

        startPose=self.m_utilities.position_2_cell(startPoseCart)
        endPose=self.m_utilities.position_2_cell(endPoseCart)

        # [Part 3] TODO Use the PRM and search_PRM to generate the path
        # Hint: see the example of the ASTAR case below, there is no scaling factor for PRM
        if type == PRM_PLANNER:

            # Define the robot's radius (Obtained from: https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)
            buffer_size=0.095
            # Add an extra buffer size because points are being placed too close to objects 
            # even with robot radius being accounted for
            robot_radius = 0.105 + buffer_size

            # Generate PRM graph using probabilistic roadmap
            sample_points, prm_roadmap = prm_graph(
                start=startPose,
                goal=endPose,
                obstacles_list=self.obstaclesListCell,
                robot_radius=robot_radius,
                rng=None,
                m_utilities=self.m_utilities
            )

            # Search for a path in the PRM using search_PRM
            start_time = time.time()
            path_indices = search_PRM(sample_points, prm_roadmap, startPose, endPose)

            # Convert path indices to positions
            sample_points_map = {tuple(point): i for i, point in enumerate(sample_points)}
            path_ = [sample_points[sample_points_map[idx]] for idx in path_indices]

            end_time = time.time()
            print(f"The time took for PRM pathfinding was {end_time - start_time}")

            # Calculate total cost
            Path=np.array(list(map(self.m_utilities.cell_2_position, path_)))
            total_cost = calculate_total_path_cost(Path)
            print(f"Total path cost (PRM + A*): {total_cost}")

        elif type == ASTAR_PLANNER: # This is the same planner you should have implemented for Lab4
            scale_factor = 4 # Depending on resolution, this can be smaller or larger
            startPose = [int(i/scale_factor) for i in startPose]
            endPose   = [int(j/scale_factor) for j in endPose]
            start_time = time.time()

            path = search(self.costMap, startPose, endPose, scale_factor)

            end_time = time.time()


            print(f"the time took for a_star calculation was {end_time - start_time}")

            path_ = [[x*scale_factor, y*scale_factor] for x,y in path ]

            # Calculate Total Cost
            Path = np.array(list(map(self.m_utilities.cell_2_position, path_)))
            total_cost = calculate_total_path_cost(Path)
            print(f"Total path cost (A*): {total_cost}")

        Path = np.array(list(map(self.m_utilities.cell_2_position, path_ )))

        # Save the generated path to a CSV file to plot expected vs actual
        np.savetxt("generated_path.csv", Path, delimiter=",")
        print("Generated path saved to 'generated_path.csv'.")

        # Plot the generated path
        plt.plot(self.obstaclesList[:,0], self.obstaclesList[:,1], '.')
        plt.plot(Path[:,0], Path[:,1], '-*')
        plt.plot(startPoseCart[0],startPoseCart[1],'*')
        plt.plot(endPoseCart[0],
                 endPoseCart[1], '*')
        

        # Add title, label, legend, grid
        plt.title("A* Trajectory Path") #use this for just A*
        plt.xlabel("X Coordinate [m]")
        plt.ylabel("Y Coordinate [m]")
        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()
        
        return Path.tolist()
    

if __name__=="__main__":

    m_utilities=mapManipulator()
    
    map_likelihood=m_utilities.make_likelihood_field()
    
    # You can test your code here...
