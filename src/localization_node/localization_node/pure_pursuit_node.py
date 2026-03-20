import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib.animation as animation
from IPython import display


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        ## Variables for Pure Pursuit

        ##are you using rotation?? IDEK
        self.using_rotation = False

        ##how often the pure pursuit loop should run
        self.hertz = 0

        ## the path, it will be delivered/
        self.path_to_follow = []




        ################    Publsihers TBD   ################

        self.motor_controller_publisher = self.create_publisher(
            
        )

        ################    Subscribers      ################

        #
        self.path_subscriber = self.create_subscription(
            ###TBD
        )

        self.current_position_subscriber = self.create_subscription(

        )



    
    
    #################################################### CURRENT POSE ####################################################
    def position_callback():
        pass

    #################################################### PATH BUILDER ####################################################

    type Point = tuple[float,float]

    ## appends the points to the path_to_follow array
    ## this should be updated later
    def path_builder(self, point: Point) -> None:
        self.path_to_follow.append(point)
        return


    #################################################### PURE PURSUIT ####################################################



    ## helper function
    def pt_to_pt_distance (pt1,pt2):
        distance = np.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
        return distance

    def sgn (num):
        if num >= 0:
            return 1
        else:
            return -1    
    
    ## actually implement one step of pure pursuit
    ## path: Array of a vector of points (ex: [[0,1], [0,2]....])
    ## currentPosition: a 2 dimensional vecotr (x,y)
    ## currentHeading: a 
    ## lookAheadDistance: number that determines distance in meters for the circle to look ahead
    ## lastFoundIndex_input: determines the last index to ensure that the robot is not going background
    def pure_pursuit_step(self, path, currentPosition, currentHeading, lookAheadDistance, lastFoundIndex_input):
        
        #Extract current X and Y positions
        currentX = currentPosition[0]
        currentY = currentPosition[1]

        #Find last foudn index
        lastFoundIndex = lastFoundIndex_input

        ##loop flag
        intersectFound = False
        
        ## Index of the first point
        startingIndex = lastFoundIndex

        for i in range (startingIndex, len(path)-1):
            x1 = path[i][0] - currentX
            y1 = path[i][1] - currentY
            x2 = path[i+1][0] - currentX
            y2 = path[i+1][1] - currentY

            dx = x2-x1
            dy = y2-y1
            dr = math.sqrt(dx**2 + dy**2)
            D = x1*y2 - x2*y1
            discriminant = (lookAheadDistance**2) * (dr**2) - D**2

            if discriminant >= 0:
                ## line intersection code
                intersection_x1 = (D * dy + self.sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
                intersection_x2 = (D * dy - self.sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
                intersection_y1 = (- D * dx + abs(dy) * np.sqrt(discriminant)) / dr**2
                intersection_y2 = (-D * dx - abs(dy) * np.sqrt(discriminant)) / dr **2

                intersection_1 = [intersection_x1 + currentX, intersection_y1 + currentY]
                intersection_2 = [intersection_x2 + currentX, intersection_y2 + currentY]

                ## calculate the viable range of values

                minX = min(path[i][0], path[i+1][0])
                minY = min(path[i][1],path[i+1][1])
                maxX = max(path[i][0], path[i+1][0])
                maxY = max(path[i][1],path[i+1][1])

                if ((minX <= intersection_1[0] <= maxX) and (minY <= intersection_1[1] <= maxY)) or ((minX <= intersection_2[0] <= maxX) and (minY <= intersection_2[1] <= maxY)):
                    ##break flag
                    foundIntersection = True

                    if ((minX <= intersection_1[0] <= maxX) and (minY <= intersection_1[1] <= maxY)) and ((minX <= intersection_2[0] <= maxX) and (minY <= intersection_2[1] <= maxY)):
                        # compare the points and make a decision if they are both in range
                        if self.pt_to_pt_distance(intersection_1, path[i+1]) < self.pt_to_pt_distance(intersection_2, path[i+1]):
                            goalPoint = intersection_1
                        else:
                            goalPoint = intersection_2
                    else:
                        # if solution pt1 is in range, set that as goal point
                        if (minX <= intersection_1[0] <= maxX) and (minY <= intersection_1[1] <= maxY):
                            goalPoint = intersection_1
                        else:
                            goalPoint = intersection_2
                    
                    if self.pt_to_pt_distance(goalPoint, path[i+1]) < self.pt_to_pt_distance([currentX, currentY], path[i+1]):
                        # update lastFoundIndex and exit
                        lastFoundIndex = i
                        break
                    else:
                        # in case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
                        lastFoundIndex = i+1
                else:
                    foundIntersection = False
                    goalPoint = [path[lastFoundIndex][0], path[lastFoundIndex][1]]
        # obtained goal point, now compute turn vel
        # initialize proportional controller constant
        Kp = 3

        # calculate absTargetAngle with the atan2 function
        absTargetAngle = math.atan2 (goalPoint[1]-currentPosition[1], goalPoint[0]-currentPosition[0]) *180/pi
        if absTargetAngle < 0: absTargetAngle += 360

        # compute turn error by finding the minimum angle
        turnError = absTargetAngle - currentHeading
        if turnError > 180 or turnError < -180 :
            turnError = -1 * self.sgn(turnError) * (360 - abs(turnError))
        
        # apply proportional controller
        turnVel = Kp*turnError
        
        return goalPoint, lastFoundIndex, turnVel