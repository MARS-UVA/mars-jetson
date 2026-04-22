import rclpy
from typing import Tuple
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.time import Time
from rclpy.duration import Duration
from teleop_msgs.msg import SetMotor, MotorChanges
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger



#TODO:
#subscribe to position and append to path
#send controls
#start and stop
#timer callback to run pure pursuit step update


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        ## Variables for Pure Pursuit

        ##are you using rotation?? IDEK
        self.using_rotation = False

        ##how often the pure pursuit loop should run
        self.hertz = 30
        self.look_ahead_distance = 0.6 #meters
        self.velocity = 180
        self.stop_motor_pwm = 127
        # Max differential to apply for turning (motor units, 0..255).
        # Keeping this modest avoids saturating to max speed during turns.
        self.max_turn_delta = 60
        # When on the last segment (last_found_index targets the final waypoint), stop inside this radius of path[-1].
        self.goal_arrival_distance_m = 0.3

        self.recording_path = False
        

        ## the path, it will be delivered/
        self.path_to_follow = []
        self.path_pub = self.create_publisher(Path, "/pure_pursuit/path", 10)
        self.lookahead_pub = self.create_publisher(PoseStamped, "/pure_pursuit/lookahead", 10)
        #lookahead and path are for testing
        self.path_msg = Path()
        self.path_frame = "odom"
        self.last_received_time = None
        self.time_between_pose = 0.25
        self.current_position = None
        self.current_heading = None
        self.last_found_index = 0
        #move to be called when pure pursuit shoudl be played
        #runs main pure pursuit loop
        self.timer = None


        ################    Publsihers TBD   ################
        self.motor_controller_publisher = self.create_publisher(
            MotorChanges,
            "traversal_autonomy",
            10
        )
        self.state_publisher = self.create_publisher(
            UInt8,
            "robot_state/toggle",
            10
        )


        ################    Subscribers      ################

        #/odometry/filtered
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            "/zed/zed_node/pose",
            self.position_callback,
            10
        )


        ###############     Services        ################
        self.start_pathbuild_srv = self.create_service(
            Trigger,
            'start_pathbuild',
            self.start_pathbuild_callback
        )
        self.stop_pathbuild_srv = self.create_service(
            Trigger,
            'stop_pathbuild',
            self.stop_pathbuild_callback
        )

        self.start_purepursuit_srv = self.create_service(
            Trigger,
            'start_purepursuit',
            self.start_purepursuit_callback
        )
        self.stop_purepursuit_srv = self.create_service(
            Trigger,
            'stop_purepursuit',
            self.stop_purepursuit_callback
        )

    #main loop to run pure pursuit
    def timer_callback(self):
        if self.current_position is None or self.current_heading is None:
            return
        if len(self.path_to_follow) < 2:
            return

        r = self.pure_pursuit_step(
            self.current_position,
            self.current_heading,
            self.look_ahead_distance,
            self.last_found_index,
        )
        if r is None:
            self._deactivate_pure_pursuit()
            return
        goalPoint, lastFoundIndex, turnErrorDeg = r
        self.last_found_index = lastFoundIndex
#publish goal point for testing
        lookahead_msg = PoseStamped()
        lookahead_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_msg.header.frame_id = self.path_frame
        lookahead_msg.pose.position.x = float(goalPoint[0])
        lookahead_msg.pose.position.y = float(goalPoint[1])
        lookahead_msg.pose.position.z = 0.0
        lookahead_msg.pose.orientation.w = 1.0
        self.lookahead_pub.publish(lookahead_msg)

        # Map angular error (deg) -> motor differential (uint8 units).
        # Clamp error so we don't saturate the motors for large transient errors.
        turnErrorDeg = max(-90.0, min(90.0, float(turnErrorDeg)))
        turn_delta = int((turnErrorDeg / 90.0) * self.max_turn_delta)

        left_wheel_speeds = int(self.velocity) - turn_delta
        right_wheel_speeds = int(self.velocity) + turn_delta

        # teleop_msgs/SetMotor.velocity is uint8 (0..255)
        left_wheel_speeds = max(0, min(255, int(left_wheel_speeds)))
        right_wheel_speeds = max(0, min(255, int(right_wheel_speeds)))
        motors_msg = MotorChanges(
            changes = [SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=left_wheel_speeds),
                                 SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=left_wheel_speeds),
                                 SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=right_wheel_speeds),
                                 SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=right_wheel_speeds)]
        )
        self.motor_controller_publisher.publish(motors_msg)
        if self.pt_to_pt_distance(self.current_position, self.path_to_follow[-1]) < self.goal_arrival_distance_m:
            self._deactivate_pure_pursuit()

    
    #################################################### CURRENT POSE ####################################################
    #add pose with path_builder if enough time has passed
    def position_callback(self, msg: PoseStamped):
        if msg.header.frame_id:
            self.path_frame = msg.header.frame_id
        current_time = msg.header.stamp.sec
        #if currently recording path and it has been long enough since last position recording
        if self.recording_path and (self.last_received_time is None or current_time-self.last_received_time > self.time_between_pose):                
            self.last_received_time = current_time
            x=msg.pose.position.x
            y=msg.pose.position.y
            self.path_builder((x,y))
        self.current_position = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        # Use degrees to match pure_pursuit_step() math below.
        self.current_heading = math.degrees(math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        ))
        return
        


    #################################################### SERVICE CALLBACKS ####################################################

    def start_pathbuild_callback(self, request, response):
        self.recording_path = True
        response.success = True
        response.message = "Path building started"
        self.path_to_follow = []        #reset path
        return response

    def stop_pathbuild_callback(self, request, response):
        self.recording_path = False
        response.success = True
        response.message = "Path building stopped with " + str(len(self.path_to_follow))+ " nodes"
        return response
    
    def start_purepursuit_callback(self, request, response):
        #start the timer callback to repeatedly call pure pursuit step
        self.recording_path = False
        self.last_found_index = 0
        self.timer = self.create_timer(1.0 / self.hertz, self.timer_callback)
        response.success = True
        response.message = "Pure pursuit started"
        return response

    def _deactivate_pure_pursuit(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        # Send one-shot neutral command so restarting doesn't "reuse" old outputs.
        stop_msg = MotorChanges(
            changes=[
                SetMotor(index=SetMotor.FRONT_LEFT_DRIVE_MOTOR, velocity=self.stop_motor_pwm),
                SetMotor(index=SetMotor.BACK_LEFT_DRIVE_MOTOR, velocity=self.stop_motor_pwm),
                SetMotor(index=SetMotor.FRONT_RIGHT_DRIVE_MOTOR, velocity=self.stop_motor_pwm),
                SetMotor(index=SetMotor.BACK_RIGHT_DRIVE_MOTOR, velocity=self.stop_motor_pwm),
            ]
        )
        self.motor_controller_publisher.publish(stop_msg)

        msg = UInt8()
        msg.data = 0
        self.state_publisher.publish(msg)

    def stop_purepursuit_callback(self, request, response):
        self._deactivate_pure_pursuit()
        response.success = True
        response.message = "Pure pursuit stopped"
        return response


    #################################################### PATH BUILDER ####################################################

    Point = Tuple[float, float]

    ## appends the points to the path_to_follow array
    ## this should be updated later
    def path_builder(self, point: Point) -> None:
        self.path_to_follow.append(point)
        ps = PoseStamped()
        ps.header.frame_id = self.path_frame
        ps.pose.position.x = float(point[0])
        ps.pose.position.y = float(point[1])
        ps.pose.orientation.w = 1.0

        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.header.frame_id = self.path_frame
        self.path_msg.poses.append(ps)
        self.path_pub.publish(self.path_msg)
        return


    #################################################### PURE PURSUIT ####################################################



    @staticmethod
    def pt_to_pt_distance(pt1, pt2):
        return math.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1])

    @staticmethod
    def sgn(num):
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
    def pure_pursuit_step(self, currentPosition, currentHeading, lookAheadDistance, lastFoundIndex_input):
        path = self.path_to_follow
        #Extract current X and Y positions
        currentX = currentPosition[0]
        currentY = currentPosition[1]

        #Find last foudn index
        lastFoundIndex = lastFoundIndex_input

        goalPoint = None

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
                sqrt_disc = math.sqrt(discriminant)
                intersection_x1 = (D * dy + self.sgn(dy) * dx * sqrt_disc) / dr**2
                intersection_x2 = (D * dy - self.sgn(dy) * dx * sqrt_disc) / dr**2
                intersection_y1 = (- D * dx + abs(dy) * sqrt_disc) / dr**2
                intersection_y2 = (-D * dx - abs(dy) * sqrt_disc) / dr **2

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
        if goalPoint is None:
            return None
        # obtained goal point, now compute turn vel
        # initialize proportional controller constant
        Kp = 3

        # calculate absTargetAngle with the atan2 function
        absTargetAngle = math.atan2(goalPoint[1]-currentPosition[1], goalPoint[0]-currentPosition[0]) * 180 / math.pi
        if absTargetAngle < 0: absTargetAngle += 360

        # compute turn error by finding the minimum angle
        turnError = absTargetAngle - currentHeading
        if turnError > 180 or turnError < -180 :
            turnError = -1 * self.sgn(turnError) * (360 - abs(turnError))
        
        # apply proportional controller (deg -> arbitrary turn signal)
        turnVel = Kp*turnError
        
        return goalPoint, lastFoundIndex, turnVel
    
def main():
    rclpy.init()
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()