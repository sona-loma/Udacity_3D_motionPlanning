import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local




# _________________________________________________________________________________________
class States(Enum):                                                 #Here you have to put the phases that you are gonna use
    MANUAL = auto()                                                 #Enum is an easier way to list and it has many benifts like auto() etc..
    ARMING = auto()                                                 #auto() = set random number
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()
# _________________________________________________________________________________________




# _________________________________________________________________________________________
class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)                                                                  #connect the dorne with the autopilot

        self.target_position = np.array([0.0, 0.0, 0.0])                                              #/setting some informatiom to the function like:
        self.waypoints = []                                                                           #like "my targest position" & "mission" is True (because it has connected)
        self.in_mission = True                                                                        # as well as a list of waypoints and check_state variable
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL                                                             #set the phase right now as "MANUAL"

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)                    #if you received this go tp that
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
# _________________________________________________________________________________________





# _________________________________________________________________________________________
#  here, i will write the calculations of some cases that happen during the takeoff:
    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:                                                      # here, we will write the calculations that should happen if we have a specifice phase

# when to stop from the takeoff
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:                       # check if altitude is within 95% of target
                self.waypoint_transition()                                                           # go to the waypoints

# if you are already in the desired altitude (local_pos=target_pos):
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()

# if we are in the last target position (check the velocity):
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:                               # if you finished all the targets points?
                        self.landing_transition()                                                    #transition to the landing

# _________________________________________________________________________________________






# _________________________________________________________________________________________
    def velocity_callback(self):                                                                     # we need velocity calculations if we have LANDING
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:                                  # make sure its landed by subtract the altitudes
                if abs(self.local_position[2]) < 0.01:                                               # double check
                    self.disarming_transition()
# ________________________________________________________________________________________






# _________________________________________________________________________________________
    def state_callback(self):                                    #/we are writting the state_callback just in case we run the code in the middle of the flight
        if self.in_mission:                                      #it could know from where to start + writting the sequences is important
            if self.flight_state == States.MANUAL:               #*since we sat it as MANUAL; it will start from "arming_trans()" function
                self.arming_transition()
            elif self.flight_state == States.ARMING:             #if the phase is arming?
                if self.armed:                                   #make sure the rotors are running
                    self.plan_path()                             #do plan_path() function
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()
# _________________________________________________________________________________________







# _________________________________________________________________________________________
    def arming_transition(self):                                        # Now, it is starting
        self.flight_state = States.ARMING                               # we write this line to check if there is a calculation in this phase
        print("arming transition")
        self.arm()                                                      #run the arms
        self.take_control()                                             #make it autopilot (in the screen

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])                           #use this if you want go up directly

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)                    # telling that the target that i want is that self.waypoint values
        print('target position', self.target_position)                  # print the target
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])   #command for that targest

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()                                                   #make it MANUAL in the screen

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        import csv
        with open('colliders.csv', newline='') as cfile:
            csv_reader = csv.reader(cfile)
            row_one = next(csv_reader)

        # TODO: set home position to (lon0, lat0, 0)
        latitude = row_one[0].strip('lat0')
        longitude = row_one[1].strip(' lon0')
        # convert string to float
        latitude_f = float(latitude)
        longitude_f = float(longitude)
        print(latitude_f)
        print(longitude_f)
        self.set_home_position(longitude_f, latitude_f, 0.0)
        print(self.global_home)

        # TODO: retrieve current global position
        global_position = (self._longitude, self._latitude, self._altitude)
        print("global position is:", global_position)

        # TODO: convert to current local position using global_to_local()
        from global_to_local import global_to_local
        local_position = global_to_local(global_position, self.global_home)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))



        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(local_position[0]-north_offset), int(local_position[1]-east_offset))     #add int()
        # Set goal as some arbitrary position on the grid
        #grid_goal = (-north_offset - 10, -east_offset - 10)



        # TODO: adapt to set goal as latitude / longitude position and convert
        global_grid_goal = [37.795205, -122.397205, 0.0]
        grid_goal = global_to_local(global_grid_goal, self.global_home)
        grid_goal = (int(grid_goal[0]-north_offset), int(grid_goal[1]-east_offset))
        print('Local Start and Goal: ', grid_start, grid_goal)
        # Run A* to find a path from start to goal



        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)



        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()









# _________________________________________________________________________________________
    def start(self):                                                      #Start and End commands
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()
# _________________________________________________________________________________________







# _________________________________________________________________________________________
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=600)
    drone = MotionPlanning(conn)                                                          #Here, we put our connection to the drone we used
    time.sleep(1)

    drone.start()
# _________________________________________________________________________________________
