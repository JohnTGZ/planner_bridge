import numpy as np
from time import time  # https://realpython.com/python-time-module/
from functools import partial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from crazyswarm_application.msg import AgentState, AgentsStateFeedback, UserCommand
from .modules.map import map

def get_euclid_dist(point_a, point_b):
    """Get the 3D euclidean distance between 2 geometry_msgs.msg.Point messages

    Parameters
    ----------
    point_a : geometry_msgs.msg.Point
        Origin Point
    point_b : geometry_msgs.msg.Point
        Goal Point

    Returns
    -------
    float
        Distance between point a and b
    """
    dx = point_a.x - point_b.x
    dy = point_a.y - point_b.y
    dz = point_a.z - point_b.z

    return np.sqrt(dx * dx + dy * dy + dz * dz)

class Agent:
    def __init__(self, agent_id=""):
        """_summary_

        Parameters
        ----------
        agent_id : String
            ID of agent

        Raises
        ------
        Exception
            Exception raised when empty agent_id is given to constructor
        """
        if agent_id == "":
            raise Exception("Created an Agent object without an ID!")
        
        self.id = agent_id
        self.executing_waypoint = False  # True if agent is in the process of executing waypoint else False.
        self.current_waypoint_cmd = None  # Current waypoint being executed (of type UserCommand)
        self.flight_state = None  # Current state of agent
        self.connected = None  
        self.completed = None  
        self.mission_capable = None  
        self.actual_pos = None  # Current actual position
        self.waypoint_queue = [] # List of waypoints to fulfill
        self.time_since_pose_update = time()  # Time elapsed since robot pose was last updated

    def add_waypoints(self, waypoints):
        """Add a list of waypoints to the agent

        Parameters
        ----------
        waypoints : List of geometry_msgs.msg.PoseStamped
            A list of waypoints of PoseStamped message type
        """
        self.waypoint_queue.extend(waypoints)

    def get_dist_to_goal(self) -> float :
        """Get distance from agent's current position to goal

        Returns
        -------
        float
            Distance to goal in meters
        """
        return get_euclid_dist(self.actual_pos.pose.position, self.current_waypoint_cmd.goal)

    def new_waypoint_cmd(self, waypoint_cmd):
        """Set Agent to new waypoint command

        Parameters
        ----------
        waypoint_cmd : UserCommand
            Waypoint command issued to agent
        """
        self.executing_waypoint = True
        self.current_waypoint_cmd = waypoint_cmd

    def complete_waypoint_cmd(self):
        self.executing_waypoint = False
        self.current_waypoint_cmd = None

class planner_ROS(Node):

    def __init__(
        self, policy_net, k_size, device="cpu", greedy=False, save_image=False
    ):
        super().__init__(
            "planner_ROS",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        #####
        # Example waypoints
        #####

        cf_waypoints = {
            "cf1": [
                self.create_posestamped(x=-0.5, y=-0.5, z=1.0, yaw=0.0),
                self.create_posestamped(x=-0.5, y=-1.0, z=1.0, yaw=0.0),
                self.create_posestamped(x=-1.0, y=-1.0, z=1.0, yaw=0.0),
                self.create_posestamped(x=-1.0, y=-0.5, z=1.0, yaw=0.0),
                self.create_posestamped(x=-0.5, y=-0.5, z=1.0, yaw=1.54),
            ],
            "cf2": [
                self.create_posestamped(x=-0.5, y=0.0, z=1.0, yaw=0.0),
                self.create_posestamped(x=-1.0, y=0.0, z=1.0, yaw=0.0),
                self.create_posestamped(x=-1.5, y=0.5, z=1.0, yaw=0.0),
                self.create_posestamped(x=-2.0, y=0.0, z=1.0, yaw=0.0),
                self.create_posestamped(x=-1.5, y=-0.5, z=1.0, yaw=0.0),
                self.create_posestamped(x=-1.0, y=-0.0, z=1.0, yaw=0.0),
                self.create_posestamped(x=-0.5, y=0.0, z=1.0, yaw=0.707),
            ],
            "cf3": [
                self.create_posestamped(x=-0.5, y=0.5, z=1.0, yaw=0.0),
                self.create_posestamped(x=-0.5, y=1.0, z=1.0, yaw=0.0),
                self.create_posestamped(x=-1.0, y=1.0, z=1.0, yaw=0.0),
                self.create_posestamped(x=-1.0, y=0.5, z=1.0, yaw=0.0),
                self.create_posestamped(x=-0.5, y=0.5, z=1.0, yaw=0.0),
            ],
        }

        #####
        # Example agent list
        #####

        self.agent_list = {
            "cf1": Agent("cf1"),
            "cf2": Agent("cf2"),
            "cf3": Agent("cf3"),
        }

        self.inactive_agent_list = []

        # Add waypoints to each agent
        for item in self.agent_list.items():
            agent_id = item[0]
            agent = item[1]

            agent.add_waypoints(cf_waypoints[agent_id])


        #####
        # Parameters
        #####
        # Commented out as 'automatically_declare_parameters_from_overrides' is enabled in node params
        # self.declare_parameter('agent_timeout', 10.0) # Timeout for agent before it is removed
        # self.declare_parameter('pub_wp_timer_period', 0.25) # Time period for publishing waypoint callback (seconds)
        # self.declare_parameter('check_agent_timer_period', 0.25) # Time period for checking if agent has fulfilled goals
        # self.declare_parameter('goal_tolerance', 0.1) # Distance between goal and actual pose for goal to be fulfilled (meters)

        self.agent_timeout = self.get_parameter('agent_timeout').get_parameter_value().double_value
        self.pub_wp_timer_period = self.get_parameter('pub_wp_timer_period').get_parameter_value().double_value
        self.check_agent_timer_period = self.get_parameter('check_agent_timer_period').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        print(self.check_agent_timer_period)

        #####
        # Create publishers
        #####
        self.usercommand_pub = self.create_publisher(UserCommand, "/user", 10)

        #####
        # Create subscribers
        #####
        self.agent_state_sub = self.create_subscription(
            AgentsStateFeedback, "/agents", self.agent_state_callback, 10
        )

        self.agent_pose_sub = []

        for agent_id in list(self.agent_list.keys()):
            self.agent_pose_sub.append(
                self.create_subscription(
                    PoseStamped,
                    agent_id + "/pose",
                    partial(self.agent_pose_callback, agent_id=agent_id),
                    10,
                )
            )

        #####
        # Create timers
        #####

        self.pub_wp_timer = self.create_timer(
            self.pub_wp_timer_period, self.publish_waypoints
        )

        self.check_agent_timer = self.create_timer(
            self.check_agent_timer_period, self.check_agent_timer_callback
        )

    def agent_pose_callback(self, pose, agent_id):
        """Subscriber callback to save the actual pose of the agent 

        Parameters
        ----------
        pose : geometry_msgs.msg.PoseStamped
            PoseStamped message
        agent_id : String
            Id of Agent
        """
        if agent_id in self.agent_list:
            self.agent_list[agent_id].actual_pos = pose
            self.agent_list[agent_id].time_since_pose_update = time()

    def agent_state_callback(self, agent_states):

        for agent in agent_states.agents:
            if agent.id in self.agent_list:
                self.agent_list[agent.id].flight_state = agent.flight_state
                self.agent_list[agent.id].connected = agent.connected  
                self.agent_list[agent.id].completed = agent.completed  
                self.agent_list[agent.id].mission_capable = agent.mission_capable  
            # IF receiving pose of inactive agent, ignore
            else:
                self.get_logger().error(
                    f"Agent {agent.id} has been removed from agent list. Ignoring..."
                )

    def publish_waypoints(self):

        for item in self.agent_list.items():  # Iterate through list of available agents
            agent_id = item[0]
            agent = item[1]

            # IF agent is still executing current waypoint, then keep publishing the current one
            if agent.executing_waypoint:
                if not agent.current_waypoint_cmd: #This condition should not activate, otherwise there is bug present in the code
                    self.get_logger().info("Agent {agent_id} has no current waypoint to follow!")
                
                self.usercommand_pub.publish(agent.current_waypoint_cmd)

            # IF waypoints queue is not empty, send a new waypoint from the existing waypoint_queue
            elif len(agent.waypoint_queue) != 0:
                new_wp = agent.waypoint_queue[0]

                waypoint_cmd = self.create_usercommand(
                    cmd = "goto_velocity",
                    uav_id = [agent_id],
                    goal = Point(
                        x = new_wp.pose.position.x,
                        y = new_wp.pose.position.y,
                        z = new_wp.pose.position.z,
                    ),
                    yaw = self.quaternion_to_euler(new_wp.pose.orientation)[2],
                    is_external=True,
                )

                # Pop the first waypoint from the queue
                agent.waypoint_queue.pop(0)
                agent.new_waypoint_cmd(waypoint_cmd)

                # Publish waypoint command to the agent
                self.usercommand_pub.publish(waypoint_cmd)

                self.get_logger().info(
                    "Published new waypoint ({},{},{}) with yaw {} for agent {}".format(
                        waypoint_cmd.goal.x,
                        waypoint_cmd.goal.y,
                        waypoint_cmd.goal.z,
                        waypoint_cmd.yaw,
                        agent_id,
                    )
                )

    def check_agent_timer_callback(self):
        """Check if agents: 
        1. Are in active flight state (not landing state). If False, remove from active agent list
        2. have time exceeded since updating last pose state (not landing state). If False, remove from active agent list
        3. Have an actual pose?  If not, send warning
        4. have fulfilled the goal, if so then activate flags for next waypoint.
        """
        time_now = time()
        for item in self.agent_list.items():
            agent_id = item[0]
            agent = item[1]

            # Check 1: If agent is active. If not remove from list 
            # and add to inactive list
            if agent.flight_state not in {
                AgentState.MOVE,
                AgentState.IDLE,
                AgentState.TAKEOFF,
                AgentState.HOVER,
            }:
                self.get_logger().warn(f"'{agent_id}' is in {agent.flight_state} State, and is classified as INACTIVE")
                # Remove from Active agent list
                self.remove_inactive_agent(agent_id)
                continue
            
            # Check 2: Timeout exceeded since last pose received?
            if abs(time_now - agent.time_since_pose_update) > self.agent_timeout:
                self.get_logger().warn(f"Timeout exceeded in updating pose for Agent '{agent_id}'")
                # Remove from list
                self.remove_inactive_agent(agent_id)
                continue

            # Check 3: Does agent have an acual position? If not, give a warning message
            if agent.actual_pos is None:
                self.get_logger().warn(f"No actual position obtained for Agent '{agent_id}'")
                continue

            # Check 4: if agent has fulfilled goal
            # If so, set "executing_waypoint" to FALSE so that agent is ready to accept new waypoints
            if agent.executing_waypoint and agent.current_waypoint_cmd:

                dist_to_goal = agent.get_dist_to_goal()
                
                self.get_logger().info("Agent '{}' has {:.2f}m left to goal".format(agent_id, dist_to_goal), throttle_duration_sec=1.0)

                if dist_to_goal < self.goal_tolerance:
                    self.get_logger().info(f"Agent {agent_id} has fulfilled waypoint, ready to accept next waypoint")
                    agent.complete_waypoint_cmd()

            self.get_logger().info(
                f"""{agent_id} \n
                flight_state: {agent.flight_state}
                actual_pos: {agent.actual_pos}
                time_since_pose_update: {agent.time_since_pose_update}
                connected: {agent.connected}
                completed: {agent.completed}
                mission_capable: {agent.mission_capable}
                """
            )

    def remove_inactive_agent(self, agent_id):
        """Remove inactive agent from active agent list and add to inactive agent list

        Parameters
        ----------
        agent_id : String
            Agent ID
        """
        try:
            self.agent_list.pop(agent_id)
            self.inactive_agent_list.append(agent_id)
            self.get_logger().warn(f"Removed inactive agent '{agent_id}'")

        except ValueError:
            self.get_logger().error(f"Unable to pop '{agent_id}' out of active agent list")

    def takeoff_all(self):
        """Helper method to command all crazyflies to take off
        """
        waypoint = self.create_usercommand(
            cmd="takeoff_all",
            uav_id=[],
        )

        self.usercommand_pub.publish(waypoint)

        self.get_logger().info("Commanded all crazyflies to take off")

    def create_posestamped(self, x, y, z, yaw=0.0):
        """Helper method to create a geometry_msgs.msg.PoseStamped message
        """
        pose = PoseStamped()
        pose.header.frame_id = ""
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position= Point(
            x=x,
            y=y,
            z=z
        )

        pose.pose.orientation = self.quaternion_from_euler(
            0, 0, -np.radians(yaw)
        )

        return pose

    def create_usercommand(self, cmd, uav_id, goal=Point(), yaw=0.0, is_external=False):
        """Helper method to create a UserCommand message, used to send drone commands

        Parameters
        ----------
        cmd : String
            Command type
        uav_id : String
            Crazyflie agent ID
        goal : geometry_msgs.msg.Point()
            (x,y,z) goal
        yaw : float, optional
            yaw, by default 0.0
        is_external : bool, optional
            TODO, by default False

        Returns
        -------
        crazyswarm_application.msg.UserCommand()
            UserCommand message
        """
        usercommand = UserCommand()

        usercommand.cmd = cmd
        usercommand.uav_id = uav_id
        usercommand.goal = goal
        usercommand.yaw = yaw
        usercommand.is_external = is_external

        return usercommand

    def quaternion_to_euler(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]

    def quaternion_from_euler(self, ai, aj, ak):
        """Converts euler values to a ROS Quaternion message

        Parameters
        ----------
        ai : float
            roll (radians)
        aj : float
            pitch (radians)
        ak : float
            yaw (radians)

        Returns
        -------
        geometry_msgs.msg.Quaternion
            quaternion ROS message type
        """
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = np.cos(ai)
        si = np.sin(ai)
        cj = np.cos(aj)
        sj = np.sin(aj)
        ck = np.cos(ak)
        sk = np.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))

        return Quaternion(
            x = cj*sc - sj*cs,
            y = cj*ss + sj*cc,
            z = cj*cs - sj*sc,
            w = cj*cc + sj*ss,
        )

def main():
    rclpy.init(args=None)
    policy_net = None
    k_size = None
    device = None

    planner_ros = planner_ROS(
        policy_net=policy_net, k_size=k_size, device=device, greedy=True
    )

    # All drones takeoff
    # planner_ros.takeoff_all()

    rclpy.spin(planner_ros)  # Keep node alive

    planner_ros.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
