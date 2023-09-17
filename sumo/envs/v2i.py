import os
import sys
import pathlib
import base64
import logging
import traci as tc
import numpy as np
from lxml import etree
import gymnasium as gym
from gymnasium.spaces.box import Box
from gymnasium.spaces import Sequence
from scipy.spatial import distance
from collections import namedtuple
from gymnasium import spaces
from typing import Any, Union, Literal
from sumo.xmls.defaultXMLs import ROUTE_XML, NET_XML

Vehicle = namedtuple("Vehicle", ["id",
                                 "x",
                                 "y",
                                 "speed"])

class V2I(gym.Env):
    metadata = {"render_modes": ["human"]}
    
    def __init__(self,
                 view_size: int=20,
                 max_nearby_vehicles:int=6,
                 render_mode=None):
        

        #self.observation_space = spaces.Box()
        #self.action_space = spaces.Box()

        if view_size < 0:
            raise ValueError("View Size must be greater than zero.")

        self._view_size = view_size

        # For logging purposes
        self._logger = logging.getLogger(__name__)
        self._logger.setLevel(logging.DEBUG)
        logging.basicConfig(filename='output.log', level=logging.INFO)
        
        self._logger.info("Config: View Size: {}. Max nearby vehicles to consider: {}.".format(view_size,
                                                                                               max_nearby_vehicles))
        # Try to fetch the module path to build the sumo path
        basePath = pathlib.Path(__file__)
        binsPath = os.path.abspath(os.path.join(basePath.parent, "bins"))
        
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

        # Time step duration
        self._t_step = 0.1

        # Max time steps in episode
        self._max_time_steps = 3e3

        # Max number of vehicles to consider near EGO vehicle
        self._max_nearby_vehicles = max_nearby_vehicles

        # Current time steps
        self._current_t_steps = 0

        # Common vehicle config
        self._vehicle_config = {
            'maxSpeed': 50,               # 180 Km/hr
            'maxAccel': 2,                # m/s2
            'maxDecel': -2,               # m/s2
            'length': 5,                  # m
            'minGap': 1,                  # MinGap between vehicles im metre
        }

        # Define action space
        self.action_space = Box(low=self._vehicle_config['maxDecel'],
                                high=self._vehicle_config['maxAccel'],
                                dtype=np.float32)
        
        # Define observation space
        obs_box = Box(low=np.array([-float('inf')] * max_nearby_vehicles * 3),
                      high=np.array([float('inf')] * max_nearby_vehicles * 3),
                      dtype=np.float32)
        self.observation_space = obs_box
        """
        # Simulation binary arguments
        args = self._build_sim_args(sumoConfig=str(basePath.parent) + "/xmls/v2v.sumocfg",
                                    t_step=self._t_step)
        
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        mode = "gui" if self.render_mode == "human" else None
        print(args)
        tc.start([self._get_binary_abs_path(binsPath, mode=mode)] 
                 + args)
        if mode == "gui":
            self._logger.debug("Running in GUI-mode.")
        else:
            self._logger.debug("Running in non GUI-mode(simulation mode).")
        """

    def _build_sim_args(self,
                        sumoConfig,
                        t_step):
        return ["-c", sumoConfig,
                "--step-length", str(t_step),
                "--collision.mingap-factor", "0",
                "--collision.action", "warn",
                "--xml-validation",  "never",
                "--time-to-teleport", "-1"]
    
    def _build_VType_node(self,
                          id,
                          length,
                          maxSpeed,
                          maxAccel,
                          maxDecel,
                          minGap,
                          color):
        args = locals()
        vType = etree.Element("vType")
        for attrb in args:
            if attrb != "self":
                vType.set(attrb, str(args[attrb]))
        return vType

    def _build_Vehicle_node(self,
                            id,
                            depart,
                            type,
                            route):
        args = locals()
        vehicle = etree.Element('vehicle')
        for attrb in args:
            if attrb != 'self':
                vehicle.set(attrb, str(args[attrb]))
        return vehicle

    def step(self, action: float):

        assert type(float(action)) == float, "Expected action to be: float, Got: %s"%(type(action))
        
        self._current_t_steps += 1
        assert self._current_t_steps <= self._max_time_steps
        
        done = False
        if action < self.action_space.low[0] or action > self.action_space.high[0]:
            raise ValueError("Invalid action: {}, Expected [{}, {}].".format( 
                             action,
                             self.action_space.low[0],
                             self.action_space.high[0]))

        # Get current speed of Ego Vehicle
        current_speed = tc.vehicle.getSpeed("ego")

        # Get the new speed of the Ego Vehicle
        new_speed = current_speed + action * self._t_step
        new_speed = np.clip(new_speed, a_min=0.0, a_max=self._vehicle_config['maxSpeed'])

        # Set the new speed of Ego Vehicle
        tc.vehicle.setSpeed('ego', new_speed)

        # Perform a step in simulation
        tc.simulationStep()

        # Check collided vehicles
        collided_vehicles = tc.simulation.getCollidingVehiclesIDList()
        if "ego" in collided_vehicles:
            done = True
            self._logger.warn("Ego vehicle has collided")
            reward = -300
            terminal_state = np.array([-1, -1, -1] * self._max_nearby_vehicles).flatten()
            return terminal_state, reward, done, False, {}
        else:
            if len(collided_vehicles) > 0:
                self._logger.warn("Vehicles other ego has collided.")
        
        # Detect End of episode
        veh_ids = list(tc.vehicle.getIDList())
        if "ego" not in veh_ids:
            done = True
        
        if tc.simulation.getMinExpectedNumber() <= 0:
            done = True
        
        truncated = True if self._current_t_steps >= self._max_time_steps else False
        if not done:
            # Reward function
            reward = tc.vehicle.getSpeed("ego")/tc.vehicle.getMaxSpeed("ego")
            obs, nearby_vehicles = self._get_obs(tc)
            info = {"num_vehicles_nearby": nearby_vehicles}
            
            # observation, reward, terminated, truncated, info     
            return obs, reward, done, truncated, info     
        else:
            # Terminal State
            reward = 0
            terminal_state = np.array([-1, -1, -1] * self._max_nearby_vehicles).flatten()
            return terminal_state, reward, done, truncated, {}


    def _get_binary_abs_path(self, 
                             basePath: str, 
                             mode:Union[None, Literal["mode"]]=None):
        basePath = str(basePath)
        if mode is not None and mode == "gui":
            binPath = basePath + "/" + "sumo-gui"
        else:
            binPath = basePath + "/" + "sumo"
        if os.path.exists(binPath):
            return binPath
        else:
            raise RuntimeWarning("SUMO bins not found at: {}".format(str(basePath)))
    
    def _get_obs(self, tc):
        nearby_vehs = self._get_ego_nearby_vehicles(tc, self._view_size)
        ego_veh = nearby_vehs['ego']
        dist_dict = {}
        for veh in nearby_vehs.values():
            dist = distance.euclidean((ego_veh.x, ego_veh.y), (veh.x, veh.y))
            dist_dict[veh.id] = dist
        dist_dict = sorted(dist_dict.items(), key=lambda x:x[1])
        
        obs = []

        for idx, (vid, dist) in enumerate(dist_dict):
            v = nearby_vehs[vid]
            obs.append( (v.x, v.y, v.speed))
            if (idx + 1) >= self._max_nearby_vehicles:
                break
        remaining_obs = self._max_nearby_vehicles - len(obs)
        if remaining_obs > 0:
            obs = obs + [(-1, -1, -1)] * remaining_obs
        obs = np.array(obs, dtype=np.float32).flatten()
        return obs, len(nearby_vehs)

    def _get_ego_nearby_vehicles(self, tc, viewSize: int):
        veh_ids = list(tc.vehicle.getIDList())
        assert 'ego' in veh_ids
        #veh_ids.remove('ego')
        veh_positions = {}
        ref_pos_ego = tc.vehicle.getPosition("ego")
        for veh_id in veh_ids:
            pos = tc.vehicle.getPosition(veh_id)
            dist = distance.euclidean(ref_pos_ego, pos)
            if dist <= viewSize:
                trans_pos = pos[0] - ref_pos_ego[0], pos[1] - ref_pos_ego[1]
                
                v = Vehicle(id=veh_id,
                            x=trans_pos[0],
                            y=trans_pos[1],
                            speed=tc.vehicle.getSpeed(veh_id))

                veh_positions[veh_id] = v

        return veh_positions

    
    def __del__(self):
        """Clean up simulation before destroying the object.
        """
        if tc.isLoaded():
            tc.close()
        sys.stdout.flush()
    
    def render(self):
        """render is called during training. 
        Keep render empty for now.
        """
        pass
    
    def close(self):
        """Closes the TraCI connection, if opened.
        """
        if tc.isLoaded():
            tc.close()
        self._logger.debug("Stopped SUMO TraCI server.")

    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)

        # Loads XMLs
        basePath = pathlib.Path(__file__).parent.parent
        xmlPath = os.path.abspath(os.path.join(basePath, "xmls"))
        
        # Route file
        routeRoot = etree.fromstring(base64.b64decode(ROUTE_XML))
        
        num_vehicles = 50
        ego_veh_id = num_vehicles//2
        
        # Define vehicle Types        
        # Ego vehicle
        vType = self._build_VType_node(id="ego_vType",
                                       length=self._vehicle_config['length'],
                                       maxSpeed=self._vehicle_config['maxSpeed'],
                                       maxAccel=self._vehicle_config['maxAccel'],
                                       maxDecel=self._vehicle_config['maxDecel'],
                                       minGap=self._vehicle_config['minGap'],
                                       color="red")
                                           
        routeRoot.append(vType)
        
        # Non-Ego vehicle
        vType = self._build_VType_node(id="non_ego_vType",
                                       length=self._vehicle_config['length'],
                                       maxSpeed=self._vehicle_config['maxSpeed'],
                                       maxAccel=self._vehicle_config['maxAccel'],
                                       maxDecel=self._vehicle_config['maxDecel'],
                                       minGap=self._vehicle_config['minGap'],
                                       color="yellow")
                                           
        routeRoot.append(vType)

        # Populate vehicles in the scene
        switch = True
        for veh in range(num_vehicles):

            if veh == ego_veh_id:
                route = self.np_random.choice(["r_0", "r_1"])
                v = self._build_Vehicle_node(id="ego",
                                             type="ego_vType",
                                             depart="0.0",
                                             route=route)
            else:
                route = "r_0" if switch else "r_1"
                v = self._build_Vehicle_node(id="car_" + str(veh),
                                             type="non_ego_vType",
                                             depart="0.0",
                                             route=route)
                switch = not switch

            routeRoot.append(v)                                 

        etree.indent(routeRoot)
        
        with open(os.path.abspath(os.path.join(xmlPath, "v2v.rou.xml")), "wb") as f:
            data = etree.tostring(routeRoot, encoding="utf-8", xml_declaration=True, pretty_print=True)
            f.write(data)
        
        # Load Net XML
        netRoot = etree.fromstring(base64.b64decode(NET_XML))

        # Write modified Net XML
        with open(os.path.abspath(os.path.join(basePath, "xmls/v2v.net.xml")), "wb") as f:
            data = etree.tostring(netRoot, encoding="utf-8", xml_declaration=True, pretty_print=True)
            f.write(data)

        # Edit sumo config
        sumoConfigPath = os.path.abspath(os.path.join(basePath, "xmls/v2v.sumocfg"))
        
        with open(sumoConfigPath, "rb") as f:
            sumocfgRoot = etree.fromstring(f.read())
        
        for item in sumocfgRoot:
            for tags in item:
                if tags.tag == "net-file":
                    tags.set("value", "v2v.net.xml")
                elif tags.tag == "route-files":
                    tags.set("value", "v2v.rou.xml")
        
        with open(sumoConfigPath, "wb") as f:
            data = etree.tostring(sumocfgRoot, encoding="utf-8", xml_declaration=True, pretty_print=True)
            f.write(data)
        

        # Reload with modified config
        binsPath = os.path.abspath(os.path.join(basePath, "bins"))
        mode = "gui" if self.render_mode == "human" else None
        args = self._build_sim_args(sumoConfig=str(basePath) + "/xmls/v2v.sumocfg",
                                    t_step=self._t_step)
        binary = None
        if mode == "gui":
            binary = "sumo-gui"
            self._logger.debug("Starting in GUI-mode.")
        else:
            binary = "sumo"
            self._logger.debug("Starting in non GUI-mode(simulation mode).")
        
        if not tc.isLoaded():
            #args = [self._get_binary_abs_path(binsPath, mode=mode)] + args
            #print(args)
            tc.start([binary] + args)
            self._logger.debug("Started SUMO TraCI server.")
        else:
            tc.load(args)
            self._logger.debug("Reloaded SUMO TraCI server.")
        
        # Simulate until Ego Vehicle appers
        done = False
        veh_ids = None
        num_steps = 0
        while tc.simulation.getMinExpectedNumber() > 0 or not done:
            tc.simulationStep()
            veh_ids = tc.vehicle.getIDList()
            num_steps += 1
            if "ego" in veh_ids:
                break
        
        # Disable all checks for ego vehicle
        # Allows to control ego vehicle externally
        # More info: https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#speed_mode_0xb3
        tc.vehicle.setSpeedMode("ego", 32)

        # Reset step counter
        self._current_t_steps = 0

        # Returns the list of vehicles and count
        obs, num_nearby_veh = self._get_obs(tc)
        return obs, {"num_vehicles_nearby": num_nearby_veh}

