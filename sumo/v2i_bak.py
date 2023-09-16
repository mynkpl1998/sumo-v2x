import os
import sys
import traci
import optparse
from lxml import etree
import base64
from collections import namedtuple
import numpy as np

class SumoV2I:

    def __init__(self):
        pass

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

def exeExists(binary):
    if os.name == "nt" and binary[-4:] != ".exe":
        binary += ".exe"
    return os.path.exists(binary)

def control_ego_vehicle(traci, lane_change, timestep):
    lane_change_low = -1
    lane_change_high = 1
    lane_change = np.clip(lane_change, lane_change_low, lane_change_high)
    max_acc = 2
    min_acc = -1
    acc = np.random.uniform(min_acc, max_acc)
    """
    lane change < -0.5 -> route r_0
    lane change > 0.5 -> route r_1
    lane change >= -0.5 and <= 0.5 -> no change in route
    """
    first = True
    try:
        veh_ids = traci.vehicle.getIDList()
        if "ego" in veh_ids:

            # Disable all checks for the vehicle, allows manual control from outside.
            if first:
                first = False
                traci.vehicle.subscribeContext("ego", traci.constants.CMD_GET_VEHICLE_VARIABLE, 10, [traci.constants.VAR_DISTANCE])
                #traci.vehicle.addSubscriptionFilterLateralDistance(20)
            
            #traci.vehicle.setSpeedMode("ego", 0)
            

            # Fetch its current route_id
            route_id = traci.vehicle.getRouteID("ego")
            
            # Fetch its current edge 
            edge = traci.vehicle.getRoadID("ego")
            
            # Get current speed
            speed = traci.vehicle.getSpeed("ego")
            new_speed = speed + acc * timestep

            traci.vehicle.setSpeed("ego", new_speed)
            if new_speed < 0:
                new_speed = 0
            if new_speed > 50:
                new_speed = 50
            
            #print(traci.simulation.getCollidingVehiclesIDList())
            traci.vehicle.setSpeed("ego", new_speed)
            #print(new_speed, traci.vehicle.getSpeed("ego"))
            #new_speed
            if edge in ["E0", "E1", "E2", "E9"]:
                if lane_change > 0.5:
                    #traci.vehicle.setRouteID("ego", "r_1")
                    pass
                elif lane_change < -0.5:
                    #traci.vehicle.setRouteID("ego", "r_0")
                    pass
                else:
                    pass
            
    except Exception as e:
        print(e)

def process_nearby_veh(nearby_veh, traci):
    locations = {}
    for veh_id in nearby_veh:
        locations[veh_id] = traci.vehicle.getPosition(veh_id)
    transformed_coordinates = {}
    reference_corrd = locations["ego"]
    for veh_id in locations:
        if veh_id != "ego":
            new_x = locations[veh_id][0] - reference_corrd[0]
            new_y = locations[veh_id][1] - reference_corrd[1]
            transformed_coordinates[veh_id] = (new_x, new_y)
    transformed_coordinates["ego"] = (0,0)
    



def run(t_step):
    """execute the TraCI control loop"""
    step = 0
    # we start with phase 2 where EW has green
    done = False
    while traci.simulation.getMinExpectedNumber() > 0:
    #while not done:
        print("Step: ", step)
        control_ego_vehicle(traci, np.random.uniform(-1, 1), t_step)
        traci.simulationStep()
        collided_vehicles = traci.simulation.getCollidingVehiclesIDList()
        veh_ids = traci.vehicle.getIDList()
        if "ego" in veh_ids:
            subResults = traci.vehicle.getContextSubscriptionResults("ego")
            nearbyVehicles = [id for id in list(subResults.keys())]
            if len(nearbyVehicles) > 0:
                process_nearby_veh(nearbyVehicles, traci)
            
        if "ego" in collided_vehicles:
            break
        step += 1
    print("Steps: ", step)
    traci.close()
    sys.stdout.flush()

def checkBinary(name, bindir=None):
    """
    Checks for the given binary in the places, defined by the environment
    variables SUMO_HOME and <NAME>_BINARY.
    """
    if name == "sumo-gui":
        envName = "GUISIM_BINARY"
    else:
        envName = name.upper() + "_BINARY"
    env = os.environ
    join = os.path.join
    if envName in env and exeExists(env.get(envName)):
        return env.get(envName)
    if bindir is not None:
        binary = join(bindir, name)
        if exeExists(binary):
            return binary
    if "SUMO_HOME" in env:
        binary = join(env.get("SUMO_HOME"), "bin", name)
        if exeExists(binary):
            return binary
    if bindir is None:
        binary = os.path.abspath(join(os.path.dirname(__file__), '..', '..', 'bin', name))
        if exeExists(binary):
            return binary
    if name[-1] != "D" and name[-5:] != "D.exe":
        binaryD = (name[:-4] if name[-4:] == ".exe" else name) + "D"
        found = checkBinary(binaryD, bindir)
        if found != binaryD:
            return found
    return name

def configure_tls(net_file):
    doc = "PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0iVVRGLTgiPz4KCjwhLS0gZ2VuZXJhdGVkIG9uIDIwMjMtMDktMDkgMDA6MDg6NTYgYnkgRWNsaXBzZSBTVU1PIG5ldGVkaXQgVmVyc2lvbiAxLjE4LjAKPGNvbmZpZ3VyYXRpb24geG1sbnM6eHNpPSJodHRwOi8vd3d3LnczLm9yZy8yMDAxL1hNTFNjaGVtYS1pbnN0YW5jZSIgeHNpOm5vTmFtZXNwYWNlU2NoZW1hTG9jYXRpb249Imh0dHA6Ly9zdW1vLmRsci5kZS94c2QvbmV0ZWRpdENvbmZpZ3VyYXRpb24ueHNkIj4KCiAgICA8aW5wdXQ+CiAgICAgICAgPHN1bW9jZmctZmlsZSB2YWx1ZT0iL2hvbWUvbXBhbC9Eb2N1bWVudHMvU3Vtb1YyVi9zdW1vL3YyaV94bWwvdjJ2LnN1bW9jZmciLz4KICAgICAgICA8cm91dGUtZmlsZXMgdmFsdWU9Ii9ob21lL21wYWwvRG9jdW1lbnRzL1N1bW9WMlYvc3Vtby92MmlfeG1sL3Yydi5yb3UueG1sIi8+CiAgICAgICAgPHN1bW8tbmV0LWZpbGUgdmFsdWU9Ii9ob21lL21wYWwvRG9jdW1lbnRzL1N1bW9WMlYvc3Vtby92MmlfeG1sL3Yydi5uZXQueG1sIi8+CiAgICA8L2lucHV0PgoKICAgIDxvdXRwdXQ+CiAgICAgICAgPG91dHB1dC1maWxlIHZhbHVlPSIvaG9tZS9tcGFsL0RvY3VtZW50cy9TdW1vVjJWL3N1bW8vdjJpX3htbC92MnYubmV0LnhtbCIvPgogICAgPC9vdXRwdXQ+CgogICAgPHByb2Nlc3Npbmc+CiAgICAgICAgPGdlb21ldHJ5Lm1pbi1yYWRpdXMuZml4LnJhaWx3YXlzIHZhbHVlPSJmYWxzZSIvPgogICAgICAgIDxnZW9tZXRyeS5tYXgtZ3JhZGUuZml4IHZhbHVlPSJmYWxzZSIvPgogICAgICAgIDxvZmZzZXQuZGlzYWJsZS1ub3JtYWxpemF0aW9uIHZhbHVlPSJ0cnVlIi8+CiAgICAgICAgPGxlZnRoYW5kIHZhbHVlPSIwIi8+CiAgICA8L3Byb2Nlc3Npbmc+CgogICAgPGp1bmN0aW9ucz4KICAgICAgICA8bm8tdHVybmFyb3VuZHMgdmFsdWU9InRydWUiLz4KICAgICAgICA8anVuY3Rpb25zLmNvcm5lci1kZXRhaWwgdmFsdWU9IjUiLz4KICAgICAgICA8anVuY3Rpb25zLmxpbWl0LXR1cm4tc3BlZWQgdmFsdWU9IjUuNTAiLz4KICAgICAgICA8cmVjdGFuZ3VsYXItbGFuZS1jdXQgdmFsdWU9IjAiLz4KICAgIDwvanVuY3Rpb25zPgoKICAgIDxwZWRlc3RyaWFuPgogICAgICAgIDx3YWxraW5nYXJlYXMgdmFsdWU9IjAiLz4KICAgIDwvcGVkZXN0cmlhbj4KCjwvY29uZmlndXJhdGlvbj4KLS0+Cgo8bmV0IHZlcnNpb249IjEuMTYiIGp1bmN0aW9uQ29ybmVyRGV0YWlsPSI1IiBsaW1pdFR1cm5TcGVlZD0iNS41MCIgeG1sbnM6eHNpPSJodHRwOi8vd3d3LnczLm9yZy8yMDAxL1hNTFNjaGVtYS1pbnN0YW5jZSIgeHNpOm5vTmFtZXNwYWNlU2NoZW1hTG9jYXRpb249Imh0dHA6Ly9zdW1vLmRsci5kZS94c2QvbmV0X2ZpbGUueHNkIj4KCiAgICA8bG9jYXRpb24gbmV0T2Zmc2V0PSIwLjAwLDAuMDAiIGNvbnZCb3VuZGFyeT0iLTEwMC4wMCwtMTUuMDAsMTUwLjAwLDgwLjAwIiBvcmlnQm91bmRhcnk9Ii0xMDAwMDAwMDAwMC4wMCwtMTAwMDAwMDAwMDAuMDAsMTAwMDAwMDAwMDAuMDAsMTAwMDAwMDAwMDAuMDAiIHByb2pQYXJhbWV0ZXI9IiEiLz4KCiAgICA8ZWRnZSBpZD0iOkoxXzAiIGZ1bmN0aW9uPSJpbnRlcm5hbCI+CiAgICAgICAgPGxhbmUgaWQ9IjpKMV8wXzAiIGluZGV4PSIwIiBzcGVlZD0iMTMuODkiIGxlbmd0aD0iMC40NyIgc2hhcGU9IjQ5LjUzLDc4LjQwIDQ5LjY3LDc4LjQwIDQ5Ljc2LDc4LjM5IDQ5Ljg2LDc4LjM3IDQ5Ljk5LDc4LjMzIi8+CiAgICA8L2VkZ2U+CiAgICA8ZWRnZSBpZD0iOkoxMF8wIiBmdW5jdGlvbj0iaW50ZXJuYWwiPgogICAgICAgIDxsYW5lIGlkPSI6SjEwXzBfMCIgaW5kZXg9IjAiIHNwZWVkPSIxMC42MCIgbGVuZ3RoPSI5LjI3IiBzaGFwZT0iMTIyLjMxLDguNTUgMTE5Ljg3LDcuNjUgMTE4LjAzLDcuNzYgMTE2LjExLDguNDYgMTEzLjQ3LDkuMzQiLz4KICAgIDwvZWRnZT4KICAgIDxlZGdlIGlkPSI6SjEwXzEiIGZ1bmN0aW9uPSJpbnRlcm5hbCI+CiAgICAgICAgPGxhbmUgaWQ9IjpKMTBfMV8wIiBpbmRleD0iMCIgc3BlZWQ9IjEzLjg5IiBsZW5ndGg9IjkuMzQiIHNoYXBlPSIxMjIuMzEsOC41NSAxMTkuODMsNy4zNCAxMTguMDUsNi42OCAxMTYuMjEsNi4xNyAxMTMuNTQsNS40MyIvPgogICAgPC9lZGdlPgogICAgPGVkZ2UgaWQ9IjpKMTFfMCIgZnVuY3Rpb249ImludGVybmFsIj4KICAgICAgICA8bGFuZSBpZD0iOkoxMV8wXzAiIGluZGV4PSIwIiBzcGVlZD0iMTMuODkiIGxlbmd0aD0iMC40MCIgc2hhcGU9IjUwLjQwLDI1LjU1IDUwLjI4LDI1LjU4IDUwLjIwLDI1LjU5IDUwLjEyLDI1LjYwIDUwLjAwLDI1LjYwIi8+CiAgICA8L2VkZ2U+CiAgICA8ZWRnZSBpZD0iOkoyXzAiIGZ1bmN0aW9uPSJpbnRlcm5hbCI+CiAgICAgICAgPGxhbmUgaWQ9IjpKMl8wXzAiIGluZGV4PSIwIiBzcGVlZD0iMy45NCIgbGVuZ3RoPSIyLjA3IiBzaGFwZT0iMTQ3LjI2LDQ5LjE1IDE0Ny43Niw0OC45MyAxNDguMTEsNDguNjAgMTQ4LjMzLDQ4LjE2IDE0OC40MCw0Ny42MiIvPgogICAgPC9lZGdlPgogICAgPGVkZ2UgaWQ9IjpKM18wIiBmdW5jdGlvbj0iaW50ZXJuYWwiPgogICAgICAgIDxsYW5lIGlkPSI6SjNfMF8wIiBpbmRleD0iMCIgc3BlZWQ9IjQuMDAiIGxlbmd0aD0iMS43NSIgc2hhcGU9IjE0OC40MCwyMy4xNyAxNDguMzUsMjIuNzIgMTQ4LjE5LDIyLjMzIDE0Ny45MiwyMi4wMSAxNDcuNTQsMjEuNzUiLz4KICAgIDwvZWRnZT4KICAgIDxlZGdlIGlkPSI6SjhfMCIgZnVuY3Rpb249ImludGVybmFsIj4KICAgICAgICA8bGFuZSBpZD0iOko4XzBfMCIgaW5kZXg9IjAiIHNwZWVkPSIxMy44OSIgbGVuZ3RoPSIwLjQ2IiBzaGFwZT0iNDkuOTksLTEzLjMzIDQ5Ljg2LC0xMy4zNyA0OS43NywtMTMuMzkgNDkuNjcsLTEzLjQwIDQ5LjU0LC0xMy40MCIvPgogICAgPC9lZGdlPgoKICAgIDxlZGdlIGlkPSJFMCIgZnJvbT0iSjAiIHRvPSJKMSIgcHJpb3JpdHk9Ii0xIj4KICAgICAgICA8bGFuZSBpZD0iRTBfMCIgaW5kZXg9IjAiIHNwZWVkPSIxMy44OSIgbGVuZ3RoPSIxNDkuNTMiIHNoYXBlPSItMTAwLjAwLDc4LjQwIDQ5LjUzLDc4LjQwIi8+CiAgICA8L2VkZ2U+CiAgICA8ZWRnZSBpZD0iRTEiIGZyb209IkoxIiB0bz0iSjIiIHByaW9yaXR5PSItMSI+CiAgICAgICAgPGxhbmUgaWQ9IkUxXzAiIGluZGV4PSIwIiBzcGVlZD0iMTMuODkiIGxlbmd0aD0iMTAxLjU1IiBzaGFwZT0iNDkuOTksNzguMzMgMTQ3LjI2LDQ5LjE1Ii8+CiAgICA8L2VkZ2U+CiAgICA8ZWRnZSBpZD0iRTEwIiBmcm9tPSJKMTAiIHRvPSJKOCIgcHJpb3JpdHk9Ii0xIj4KICAgICAgICA8bGFuZSBpZD0iRTEwXzAiIGluZGV4PSIwIiBzcGVlZD0iMTMuODkiIGxlbmd0aD0iNjYuMjYiIHNoYXBlPSIxMTMuNTQsNS40MyA0OS45OSwtMTMuMzMiLz4KICAgIDwvZWRnZT4KICAgIDxlZGdlIGlkPSJFMTEiIGZyb209IkoxMCIgdG89IkoxMSIgcHJpb3JpdHk9Ii0xIj4KICAgICAgICA8bGFuZSBpZD0iRTExXzAiIGluZGV4PSIwIiBzcGVlZD0iMTMuODkiIGxlbmd0aD0iNjUuMTIiIHNoYXBlPSIxMTMuNDcsOS4zNCA1MC40MCwyNS41NSIvPgogICAgPC9lZGdlPgogICAgPGVkZ2UgaWQ9IkUxMiIgZnJvbT0iSjExIiB0bz0iSjEyIiBwcmlvcml0eT0iLTEiPgogICAgICAgIDxsYW5lIGlkPSJFMTJfMCIgaW5kZXg9IjAiIHNwZWVkPSIxMy44OSIgbGVuZ3RoPSIxNTAuMDAiIHNoYXBlPSI1MC4wMCwyNS42MCAtMTAwLjAwLDI1LjYwIi8+CiAgICA8L2VkZ2U+CiAgICA8ZWRnZSBpZD0iRTIiIGZyb209IkoyIiB0bz0iSjMiIHByaW9yaXR5PSItMSI+CiAgICAgICAgPGxhbmUgaWQ9IkUyXzAiIGluZGV4PSIwIiBzcGVlZD0iMTMuODkiIGxlbmd0aD0iMjQuNDUiIHNoYXBlPSIxNDguNDAsNDcuNjIgMTQ4LjQwLDIzLjE3Ii8+CiAgICA8L2VkZ2U+CiAgICA8ZWRnZSBpZD0iRTgiIGZyb209Iko4IiB0bz0iSjkiIHByaW9yaXR5PSItMSI+CiAgICAgICAgPGxhbmUgaWQ9IkU4XzAiIGluZGV4PSIwIiBzcGVlZD0iMTMuODkiIGxlbmd0aD0iMTQ5LjU0IiBzaGFwZT0iNDkuNTQsLTEzLjQwIC0xMDAuMDAsLTEzLjQwIi8+CiAgICA8L2VkZ2U+CiAgICA8ZWRnZSBpZD0iRTkiIGZyb209IkozIiB0bz0iSjEwIiBwcmlvcml0eT0iLTEiPgogICAgICAgIDxsYW5lIGlkPSJFOV8wIiBpbmRleD0iMCIgc3BlZWQ9IjEzLjg5IiBsZW5ndGg9IjI4LjQ4IiBzaGFwZT0iMTQ3LjU0LDIxLjc1IDEyMi4zMSw4LjU1Ii8+CiAgICA8L2VkZ2U+CgogICAgPHRsTG9naWMgaWQ9IkoxMSIgdHlwZT0ic3RhdGljIiBwcm9ncmFtSUQ9IjAiIG9mZnNldD0iMCI+CiAgICAgICAgPHBoYXNlIGR1cmF0aW9uPSIyIiAgc3RhdGU9IkciLz4KICAgICAgICA8cGhhc2UgZHVyYXRpb249IjEwIiAgc3RhdGU9InkiLz4KICAgICAgICA8cGhhc2UgZHVyYXRpb249IjIwIiAgc3RhdGU9InIiLz4KICAgIDwvdGxMb2dpYz4KICAgIDx0bExvZ2ljIGlkPSJKOCIgdHlwZT0ic3RhdGljIiBwcm9ncmFtSUQ9IjAiIG9mZnNldD0iMCI+CiAgICAgICAgPHBoYXNlIGR1cmF0aW9uPSIyIiAgc3RhdGU9IkciLz4KICAgICAgICA8cGhhc2UgZHVyYXRpb249IjIwIiAgc3RhdGU9InkiLz4KICAgICAgICA8cGhhc2UgZHVyYXRpb249IjEwIiAgc3RhdGU9InIiLz4KICAgIDwvdGxMb2dpYz4KCiAgICA8anVuY3Rpb24gaWQ9IkowIiB0eXBlPSJkZWFkX2VuZCIgeD0iLTEwMC4wMCIgeT0iODAuMDAiIGluY0xhbmVzPSIiIGludExhbmVzPSIiIHNoYXBlPSItMTAwLjAwLDgwLjAwIC0xMDAuMDAsNzYuODAiLz4KICAgIDxqdW5jdGlvbiBpZD0iSjEiIHR5cGU9InByaW9yaXR5IiB4PSI1MC4wMCIgeT0iODAuMDAiIGluY0xhbmVzPSJFMF8wIiBpbnRMYW5lcz0iOkoxXzBfMCIgc2hhcGU9IjUwLjQ1LDc5Ljg3IDQ5LjUzLDc2LjgwIDQ5LjUzLDgwLjAwIDQ5Ljg4LDc5Ljk5IDUwLjAwLDc5Ljk4IDUwLjEyLDc5Ljk2IDUwLjI2LDc5LjkyIj4KICAgICAgICA8cmVxdWVzdCBpbmRleD0iMCIgcmVzcG9uc2U9IjAiIGZvZXM9IjAiIGNvbnQ9IjAiLz4KICAgIDwvanVuY3Rpb24+CiAgICA8anVuY3Rpb24gaWQ9IkoxMCIgdHlwZT0icHJpb3JpdHkiIHg9IjEyMC42MCIgeT0iNS44NSIgaW5jTGFuZXM9IkU5XzAiIGludExhbmVzPSI6SjEwXzBfMCA6SjEwXzFfMCIgc2hhcGU9IjEyMS41Nyw5Ljk3IDEyMy4wNSw3LjEzIDEyMS4yMiw2LjIxIDExOS44Myw1LjYxIDExOC42NSw1LjE5IDExNy40NCw0Ljg1IDExNS45Nyw0LjQ2IDExMy45OSwzLjkwIDExMy4wOSw2Ljk3IDExMy4wNyw3Ljc5IDExMy44NywxMC44OCAxMTYuNzcsOS44MiAxMTcuODQsOS4zOSAxMTguODgsOS4xOSAxMjAuMDcsOS4zNCI+CiAgICAgICAgPHJlcXVlc3QgaW5kZXg9IjAiIHJlc3BvbnNlPSIwMCIgZm9lcz0iMDAiIGNvbnQ9IjAiLz4KICAgICAgICA8cmVxdWVzdCBpbmRleD0iMSIgcmVzcG9uc2U9IjAwIiBmb2VzPSIwMCIgY29udD0iMCIvPgogICAgPC9qdW5jdGlvbj4KICAgIDxqdW5jdGlvbiBpZD0iSjExIiB0eXBlPSJ0cmFmZmljX2xpZ2h0IiB4PSI1MC4wMCIgeT0iMjQuMDAiIGluY0xhbmVzPSJFMTFfMCIgaW50TGFuZXM9IjpKMTFfMF8wIiBzaGFwZT0iNTAuODAsMjcuMTAgNTAuMDAsMjQuMDAgNTAuMDAsMjcuMjAgNTAuMzAsMjcuMjAgNTAuNDAsMjcuMTkgNTAuNTEsMjcuMTcgNTAuNjMsMjcuMTQiPgogICAgICAgIDxyZXF1ZXN0IGluZGV4PSIwIiByZXNwb25zZT0iMCIgZm9lcz0iMCIgY29udD0iMCIvPgogICAgPC9qdW5jdGlvbj4KICAgIDxqdW5jdGlvbiBpZD0iSjEyIiB0eXBlPSJkZWFkX2VuZCIgeD0iLTEwMC4wMCIgeT0iMjQuMDAiIGluY0xhbmVzPSJFMTJfMCIgaW50TGFuZXM9IiIgc2hhcGU9Ii0xMDAuMDAsMjcuMjAgLTEwMC4wMCwyNC4wMCIvPgogICAgPGp1bmN0aW9uIGlkPSJKMiIgdHlwZT0icHJpb3JpdHkiIHg9IjE1MC4wMCIgeT0iNTAuMDAiIGluY0xhbmVzPSJFMV8wIiBpbnRMYW5lcz0iOkoyXzBfMCIgc2hhcGU9IjE1MC4wMCw0Ny42MiAxNDYuODAsNDcuNjIgMTQ3LjcyLDUwLjY4IDE0OC45OSw1MC4wNCAxNDkuNDMsNDkuNTggMTQ5Ljc1LDQ5LjAyIDE0OS45NCw0OC4zNyI+CiAgICAgICAgPHJlcXVlc3QgaW5kZXg9IjAiIHJlc3BvbnNlPSIwIiBmb2VzPSIwIiBjb250PSIwIi8+CiAgICA8L2p1bmN0aW9uPgogICAgPGp1bmN0aW9uIGlkPSJKMyIgdHlwZT0icHJpb3JpdHkiIHg9IjE1MC4wMCIgeT0iMjEuMjMiIGluY0xhbmVzPSJFMl8wIiBpbnRMYW5lcz0iOkozXzBfMCIgc2hhcGU9IjE0Ni44MCwyMy4xNyAxNTAuMDAsMjMuMTcgMTQ5LjgxLDIxLjk5IDE0OS41NywyMS40OSAxNDkuMjQsMjEuMDUgMTQ4LjgxLDIwLjY2IDE0OC4yOCwyMC4zMyI+CiAgICAgICAgPHJlcXVlc3QgaW5kZXg9IjAiIHJlc3BvbnNlPSIwIiBmb2VzPSIwIiBjb250PSIwIi8+CiAgICA8L2p1bmN0aW9uPgogICAgPGp1bmN0aW9uIGlkPSJKOCIgdHlwZT0idHJhZmZpY19saWdodCIgeD0iNTAuMDAiIHk9Ii0xNS4wMCIgaW5jTGFuZXM9IkUxMF8wIiBpbnRMYW5lcz0iOko4XzBfMCIgc2hhcGU9IjQ5LjU0LC0xMS44MCA1MC40NCwtMTQuODcgNTAuMTIsLTE0Ljk2IDUwLjAwLC0xNC45OCA0OS44OCwtMTQuOTkgNDkuNzMsLTE1LjAwIDQ5LjU0LC0xNS4wMCI+CiAgICAgICAgPHJlcXVlc3QgaW5kZXg9IjAiIHJlc3BvbnNlPSIwIiBmb2VzPSIwIiBjb250PSIwIi8+CiAgICA8L2p1bmN0aW9uPgogICAgPGp1bmN0aW9uIGlkPSJKOSIgdHlwZT0iZGVhZF9lbmQiIHg9Ii0xMDAuMDAiIHk9Ii0xNS4wMCIgaW5jTGFuZXM9IkU4XzAiIGludExhbmVzPSIiIHNoYXBlPSItMTAwLjAwLC0xMS44MCAtMTAwLjAwLC0xNS4wMCIvPgoKICAgIDxjb25uZWN0aW9uIGZyb209IkUwIiB0bz0iRTEiIGZyb21MYW5lPSIwIiB0b0xhbmU9IjAiIHZpYT0iOkoxXzBfMCIgZGlyPSJzIiBzdGF0ZT0iTSIvPgogICAgPGNvbm5lY3Rpb24gZnJvbT0iRTEiIHRvPSJFMiIgZnJvbUxhbmU9IjAiIHRvTGFuZT0iMCIgdmlhPSI6SjJfMF8wIiBkaXI9InIiIHN0YXRlPSJNIi8+CiAgICA8Y29ubmVjdGlvbiBmcm9tPSJFMTAiIHRvPSJFOCIgZnJvbUxhbmU9IjAiIHRvTGFuZT0iMCIgdmlhPSI6SjhfMF8wIiB0bD0iSjgiIGxpbmtJbmRleD0iMCIgZGlyPSJzIiBzdGF0ZT0iTyIvPgogICAgPGNvbm5lY3Rpb24gZnJvbT0iRTExIiB0bz0iRTEyIiBmcm9tTGFuZT0iMCIgdG9MYW5lPSIwIiB2aWE9IjpKMTFfMF8wIiB0bD0iSjExIiBsaW5rSW5kZXg9IjAiIGRpcj0icyIgc3RhdGU9Ik8iLz4KICAgIDxjb25uZWN0aW9uIGZyb209IkUyIiB0bz0iRTkiIGZyb21MYW5lPSIwIiB0b0xhbmU9IjAiIHZpYT0iOkozXzBfMCIgZGlyPSJyIiBzdGF0ZT0iTSIvPgogICAgPGNvbm5lY3Rpb24gZnJvbT0iRTkiIHRvPSJFMTEiIGZyb21MYW5lPSIwIiB0b0xhbmU9IjAiIHZpYT0iOkoxMF8wXzAiIGRpcj0iUiIgc3RhdGU9Ik0iLz4KICAgIDxjb25uZWN0aW9uIGZyb209IkU5IiB0bz0iRTEwIiBmcm9tTGFuZT0iMCIgdG9MYW5lPSIwIiB2aWE9IjpKMTBfMV8wIiBkaXI9InMiIHN0YXRlPSJNIi8+CgogICAgPGNvbm5lY3Rpb24gZnJvbT0iOkoxXzAiIHRvPSJFMSIgZnJvbUxhbmU9IjAiIHRvTGFuZT0iMCIgZGlyPSJzIiBzdGF0ZT0iTSIvPgogICAgPGNvbm5lY3Rpb24gZnJvbT0iOkoxMF8wIiB0bz0iRTExIiBmcm9tTGFuZT0iMCIgdG9MYW5lPSIwIiBkaXI9IlIiIHN0YXRlPSJNIi8+CiAgICA8Y29ubmVjdGlvbiBmcm9tPSI6SjEwXzEiIHRvPSJFMTAiIGZyb21MYW5lPSIwIiB0b0xhbmU9IjAiIGRpcj0icyIgc3RhdGU9Ik0iLz4KICAgIDxjb25uZWN0aW9uIGZyb209IjpKMTFfMCIgdG89IkUxMiIgZnJvbUxhbmU9IjAiIHRvTGFuZT0iMCIgZGlyPSJzIiBzdGF0ZT0iTSIvPgogICAgPGNvbm5lY3Rpb24gZnJvbT0iOkoyXzAiIHRvPSJFMiIgZnJvbUxhbmU9IjAiIHRvTGFuZT0iMCIgZGlyPSJyIiBzdGF0ZT0iTSIvPgogICAgPGNvbm5lY3Rpb24gZnJvbT0iOkozXzAiIHRvPSJFOSIgZnJvbUxhbmU9IjAiIHRvTGFuZT0iMCIgZGlyPSJyIiBzdGF0ZT0iTSIvPgogICAgPGNvbm5lY3Rpb24gZnJvbT0iOko4XzAiIHRvPSJFOCIgZnJvbUxhbmU9IjAiIHRvTGFuZT0iMCIgZGlyPSJzIiBzdGF0ZT0iTSIvPgoKPC9uZXQ+"
    decoded_string = base64.b64decode(doc)

    TLS = namedtuple('TLS', ['g', 'y', 'r'])

    tsl_fast_config = {'G': 5, 'y': 5, 'r': 5}
    tsl_slow_config = {'G': 2, 'y': 10, 'r': 10}
    
    root = etree.XML(decoded_string)

    switch = True
    for element in root.findall( "tlLogic"):
        seq = ("G", 'y', 'r')
        if switch:
            values = tsl_fast_config
        else:
            values = tsl_slow_config
        for phase,s in zip(element, seq):
            phase.set("duration", str(values[s]))
        switch = not switch

    etree.indent(root)
    with open(net_file, "wb") as f:
        data = etree.tostring(root, encoding='utf-8', xml_declaration=True, pretty_print=True)
        f.write(data)

def generate_traffic(route_file):

    doc = "PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0iVVRGLTgiPz4KCjwhLS0gZ2VuZXJhdGVkIG9uIDIwMjMtMDktMDkgMDA6MDg6MTYgYnkgRWNsaXBzZSBTVU1PIG5ldGVkaXQgVmVyc2lvbiAxLjE4LjAKLS0+Cgo8cm91dGVzIHhtbG5zOnhzaT0iaHR0cDovL3d3dy53My5vcmcvMjAwMS9YTUxTY2hlbWEtaW5zdGFuY2UiIHhzaTpub05hbWVzcGFjZVNjaGVtYUxvY2F0aW9uPSJodHRwOi8vc3Vtby5kbHIuZGUveHNkL3JvdXRlc19maWxlLnhzZCI+CiAgICA8IS0tIFJvdXRlcyAtLT4KICAgIDxyb3V0ZSBpZD0icl8wIiBlZGdlcz0iRTAgRTEgRTIgRTkgRTExIEUxMiIvPgogICAgPHJvdXRlIGlkPSJyXzEiIGVkZ2VzPSJFMCBFMSBFMiBFOSBFMTAgRTgiLz4KICAgIDwhLS0gVmVoaWNsZXMsIHBlcnNvbnMgYW5kIGNvbnRhaW5lcnMgKHNvcnRlZCBieSBkZXBhcnQpIC0tPgo8L3JvdXRlcz4K"
    decoded_string = base64.b64decode(doc)
    
    num_vehicles = 50
    ego_vehicle_id = num_vehicles//2
    root = etree.XML(decoded_string)

    # Define vehicle properties
    vType_ego = etree.Element('vType')
    vType_ego.set("id", "ego_vtype")
    vType_ego.set("length", "5")
    vType_ego.set("maxSpeed", "50")
    vType_ego.set("accel", "2")
    vType_ego.set("decel", "1")
    #vType_ego.set("minGap", "0")
    root.append(vType_ego)


    vType_car = etree.Element('vType')
    vType_car.set("id", "car_vtype")
    vType_car.set("length", "5")
    vType_car.set("maxSpeed", "50")
    vType_car.set("accel", "2")
    vType_car.set("decel", "1")
    root.append(vType_car)

    switch = True
    
    for veh in range(num_vehicles):

        trip = etree.Element('vehicle')

        if veh == ego_vehicle_id:
            veh_id = "ego"
            color = "red"
            trip.set("type", "ego_vtype")
        else:
            veh_id = 't_' + str(veh)
            color = "yellow"
            trip.set("type", "car_vtype")
        
        
        trip.set("id", veh_id)
        trip.set("depart", "0.00")
        trip.set("color", color)
        

        if switch:
            trip.set("route", "r_0")
        else:
            trip.set("route", "r_1")

        switch = not switch

        root.append(trip)
    
    etree.indent(root)
    with open(route_file, "wb") as f:
        data = etree.tostring(root, encoding='utf-8', xml_declaration=True, pretty_print=True)
        f.write(data)

if __name__ == "__main__":

    options = get_options()
    
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    configure_tls("v2i_xml/v2v.net.xml")
    generate_traffic("v2i_xml/v2v.rou.xml")

    t_step = 1.0

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "v2i_xml/v2v.sumocfg", "--step-length", str(t_step), "--collision.mingap-factor", "0", "--collision.action", "warn"])
    
    run(t_step)

    obj = SumoV2I()
    