# from bluesky.simulation import ScreenIO
import geopy.distance
import pandas as pd
import energy as en
import line as le
import model as md
import sympy as sp
import os

# class ScreenDummy(ScreenIO):
#     """
#     Dummy class for the screen. Inherits from ScreenIO to make sure all the
#     necessary methods are there. This class is there to reimplement the echo
#     method so that console messages are printed.
#     """
#
#     def echo(self, text='', flags=0):
#         """Just print echo messages"""
#         print("BlueSky console:", text)
# Hyper-parameters
KNOTS = 0.514444
Y_DELIVERY = 6000


def compute_takeoff_landing_OPT_PP(payload, drone_speed, wind_speed, wind_direction, n_class=6, verbose=True):
    """
    	point: [P, H, HP, side]
    	env_param: [Drone_speed,wind_speed,wind_direction,payload]
    	w_direction: wind direction
    	DP_side: point P line side
    	p: alpha angles first quadrant
    	n: alpha angles fourth quadrant
    	mt : [mt,mt_angle, direction]
    	env_dict : [dict_classes, dict_sect, dict_unit]
    """
    POINT = sp.Point(1, Y_DELIVERY)
    wind_classes, wind_sector, relative_wind_values = md.compute_wind_classes_strict(n_class)
    energy_dict = md.compute_prefixes(payload, drone_speed, wind_speed, relative_wind_values)
    mt, point = le.create_mt(0, 1, POINT)
    # print(point)
    env_dict = [wind_classes, wind_sector, energy_dict]
    env = [drone_speed, wind_speed, wind_direction, payload]
    p_slack, n_slack = md.list_delivery_slack(n_class, mt[1], wind_direction)
    takeoffs, landings = md.compute_takeoff_landing_strict(point, env, env_dict, mt, p_slack, n_slack)

    best_takeoff = min(takeoffs, key=lambda t: t[3])
    best_landing = min(landings, key=lambda t: t[3])
    ### EFFICIENT DISTANCE
    opt_takeoff_point = best_takeoff[0]
    opt_landing_point = best_landing[0]
    opt_cost = best_takeoff[5] * best_takeoff[2] + best_landing[5] * best_landing[2]

    opt_dist = best_takeoff[5] + best_landing[5]
    index = (len(p_slack) - 1)
    ### SHORTEST DISTANCE
    pp_takeoff_point = takeoffs[index][0]
    pp_landing_point = landings[index][0]
    pp_cost = takeoffs[index][5] * takeoffs[index][2] + landings[index][5] * landings[index][2]
    pp_dist = takeoffs[index][5] + landings[index][5]
    # print(best_takeoff[2])
    # print(takeoffs[index][2])
    if verbose:
        print("VERBOSE MODE: ", verbose)
        print("Parameters:")
        print("Matemathical Wind Direction: ", wind_direction, "° Meteo Wind Direction: ",
              md.math2meteo(wind_direction), "°")
        print("Wind Speed: ", wind_speed, " m/s Wind Speed: ", round(wind_speed / KNOTS, 4), " kts")
        print("Drone Speed: ", drone_speed, "m/s Drone Speed: ", round(drone_speed / KNOTS, 4), " kts")
        print("Takeoff: ", sp.N(best_takeoff[0]), "")
        print("Landing: ", sp.N(best_landing[0]), "")
        print("Takeoff-Angle: ", best_takeoff[7])
        print("landing-Angle: ", best_landing[7])
        print("OPT Trajectories Cost: ", opt_cost, " J")
        print("OPT Trajectories Length: ", opt_dist, " m")
        print("PP Trajectories Cost: ", pp_cost, " J")
        print("PP Trajectories Length: ", pp_dist, " meters")
    return [[opt_takeoff_point.x, opt_takeoff_point.y], [opt_landing_point.x, opt_landing_point.y], opt_cost,
            opt_dist], [
               [pp_takeoff_point.x, pp_takeoff_point.y], [pp_landing_point.x, pp_landing_point.y], pp_cost, pp_dist]


def compute_track_points(orig, end, type="T", granularity=8):
    track_wps = le.intermediates(orig, end, granularity)
    track_gps = [md.xy_to_gps(p) for p in track_wps]
    command = ""
    for index in range(len(track_gps)):
        wp = "W" + type + str(index)
        command = command + "0:00:00>DEFWPT " + wp + " " + str(track_gps[index][0]) + " " + str(
            track_gps[index][1]) + "\n"
    return command


def create_scenarios_opt_pp(drone_speed, payload, wind_speed, wind_direction, granularity, n_class=6):
    opt, pp = compute_takeoff_landing_OPT_PP(payload, drone_speed, wind_speed, wind_direction, n_class, False)
    print("Creating opt scenario...")
    command1 = create_scenario(opt, drone_speed, wind_speed, wind_direction, granularity, "opt", n_class)
    print("opt scenario created.\n")
    print("Creating pp scenario...")
    command2 = create_scenario(pp, drone_speed, wind_speed, wind_direction, granularity, "pp", n_class)
    print("pp scenario created.\n")
    return command1 +"\n"+ command2


def create_scenario(trj, drone_speed, wind_speed, wind_direction, granularity, type="opt", n_class=6):
    ff_name = os.path.expanduser('~') + "/bluesky/scenario/scenario_" + str(type) + "_ds" + str(
        drone_speed) + "_ws" + str(
        wind_speed) + "_wd" + str(
        wind_direction) + "_c" + str(
        n_class) + ".scn"
    ff = open(ff_name, "w")
    ff.write("0:00:00>TAXI OFF 1\n")
    ff.write("0:00:00>TRAILS ON\n")
    # waypoints creation
    opt_T_x, opt_T_y = md.xy_to_gps(trj[0])
    opt_L_x, opt_L_y = md.xy_to_gps(trj[1])
    dev_x, dev_y = md.xy_to_gps([1, Y_DELIVERY])
    DEAD_x, DEAD_y = md.xy_to_gps((trj[1][0] - 500, trj[1][1] - 500))
    command_wp = "0:00:00>DEFWPT Takeoff " + str(opt_T_x) + " " + str(opt_T_y) + "\n"
    ff.write(command_wp)
    command_wp = "0:00:00>DEFWPT Landing " + str(opt_L_x) + " " + str(opt_L_y) + "\n"
    ff.write(command_wp)
    command_wp = "0:00:00>DEFWPT Delivery " + str(dev_x) + " " + str(dev_y) + "\n"
    ff.write(command_wp)
    command_wp = "0:00:00>DEFWPT DEAD " + str(DEAD_x) + " " + str(DEAD_y) + "\n"
    ff.write(command_wp)
    command_wp = compute_track_points(trj[0], [1, Y_DELIVERY], "T", granularity)
    ff.write(command_wp)
    command_wp = compute_track_points([1, Y_DELIVERY], trj[1], "L", granularity)
    ff.write(command_wp)
    # M600 creation in pos T
    command_M600 = "0:00:00>CRE DRONE M600 Takeoff 0 40 " + str(round(drone_speed / KNOTS, 4)) + "\n"
    ff.write(command_M600)
    command_M600 = "0:00:00>PAN DRONE\n"
    ff.write(command_M600)
    command_M600 = "0:00:00>POS DRONE\n"
    ff.write(command_M600)
    command_M600 = "0:00:00>ZOOM 50\n"
    ff.write(command_M600)
    # wind
    command_wind = "0:00:00>WIND " + str(opt_T_x) + " " + str(opt_T_y) + " " + str(
        md.math2meteo(wind_direction)) + " " + str(round(wind_speed / KNOTS, 4)) + "\n"
    ff.write(command_wind)
    # log
    command_log = "0:00:00>CRELOG " + str(type) + "_ds" + str(drone_speed) + "_ws" + str(wind_speed) + "_wd" + str(
        wind_direction) + "_c" + str(n_class) + " 1\n"
    ff.write(command_log)
    command_log = "0:00:00>" + str(type) + "_ds" + str(drone_speed) + "_ws" + str(wind_speed) + "_wd" + str(
        wind_direction) + "_c" + str(n_class) + " ADD traf.lat, traf.lon, traf.gs, traf.tas, traf.trk, traf.hdg\n"
    ff.write(command_log)
    command_log = "0:00:00>" + str(type) + "_ds" + str(drone_speed) + "_ws" + str(wind_speed) + "_wd" + str(
        wind_direction) + "_c" + str(n_class) + " ON\n"
    ff.write(command_log)
    # mission
    command_mission = "0:00:01>ORIG DRONE Takeoff\n"
    ff.write(command_mission)
    if granularity != 0:
        command_mission = "0:00:01> DRONE AFTER Takeoff ADDWPT WT0\n"
        ff.write(command_mission)
        for i in range(granularity - 1):
            name1 = "WT" + str(i)
            name2 = "WT" + str(i + 1)
            command_mission = "0:00:01> DRONE AFTER " + str(name1) + " ADDWPT " + str(name2) + "\n"
            ff.write(command_mission)
        # command_mission = "0:00:01> DRONE AFTER Takeoff ADDWPT Delivery\n"
        # ff.write(command_mission)
        command_mission = "0:00:01> DRONE AFTER " + str(name2) + " ADDWPT Delivery\n"
        ff.write(command_mission)
        command_mission = "0:00:01> DRONE AT Delivery STACK DRONE AFTER Delivery ADDWPT WL0\n"
        ff.write(command_mission)
        for i in range(granularity - 1):
            name1 = "WL" + str(i)
            name2 = "WL" + str(i + 1)
            command_mission = "0:00:01> DRONE AT Delivery STACK DRONE AFTER " + str(name1) + " ADDWPT " + str(
                name2) + "\n"
            ff.write(command_mission)
        command_mission = "0:00:01> DRONE AT Delivery STACK DRONE AFTER " + str(name2) + " ADDWPT Landing\n"
        ff.write(command_mission)
    else:
        command_mission = "0:00:01> DRONE AFTER Takeoff ADDWPT Delivery\n"
        ff.write(command_mission)
        command_mission = "0:00:01> DRONE AT Delivery STACK DRONE ADDWPT Landing\n"
        ff.write(command_mission)
    command_mission = "0:00:01>ADDWPTMODE DRONE FLYOVER\n"
    ff.write(command_mission)
    command_mission = "0:00:01>VNAV DRONE ON\n"
    ff.write(command_mission)
    command_mission = "0:00:01>LNAV DRONE ON\n"
    ff.write(command_mission)
    command_mission = "0:00:01>DRONE AT Delivery STACK DRONE AT Landing STACK " + str(type) + "_ds" + str(
        drone_speed) + "_ws" + str(
        wind_speed) + "_wd" + str(wind_direction) + "_c" + str(n_class) + " OFF\n"
    ff.write(command_mission)
    command_mission = "0:00:01>DRONE AT Delivery STACK DRONE AT Landing STACK DRONE ADDWPT DEAD\n"
    ff.write(command_mission)
    command_mission = "0:00:01>DRONE AT Delivery STACK DRONE AT Landing STACK DRONE AT DEAD STACK DRONE SPD 0\n"
    ff.write(command_mission)
    command_mission = "0:00:01>DRONE ATSPD 0, QUIT\n"
    ff.write(command_mission)
    command_mission = "0:00:01>DTMULT 30\n"
    ff.write(command_mission)
    command_mission = "0:00:10>FF\n"
    ff.write(command_mission)
    print("COMMAND TO RUN: bluesky --scenfile scenario_" + str(type) + "_ds" + str(drone_speed) + "_ws" + str(
            wind_speed) + "_wd" + str(
            wind_direction) + "_c" + str(
            n_class) + ".scn --detached")
    batch_command = "bluesky --scenfile scenario_" + str(type) + "_ds" + str(drone_speed) + "_ws" + str(
            wind_speed) + "_wd" + str(
            wind_direction) + "_c" + str(
            n_class) + ".scn --detached"
    # os.system("bluesky --scenfile scenario_" + str(type) + "_ds" + str(drone_speed) + "_ws" + str(
    #         wind_speed) + "_wd" + str(
    #         wind_direction) + "_c" + str(
    #         n_class) + ".scn --detached")
    return batch_command


def blueskylog2csv():
    directory = os.path.expanduser('~') + "/bluesky/output/"
    for file in os.listdir(directory):
        csv_output = file.replace(".log", "-log") + ".csv"
        if not os.path.exists(directory + csv_output) and not file.endswith(".csv"):
            with open(directory + file, 'r') as infile, open(directory + csv_output, 'w') as outfile:
                data = infile.read()
                data = data.replace("#", "")
                data = data.replace(" ", "")
                outfile.write(data)
    return


def csv2energy():
    directory = os.path.expanduser('~') + "/bluesky/output/"
    for file in os.listdir(directory):
        if file.endswith(".csv"):
            df = pd.read_csv(directory + file)
            name_split = file.split("_")
            trj_type = name_split[0]
            ds = int(name_split[1].split("DS")[1])
            ws = int(name_split[2].split("WS")[1])
            wd = int(name_split[3].split("WD")[1])
            cl = int(name_split[4].split("C")[1])
            wind_classes, wind_sector, relative_wind_values = md.compute_wind_classes_strict(cl)
            energy_dict = md.compute_prefixes(2, ds, ws, relative_wind_values)
            env_dict = [wind_classes, wind_sector, energy_dict]
            csv_parsing(trj_type, df, ds, ws, wd, cl, env_dict)
    return


def csv_parsing(trj_type, csv, ds, ws, wd, cl, dict_en):
    n = len(csv)
    energy = 0
    tot_dist = 0
    for index in range(n - 1):
        coo1 = (csv["lat"][index], csv["lon"][index])
        coo2 = (csv["lat"][index + 1], csv["lon"][index + 1])
        rel_wind = (md.math2meteo(wd) - (csv["hdg"][index + 1] + 180)) % 360
        # print(index, " relative wind: ",rel_wind)
        wc = md.query_wind_dict(int(rel_wind), dict_en[0])
        unit_t = dict_en[2][0, wc]
        distance = compute_distance(coo1, coo2)
        cost_t = unit_t * distance
        #print(index, " Heading: ", csv["hdg"][index + 1], " relative wind: ", rel_wind, " Consumption: ", cost_t)
        energy = energy + cost_t
        tot_dist = tot_dist + distance
    opt, pp = compute_takeoff_landing_OPT_PP(0, ds, ws, wd, cl)
    print("TYPE: ", trj_type)
    print("TOT DISTANCE: ", tot_dist, " TOT ENERGY: ", energy)
    csv_en = pd.DataFrame()
    if trj_type == "PP":
        csv_en["pp_cost_model"] = [pp[2]]
        csv_en["pp_cost_sim"] = [energy]
        csv_en["pp_dist_model"] = [pp[3]]
        csv_en["pp_dist_sim"] = [tot_dist]
    else:
        csv_en["opt_cost_model"] = [opt[2]]
        csv_en["opt_cost_sim"] = [energy]
        csv_en["opt_dist_model"] = [opt[3]]
        csv_en["opt_dist_sim"] = [tot_dist]
    output_name = "energy/nopayload_" + trj_type + "_static_ds" + str(ds) + "_ws" + str(ws) + "_wd" + str(
        wd) + "_c" + str(cl) + ".csv"
    csv_en.to_csv(output_name)
    return


def compute_distance(coo1, coo2):
    return geopy.distance.geodesic(coo1, coo2).m


def create_scenarios(granularity, n_class=6):
    dss = [10, 20, 30]
    wss = [10, 20]
    wds = [value for value in range(0, 360, 15)]
    f = open("run-test.bat", "w")
    for ds in dss:
        for ws in wss:
            for wd in wds:
                lines = create_scenarios_opt_pp(ds, 0, ws, wd, granularity, n_class)
                f.write(lines+"\n")
    f.close()
    return

def process_results():
    #TODO: rendere dinamico il controllo della cartella sorgente
    dss = [10, 20, 30]
    wss = [10, 20]
    wds = [value for value in range(0, 360, 15)]
    classes = [6]

    for cl in classes:
        for ds in dss:
            for ws in wss:
                csv_rs = pd.DataFrame()
                opt_energy_model = []
                opt_dist_model = []
                opt_energy_sim = []
                opt_dist_sim = []
                pp_energy_model = []
                pp_dist_model = []
                pp_energy_sim = []
                pp_dist_sim = []
                for wd in wds:
                    name1 = "energy/nopayload_OPT_static_ds"+str(ds)+"_ws"+str(ws)+"_wd"+str(wd)+"_c"+str(cl)+".csv"
                    csv_opt = pd.read_csv(name1)
                    opt_energy_model.append(csv_opt["opt_cost_model"].iloc[0])
                    opt_dist_model.append(csv_opt["opt_dist_model"].iloc[0])
                    opt_energy_sim.append(csv_opt["opt_cost_sim"].iloc[0])
                    opt_dist_sim.append(csv_opt["opt_dist_sim"].iloc[0])
                    name2 = "energy/nopayload_PP_static_ds" + str(ds) + "_ws" + str(ws) + "_wd" + str(wd) + "_c" + str(
                        cl) + ".csv"
                    csv_pp = pd.read_csv(name2)
                    pp_energy_model.append(csv_pp["pp_cost_model"].iloc[0])
                    pp_dist_model.append(csv_pp["pp_dist_model"].iloc[0])
                    pp_energy_sim.append(csv_pp["pp_cost_sim"].iloc[0])
                    pp_dist_sim.append(csv_pp["pp_dist_sim"].iloc[0])
                csv_rs["opt_cost_model"] = opt_energy_model
                csv_rs["opt_dist_model"] = opt_dist_model
                csv_rs["opt_cost_sim"] = opt_energy_sim
                csv_rs["opt_dist_sim"] = opt_dist_sim
                csv_rs["pp_cost_model"] = pp_energy_model
                csv_rs["pp_dist_model"] = pp_dist_model
                csv_rs["pp_cost_sim"] = pp_energy_sim
                csv_rs["pp_dist_sim"] = pp_dist_sim
                csv_rs["ratio_opt_model"] = csv_rs["opt_cost_model"] / csv_rs["opt_dist_model"]
                csv_rs["ratio_opt_sim"] = csv_rs["opt_cost_sim"] / csv_rs["opt_dist_sim"]
                csv_rs["ratio_pp_model"] = csv_rs["pp_cost_model"] / csv_rs["pp_dist_model"]
                csv_rs["ratio_pp_sim"] = csv_rs["pp_cost_sim"] / csv_rs["pp_dist_sim"]
                csv_rs["wind_direction"] = wds
                ff_name = "result/result_ds" + str(ds) + "_ws" + str(ws) + "_c" + str(
                        cl) + ".csv"
                csv_rs.to_csv(ff_name)
    return