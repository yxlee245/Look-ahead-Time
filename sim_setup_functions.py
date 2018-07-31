### Libraries, Settings and Functions
### for modelling and simulation for optimal look-ahead time

## Libraries and Setting
import numpy as np
import random, math
from matplotlib import pyplot as plt
import pandas as pd
%matplotlib inline

## Models for Uncertainty Paramters

# Along track position error
def error_along(sd, t):
    """
    sd - Standard devation in NM/min
    t - time in min
    Returns along track speed error in kts (float)
    """
    return random.normalvariate(0, sd * t)
	
# Cross track position error
def error_cross(sd):
    """
    sd - Standard deviation in NM
    Returns Cross track position error in NM (float)
    """
    return random.normalvariate(0, sd)
	
## User-defined functions to support simulation
# Function to compute heading
def heading(p_init_lat, p_init_long, p_final_lat, p_final_long):
    '''
    p_init_lat - Initial latitudinal position in NM
    p_init_long - Initial longitudinal position in NM
    p_final_lat - Final latitudinal position in NM
    p_final_long - Final longitudinal position in NM
    Returns the heading in radians (float)
    '''
    theta = math.atan2((p_final_long - p_init_long), (p_final_lat - p_init_lat))
    return theta

# Generate trajectory dataframe for aircraft
def generate_trajectory(p_init_lat, p_init_long, v_nominal, T, num_step, heading, error_along_sd, error_cross_sd):
    """
    p_init_lat - Initial latitudinal position in NM
    p_init_long - Initial longitudinal position in NM
    v_nominal - nominal speed in knots
    T - time period of study, in minutes
    num_step - Number of time steps for the simulation
    heading - heading in radians
    error_along_sd - along track sped error in in NM/min
    error_cross_sd - cross track error in NM
    Returns a pandas dataframe containing time, latitudinal and longitudinal positions of nominal trajectory
    """
    time_array = np.linspace(0, T, num_step)
    time_step_float = float(time_array[1] - time_array[0])
    # Generate nominal trajectory
    lat_nominal_list = []
    long_nominal_list = []
    for t in time_array:
        lat_nom_float = p_init_lat + v_nominal / 60 * t * math.cos(heading)
        long_nom_float = p_init_long + v_nominal / 60 * t * math.sin(heading)
        lat_nominal_list.append(lat_nom_float)
        long_nominal_list.append(long_nom_float)
    trajectory_df = pd.DataFrame({
        'Time': time_array,
        'Nom_Lat': lat_nominal_list,
        'Nom_Long': long_nominal_list
    }, columns=['Time', 'Nom_Lat', 'Nom_Long'])
    # Generate stochastic trajectory with only speed errors taken into account
    lat_stochastic1_list = []
    long_stochastic1_list = []
    for i, t in enumerate(trajectory_df.Time):
        # Step-wise simulation of trajectory with speed error
        if i == 0:
            lat_sto_float = trajectory_df.Nom_Lat[i]
            long_sto_float = trajectory_df.Nom_Long[i]
        else:
            error_along_float = error_along(error_along_sd, t)
            lat_sto_float = lat_stochastic1_list[i - 1] + v_nominal / 60 * math.cos(heading) * time_step_float + \
            error_along_float * math.cos(heading)
            long_sto_float = long_stochastic1_list[i - 1] + v_nominal / 60 * math.sin(heading) * time_step_float + \
            error_along_float * math.sin(heading)
        
        #lat_sto_float = trajectory_df.Nom_Lat[i] + error_along(error_along_sd, t) * math.cos(heading) + \
        #error_cross(error_cross_sd) * math.sin(heading)
        #long_sto_float = trajectory_df.Nom_Long[i] + error_along(error_along_sd, t) * math.sin(heading) - \
        #error_cross(error_cross_sd) * math.cos(heading)
        lat_stochastic1_list.append(lat_sto_float)
        long_stochastic1_list.append(long_sto_float)
    # Generate and add cross track errors to trajectories
    cross_error_list = [error_cross(error_cross_sd) for i in trajectory_df.index]
    lat_crosserror_list = [cross_error_list[i] * math.sin(heading) for i in trajectory_df.index]
    long_crosserror_list = [cross_error_list[i] * math.cos(heading) for i in trajectory_df.index]
    
    lat_stochastic2_array = np.array(lat_stochastic1_list) + np.array(lat_crosserror_list)
    long_stochastic2_array = np.array(long_stochastic1_list) - np.array(long_crosserror_list)
    
    
    trajectory_df['Sto_Lat'] = lat_stochastic2_array
    trajectory_df['Sto_Long'] = long_stochastic2_array
    return trajectory_df

# Estimate time to predicted conflict from nominal trajectories
def tc_estimate(trajectory1, trajectory2, sep):
    '''
    trajectory1 - Trajectory of aircraft 1 in pandas dataframe format
    trajectory2 - Trajectory of aircraft 2 in pandas dataframe format
    sep - Required minimum separation in NM
    Returns the estimated time to predicted conflict in minutes (float)
    '''
    time_array = trajectory1.Time
    # Finding distance using sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    diff_mag = np.sqrt((trajectory2.Nom_Lat - trajectory1.Nom_Lat) ** 2 + (trajectory2.Nom_Long - trajectory1.Nom_Long) ** 2)
    
    index_conflict = 0
    for i in range(len(diff_mag)):
        if diff_mag[i] < sep:
            index_conflict = i
            break
    
    if index_conflict == 0:
        tc = 0
    else:
        tc = time_array[index_conflict]
        
    return tc

# Detect conflict in stochastic trajectory
def detect_conflict(trajectory1, trajectory2, sep):
    '''
    trajectory1 - Trajectory of aircraft 1 in pandas dataframe format
    trajectory2 - Trajectory of aircraft 2 in pandas dataframe format
    sep - Required minimum separation in NM
    Returns an indicator for conflict (0 or 1) (integer)
    '''
    time_array = trajectory1.Time
    # Finding distance using sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    diff_mag = np.sqrt((trajectory2.Sto_Lat - trajectory1.Sto_Lat) ** 2 + (trajectory2.Sto_Long - trajectory1.Sto_Long) ** 2)
    
    conflict_indicator = 0
    for i in range(len(diff_mag)):
        if diff_mag[i] < sep:
            conflict_indicator = 1
            break
            
    return conflict_indicator
	
# Wrapper function for one simulation run
def run_simulation(p1_init_lat, p1_init_long, p2_init_lat, p2_init_long,
                   p_final_lat, p_final_long, T, num_step, v1_nom, v2_nom,
                   error_along_sd, error_cross_sd, sep):
    '''
    p1_init_lat - Initial latitudinal position of aircraft 1 in NM
    p1_init_long - Initial longitudinal position of aircraft 1 in NM
    p2_init_lat - Initial latitudinal position of aircraft 1 in NM
    p2_init_long - Initial longitudinal position of aircraft 1 in NM
    p_final_lat - Final latitudinal position in NM
    p_final_long - Final longitudinal position in NM
    T - time period of study, in minutes
    num_step - Number of time steps for the simulation
    v1_nom - Nominal speed of aircraft 1 in kts
    v2_nom - Nominal speed of aircraft 2 in kts
    error_along_sd - along track error rate in NM/min
    error_cross_sd - cross track error in NM
    sep - Required minimum separation in NM
    Returns time to predicted conflict in minutes and conflict probability (float)
    '''
    # Set headings
    heading_1 = heading(p1_init_lat, p1_init_long, p_final_lat, p_final_long)
    heading_2 = heading(p2_init_lat, p2_init_long, p_final_lat, p_final_long)
    # Generate trajectories for aircrafts 1 and 2
    trajectory1_df = generate_trajectory(p1_init_lat, p1_init_long, v1_nom, T, num_step,
                                         heading_1, error_along_sd, error_cross_sd)
    trajectory2_df = generate_trajectory(p2_init_lat, p2_init_long, v2_nom, T, num_step,
                                         heading_2, error_along_sd, error_cross_sd)
    # Estimate tc from nominal trajectories
    tc_float = tc_estimate(trajectory1_df, trajectory2_df, sep)
    # Detect conflict in stochastic trajectories
    conflict_ind_int = detect_conflict(trajectory1_df, trajectory2_df, sep)
    
    return tc_float, conflict_ind_int
	
# Wrapper function for one scenario
def run_montecarlo(p1_start_lat, p1_start_long, p2_start_lat, p2_start_long, p_final_lat, p_final_long,
                  T, num_step, v1_nom, v2_nom, error_along_sd, error_cross_sd, sep, N, num_init):
    '''
    p1_start_lat - The latitudinal position of the starting point of the whole set of initial points for aircraft 1
    (the one that gives the largest Tc)
    p1_start_long - The longitudinal position of the starting point of the whole set of initial points for aircraft 1
    p2_start_lat - The latitudinal position of the starting point of the whole set of initial points for aircraft 2
    (the one that gives the largest Tc)
    p2_start_long - The longitudinal position of the starting point of the whole set of initial points for aircraft 2
    p_final_lat - Final latitudinal position in NM
    p_final_long - Final longitudinal position in NM
    T - time period of study, in minutes
    num_step - Number of time steps for the simulation
    v1_nom - Nominal speed of aircraft 1 in kts
    v2_nom - Nominal speed of aircraft 2 in kts
    error_along_sd - along track error rate in NM/min
    error_cross_sd - cross track error in NM
    sep - Required minimum separation in NM
    N - number of runs for Monte Carlo simulation
    num_init - number of initial points
    Returns a pandas dataframe containing Tc and Pc values for the scenario
    '''
    # Create list of initial latitudes and longitudes for initial points
    p1_init_lat_list = [p1_start_lat + i * (p_final_lat - p1_start_lat) / num_init for i in range(num_init)]
    p1_init_long_list = [p1_start_long + i * (p_final_long - p1_start_long) / num_init for i in range(num_init)]
    p2_init_lat_list = [p2_start_lat + i * (p_final_lat - p2_start_lat) / num_init for i in range(num_init)]
    p2_init_long_list = [p2_start_long + i * (p_final_long - p2_start_long) / num_init for i in range(num_init)]
    
    # Initialize tc and pc lists
    tc_list = []
    pc_list = []
    
    # Nested for loop through initial points and runs
    for p1_init_lat, p1_init_long, p2_init_lat, p2_init_long in \
    zip(p1_init_lat_list, p1_init_long_list, p2_init_lat_list, p2_init_long_list):
        conflict_indicator_list = []
        
        # Monte Carlo simulation
        for n in range(N):
            tc, conflict_indicator = run_simulation(p1_init_lat, p1_init_long, p2_init_lat, p2_init_long,
                                                    p_final_lat, p_final_long, T, num_step, v1_nom, v2_nom,
                                                    error_along_sd, error_cross_sd, sep)
            conflict_indicator_list.append(conflict_indicator)
        
        # Compute conflict probability
        pc_float = len([i for i in conflict_indicator_list if i == 1]) / len(conflict_indicator_list)
        tc_list.append(tc)
        pc_list.append(pc_float)
        
    # Create dataframe of results
    results_df = pd.DataFrame({
        'Tc': tc_list,
        'Pc': pc_list
    }, columns=['Tc', 'Pc'])
    
    # Sort dataframe by Tc
    results_sorted_df = results_df.sort_index(ascending=False)
    
    # Reset index
    results_sorted_df.reset_index(drop=True, inplace=True)
    
    return results_sorted_df
	
# Function to determine Tc at threshold
def optimal_search(results_df, threshold):
    for i in results_df.index:
        if results_df.Pc[i] < threshold:
            threshold_index_int = i
            break
    optimal_tc_float = results_df.Tc[threshold_index_int]
    return optimal_tc_float