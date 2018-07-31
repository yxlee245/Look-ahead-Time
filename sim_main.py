### Main code
### for modelling and simulation for optimal look-ahead time

## Import libraries, settings, and functions from external file
from sim	_setup_functions import *

## Encase process in while loop
while True:
    # Get inputs
	p1_start_lat = float(input('Enter starting latitude for aircraft 1: '))
	p1_start_long = float(input('Enter starting longitude for aircraft 1: '))
	p2_start_lat = float(input('Enter starting latitude for aircraft 2: '))
	p2_start_long = float(input('Enter starting longitude for aircraft 2: '))
	p_final_lat = float(input('Enter final latitude for both aircraft: '))
	p_final_long = float(input('Enter final longitude for both aircraft: '))
	T = float(input('Enter time period of study, in min: '))
	num_step = int(input('Enter no of time steps for each trajectory: '))
	v1_nom = float(input('Enter speed of aircraft 1, in kts: '))
	v2_nom = float(input('Enter speed of aircraft 2, in kts: '))
	error_along_sd = float(input('Enter SD of along track error, in NM/min: '))
	error_cross_sd = float(input('Enter SD of cross track error, in NM: '))
	N = int(input('Enter no of runs for Monte-Carlo simulation: ))
	num_init = int(input('Enter no of initial points (no of Tc values): '))
	sep = float(input('Enter required separation minimum, in NM: '))
    try:
	    # Run simulation
		results_df = run_montecarlo(p1_start_lat, p1_start_long, p2_start_lat, p2_start_long,
		p_final_lat, p_final_long,
		T, num_step, v1_nom, v2_nom,
		error_along_sd, error_cross_sd, sep, N, num_init)
		break
	except TypeError:
	    print('Wrong input format, please try again')
		
## Save results in CSV format
results_filename = input('Enter filename in CSV format for results: ')
results_df.to_csv(results_filename, header=True, index_label=False, index=False)