import os
import time
import subprocess
import pandas as pd
import matplotlib.pyplot as plt
import yaml
import json
from datetime import datetime
from itertools import product

# Function to update the YAML file
def update_task_specification(yaml_file_path, updates):
    with open(yaml_file_path, 'r') as file:
        task_spec = yaml.safe_load(file)
    
    for key, value in updates.items():
        task_spec[key] = value
    
    with open(yaml_file_path, 'w') as file:
        yaml.dump(task_spec, file, default_flow_style=None)

# Function to start the build and simulation
def start_simulation(build_command, simulation_command):
    subprocess.run(build_command)
    subprocess.run(simulation_command)

# Function to load the logger data from the CSV file
def load_logger_data(file_path):
    data = pd.read_csv(file_path, names=[
        'time', 'userdata0', 'userdata1', 
        'subtree_com0', 'subtree_com1', 'subtree_com2',
        'stancefootPos0', 'stancefootPos1', 'stancefootPos2',
        'force_norm0', 'force_norm1', 'torque_norm0', 'torque_norm1'
    ])
    return data

# Function to save data to CSV
def save_logger_data(data, file_path):
    data.to_csv(file_path, index=False)

# Function to generate combinations of parameters
def generate_combinations(parameters):
    keys, values = zip(*parameters.items())
    return [dict(zip(keys, v)) for v in product(*values)]

# Function to log trial information
def log_trial_info(trial_log_path, trial_info):
    if os.path.exists(trial_log_path):
        with open(trial_log_path, 'r') as file:
            completed_trials = json.load(file)
    else:
        completed_trials = []
    
    completed_trials.append(trial_info)
    
    with open(trial_log_path, 'w') as file:
        json.dump(completed_trials, file, indent=4)

# Function to resume trials from where they left off
def resume_trials(trial_log_path):
    if os.path.exists(trial_log_path):
        with open(trial_log_path, 'r') as file:
            completed_trials = json.load(file)
    else:
        completed_trials = []
    completed_trials = [{k: v for k, v in trial['parameters'].items() if k != 'log_file'} for trial in completed_trials]
    
    
    return completed_trials

# Main function to run multiple trials
def run_trials(yaml_file_path, build_command, simulation_command, log_dir, parameter_combinations, seeds, trial_log_path):
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    completed_trials = resume_trials(trial_log_path)
    
    data_list = []

    for trial, param_combination in enumerate(parameter_combinations):
        for seed in seeds:
            param_combination['random_seed'] = seed
            trial_info = {'parameters': param_combination}
            
            param_only_trial_info = {k: v for k, v in trial_info['parameters'].items() if k != 'log_file'}

            if param_only_trial_info in completed_trials:
                # Skip the trial if it has already been completed
                # print paramters and seed
                
                print(f'Skipping trial {trial+1} with seed {seed}')
                continue
            
            # Define log file path for the current trial
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            log_file_path = os.path.join(log_dir, f'log_trial_{trial+1}_{seed}_{timestamp}.csv')
            
            # Update YAML file with the new log file path and parameter combination
            param_combination['log_file'] = log_file_path
            
            update_task_specification(yaml_file_path, param_combination)
            
            # Start simulation
            start_simulation(build_command, simulation_command)
            
            # Load logger data
            data = load_logger_data(log_file_path)
            data_list.append(data)
                        
            # Log the completed trial
            log_trial_info(trial_log_path, trial_info)
            
            # Optional: delay between trials
            time.sleep(1)
    


    return 


if __name__ == '__main__':
    # Define paths and parameters
    yaml_file_path = '/home/amy/gitrepo/mujoco_mpc_exo/mjpc/tasks/exo/task_specification.yaml'
    build_command = ['/usr/local/bin/cmake', '--build', '/home/amy/gitrepo/mujoco_mpc_exo/build', '--config', 'Release', '--target', 'all']
    simulation_command = ['/home/amy/gitrepo/mujoco_mpc_exo/build/bin/mjpc']
    log_dir = '/home/amy/gitrepo/mujoco_mpc_exo/mjpc/tasks/exo/sim_data'
    trial_log_path = os.path.join(log_dir, 'trial_log.json')

    # Define the parameters and their values for combination
    parameters = {
        'nominal_policy': [True, False],
        'xfrc_mean': [100,200],
    }

    # Define the list of random seeds
    seeds = [12345,42,456,789,101112,131415,161718,192021,222324,252627,
             282930,313233,343536,373839,404142,434445,464748,495051,525354,555657,
             585960,616263,646566,676869,707172,737475,767778,798081,828384,858687]

    # Generate all combinations of parameters
    parameter_combinations = generate_combinations(parameters)
    
    # Run trials and get the summary DataFrame
    run_trials(yaml_file_path, build_command, simulation_command, log_dir, parameter_combinations, seeds, trial_log_path)

