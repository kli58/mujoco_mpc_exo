import os
import json

def remove_unlisted_files(log_dir, json_file):
    with open(json_file, 'r') as file:
        trials = json.load(file)
    
    # Extract the list of log files from the JSON data
    listed_files = [entry['parameters']['log_file'] for entry in trials]
    
    # Get the list of files in the log directory
    all_files = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if os.path.isfile(os.path.join(log_dir, f))]
    
    # Exclude trial_log.json from all_files
    all_files = [f for f in all_files if os.path.basename(f) != os.path.basename(json_file)]
    
    # Determine the files to be removed (not in the listed_files)
    files_to_remove = set(all_files) - set(listed_files)
    
    # Remove the files that are not listed
    for file in files_to_remove:
        print(f"Removing file: {file}")
        os.remove(file)

if __name__ == '__main__':
    log_dir = '/home/amy/gitrepo/mujoco_mpc_exo/mjpc/tasks/exo/sim_data'
    json_file = os.path.join(log_dir, 'trial_log.json')
    
    remove_unlisted_files(log_dir, json_file)
