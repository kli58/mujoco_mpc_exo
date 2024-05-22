import pandas as pd
import matplotlib.pyplot as plt
import os
import json

def load_log_data(file_path):
    data = pd.read_csv(file_path, names=[
        'time', 'userdata0', 'userdata1', 
        'subtree_com0', 'subtree_com1', 'subtree_com2',
        'stancefootPos0', 'stancefootPos1', 'stancefootPos2',
        'force_norm0', 'force_norm1', 'torque_norm0', 'torque_norm1'
    ])
    return data

def get_end_time(log_entries):
    end_times = []
    param_combinations = []
    for entry in log_entries:
        log_file = entry['parameters']['log_file']
        nominal_policy = entry['parameters']['nominal_policy']
        xfrc_mean = entry['parameters']['xfrc_mean']
        param_combination = {'nominal_policy': nominal_policy, 'xfrc_mean': xfrc_mean}
        file_path = log_file
        data = load_log_data(file_path)
        end_time = data.iloc[-1]['time']
        end_times.append(end_time)
        param_combinations.append(json.dumps(param_combination))  # Convert param combination to string for easier comparison
    return end_times, param_combinations

def aggregate_summary_data(param_combinations, end_times):
    data = pd.DataFrame({'param_combination': param_combinations, 'end_time': end_times})
    aggregated_data = data.groupby('param_combination').agg(
        mean_end_time=('end_time', 'mean'),
        std_end_time=('end_time', 'std')
    ).reset_index()
    breakpoint()
    return aggregated_data

def save_aggregated_data(aggregated_data, file_path):
    aggregated_data.to_csv(file_path, index=False)

def plot_metric(ax, data, metric_mean, metric_std, title, ylabel):
    ax.errorbar(data.index, data[metric_mean], yerr=data[metric_std], fmt='o', label=data.index)
    ax.set_xlabel('Parameter Combination')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.tick_params(axis='x', rotation=45)
    ax.legend()

def plot_aggregated_data(aggregated_data, parameter_info, save_path=None):
    fig, ax = plt.subplots(figsize=(12, 6))

    # Shorten param_combination labels
    aggregated_data['short_label'] = aggregated_data['param_combination'].apply(lambda x: f'Trial {aggregated_data["param_combination"].tolist().index(x) + 1}')
    aggregated_data.set_index('short_label', inplace=True)

    plot_metric(ax, aggregated_data, 'mean_end_time', 'std_end_time', 'Mean Simulation End Time with Standard Deviation', 'Simulation End Time')

    # Add a custom legend for trial parameters
    handles, labels = ax.get_legend_handles_labels()
    param_labels = [f'Trial {i+1}: {param_info}' for i, param_info in enumerate(parameter_info)]
    fig.legend(handles, param_labels, loc='upper right')

    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path)
    plt.show()

if __name__ == '__main__':
    log_dir = '/home/amy/gitrepo/mujoco_mpc_exo/mjpc/tasks/exo/sim_data'
    trial_log_path = os.path.join(log_dir, 'trial_log.json')
    aggregated_file_path = os.path.join(log_dir, 'aggregated_results.csv')
    plot_save_path = os.path.join(log_dir, 'summary_plot.png')

    with open(trial_log_path, 'r') as file:
        trial_log = json.load(file)

    end_times, param_combinations = get_end_time(trial_log)
    aggregated_data = aggregate_summary_data(param_combinations, end_times)
    save_aggregated_data(aggregated_data, aggregated_file_path)

    breakpoint()
    # Extract parameter info for the legend
    parameter_info = [json.dumps({'nominal_policy': entry['parameters']['nominal_policy'], 'xfrc_mean': entry['parameters']['xfrc_mean']}) for entry in trial_log]

    plot_aggregated_data(aggregated_data, parameter_info, save_path=plot_save_path)
