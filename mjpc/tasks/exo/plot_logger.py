import pandas as pd
import matplotlib.pyplot as plt

# Function to load data from the CSV file
def load_logger_data(file_path):
    # Load the data into a pandas DataFrame
    data = pd.read_csv(file_path, names=[
        'time', 'userdata0', 'userdata1', 
        'subtree_com0', 'subtree_com1', 'subtree_com2',
        'stancefootPos0', 'stancefootPos1', 'stancefootPos2'
    ])
    return data

# Function to plot the logger data
def plot_logger_data(data):
    plt.figure(figsize=(12, 15))
    
    # Plot userdata[0] and userdata[1] over time
    plt.subplot(5, 1, 1)
    plt.plot(data['time'], data['userdata0'], label='userdata[0]')
    plt.plot(data['time'], data['userdata1'], label='userdata[1]')
    plt.xlabel('Time')
    plt.ylabel('User Data')
    plt.legend()
    plt.title('User Data vs Time')
    
    # Plot subtree_com[0] over time
    plt.subplot(5, 1, 2)
    plt.plot(data['time'], data['subtree_com0'], label='subtree_com[0]')
    plt.xlabel('Time')
    plt.ylabel('subtree_com[0]')
    plt.legend()
    plt.title('Subtree COM[0] vs Time')
    
    # Plot subtree_com[1] over time
    plt.subplot(5, 1, 3)
    plt.plot(data['time'], data['subtree_com1'], label='subtree_com[1]')
    plt.xlabel('Time')
    plt.ylabel('subtree_com[1]')
    plt.legend()
    plt.title('Subtree COM[1] vs Time')
    
    # Plot subtree_com[2] over time
    plt.subplot(5, 1, 4)
    plt.plot(data['time'], data['subtree_com2'], label='subtree_com[2]')
    plt.xlabel('Time')
    plt.ylabel('subtree_com[2]')
    plt.legend()
    plt.title('Subtree COM[2] vs Time')
    
    # Plot stancefootPos[0], stancefootPos[1], and stancefootPos[2] over time
    plt.subplot(5, 1, 5)
    plt.plot(data['time'], data['stancefootPos0'], label='stancefootPos[0]')
    plt.plot(data['time'], data['stancefootPos1'], label='stancefootPos[1]')
    plt.plot(data['time'], data['stancefootPos2'], label='stancefootPos[2]')
    plt.xlabel('Time')
    plt.ylabel('Stance Foot Position')
    plt.legend()
    plt.title('Stance Foot Position vs Time')
    
    plt.tight_layout()
    plt.show()


# Main function
def main():
    file_path = '/home/amy/gitrepo/mujoco_mpc_exo/mjpc/tasks/exo/sim_data/data.csv'  # Replace with the path to your CSV file
    data = load_logger_data(file_path)
    plot_logger_data(data)

if __name__ == '__main__':
    main()
