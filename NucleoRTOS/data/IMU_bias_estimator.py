# I have data in the format:
# gyr_x:-0.893129766
# gyr_y:1.106870294
# gyr_z:1.923664093
# acc_x:-0.796630859
# acc_y:-0.172839507
# acc_z:-0.522949219

import numpy as np
import matplotlib.pyplot as plt
import sys
from scipy import stats

# Read file contents into gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z lists
def read_file(file_name):
    gyr_x = []
    gyr_y = []
    gyr_z = []
    acc_x = []
    acc_y = []
    acc_z = []
    with open(file_name, 'r') as file:
        lines = file.readlines()
        for line in lines:
            line = line.strip()
            if line.startswith('gyr_x:'):
                gyr_x.append(float(line.split(':')[1]))
            elif line.startswith('gyr_y:'):
                gyr_y.append(float(line.split(':')[1]))
            elif line.startswith('gyr_z:'):
                gyr_z.append(float(line.split(':')[1]))
            elif line.startswith('acc_x:'):
                acc_x.append(float(line.split(':')[1]))
            elif line.startswith('acc_y:'):
                acc_y.append(float(line.split(':')[1]))
            elif line.startswith('acc_z:'):
                acc_z.append(float(line.split(':')[1]))
    return gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z

# Calculate the mean and std deviation for each axis
def calculate_stats(data):
    mean = np.mean(data)
    std_dev = np.std(data)
    median = np.median(data)
    return mean, median, std_dev

# Plot the data as a histogram with the mean and std deviation
# def plot_histogram(data, mean, std_dev, title):
#     plt.hist(data, bins='auto', color='chocolate', edgecolor='black')
#     plt.axvline(mean, color='darkorange', linewidth=2)
#     plt.axvline(mean + std_dev, color='tomato', linewidth=2)
#     plt.axvline(mean - std_dev, color='tomato', linewidth=2)
#     plt.title(title)
#     plt.show()

# A single plot function that plots the histogram for each axis in a separate matplotlib axis
def plot_histograms(gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z):
    fig, axs = plt.subplots(2,3, figsize=(20, 10))
    # Plot the normalized histogram data
    axs[0,0].hist(gyr_x, bins='auto', color='lightgray', edgecolor='darkgray', density=True)
    axs[0, 0].axvline(np.mean(gyr_x), color='darkorange', linewidth=2)
    axs[0, 0].axvline(np.median(gyr_x), color='red', linewidth=1)
    axs[0, 0].axvline(np.mean(gyr_x) + np.std(gyr_x), color='tomato', linewidth=2)
    axs[0, 0].axvline(np.mean(gyr_x) - np.std(gyr_x), color='tomato', linewidth=2)
    axs[0, 0].set_title('gyr_x')
    # Plot gaussian curve
    x = np.linspace(np.min(gyr_x), np.max(gyr_x), 100)
    axs[0, 0].plot(x, stats.norm.pdf(x, np.mean(gyr_x), np.std(gyr_x)), color='chocolate')

    axs[0, 1].hist(gyr_y, bins='auto', color='lightgray', edgecolor='darkgray', density=True)
    axs[0, 1].axvline(np.mean(gyr_y), color='darkorange', linewidth=2)
    axs[0, 1].axvline(np.median(gyr_y), color='red', linewidth=1)
    axs[0, 1].axvline(np.mean(gyr_y) + np.std(gyr_y), color='tomato', linewidth=2)
    axs[0, 1].axvline(np.mean(gyr_y) - np.std(gyr_y), color='tomato', linewidth=2)
    axs[0, 1].set_title('gyr_y')
    # Plot gaussian curve
    x = np.linspace(np.min(gyr_y), np.max(gyr_y), 100)
    axs[0, 1].plot(x, stats.norm.pdf(x, np.mean(gyr_y), np.std(gyr_y)), color='chocolate')


    axs[0, 2].hist(gyr_z, bins='auto', color='lightgray', edgecolor='darkgray', density=True)
    axs[0, 2].axvline(np.mean(gyr_z), color='darkorange', linewidth=2)
    axs[0, 2].axvline(np.median(gyr_z), color='red', linewidth=1)
    axs[0, 2].axvline(np.mean(gyr_z) + np.std(gyr_z), color='tomato', linewidth=2)
    axs[0, 2].axvline(np.mean(gyr_z) - np.std(gyr_z), color='tomato', linewidth=2)
    axs[0, 2].set_title('gyr_z')
    # Plot gaussian curve
    x = np.linspace(np.min(gyr_z), np.max(gyr_z), 100)
    axs[0, 2].plot(x, stats.norm.pdf(x, np.mean(gyr_z), np.std(gyr_z)), color='chocolate')

    axs[1, 0].hist(acc_x, bins='auto', color='lightgray', edgecolor='darkgray', density=True)
    axs[1, 0].axvline(np.mean(acc_x), color='darkorange', linewidth=2)
    axs[1, 0].axvline(np.median(acc_x), color='red', linewidth=1)
    axs[1, 0].axvline(np.mean(acc_x) + np.std(acc_x), color='tomato', linewidth=2)
    axs[1, 0].axvline(np.mean(acc_x) - np.std(acc_x), color='tomato', linewidth=2)
    axs[1, 0].set_title('acc_x')
    # Plot gaussian curve
    x = np.linspace(np.min(acc_x), np.max(acc_x), 100)
    axs[1, 0].plot(x, stats.norm.pdf(x, np.mean(acc_x), np.std(acc_x)), color='chocolate')

    axs[1, 1].hist(acc_y, bins='auto', color='lightgray', edgecolor='darkgray', density=True)
    axs[1, 1].axvline(np.mean(acc_y), color='darkorange', linewidth=2)
    axs[1, 1].axvline(np.median(acc_y), color='red', linewidth=1)
    axs[1, 1].axvline(np.mean(acc_y) + np.std(acc_y), color='tomato', linewidth=2)
    axs[1, 1].axvline(np.mean(acc_y) - np.std(acc_y), color='tomato', linewidth=2)
    axs[1, 1].set_title('acc_y')
    # Plot gaussian curve
    x = np.linspace(np.min(acc_y), np.max(acc_y), 100)
    axs[1, 1].plot(x, stats.norm.pdf(x, np.mean(acc_y), np.std(acc_y)), color='chocolate')


    axs[1, 2].hist(acc_z, bins='auto', color='lightgray', edgecolor='darkgray', density=True)
    axs[1, 2].axvline(np.mean(acc_z), color='darkorange', linewidth=2)
    axs[1, 2].axvline(np.median(acc_z), color='red', linewidth=1)
    axs[1, 2].axvline(np.mean(acc_z) + np.std(acc_z), color='tomato', linewidth=2)
    axs[1, 2].axvline(np.mean(acc_z) - np.std(acc_z), color='tomato', linewidth=2)
    axs[1, 2].set_title('acc_z')
    # Plot gaussian curve
    x = np.linspace(np.min(acc_z), np.max(acc_z), 100)
    axs[1, 2].plot(x, stats.norm.pdf(x, np.mean(acc_z), np.std(acc_z)), color='chocolate')

    plt.show()

    

# Run program
def main():
    # Read file contents
    file_name = sys.argv[1]
    gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z = read_file(file_name)


    # Calculate mean and std deviation for each axis
    gyr_x_mean, gyr_x_median, gyr_x_std_dev = calculate_stats(gyr_x)
    gyr_y_mean, gyr_y_median, gyr_y_std_dev = calculate_stats(gyr_y)
    gyr_z_mean, gyr_z_median, gyr_z_std_dev = calculate_stats(gyr_z)
    acc_x_mean, acc_x_median, acc_x_std_dev = calculate_stats(acc_x)
    acc_y_mean, acc_y_median, acc_y_std_dev = calculate_stats(acc_y)
    acc_z_mean, acc_z_median, acc_z_std_dev = calculate_stats(acc_z)

    # Print the mean and std deviation for each axis
    print('gyr_x mean: ', gyr_x_mean)
    print('gyr_x median: ', gyr_x_median)
    print('gyr_x std deviation: ', gyr_x_std_dev)
    print('gyr_y mean: ', gyr_y_mean)
    print('gyr_y median: ', gyr_y_median)
    print('gyr_y std deviation: ', gyr_y_std_dev)
    print('gyr_z mean: ', gyr_z_mean)
    print('gyr_z median: ', gyr_z_median)
    print('gyr_z std deviation: ', gyr_z_std_dev)
    print('acc_x mean: ', acc_x_mean)
    print('acc_x median: ', acc_x_median)
    print('acc_x std deviation: ', acc_x_std_dev)
    print('acc_y mean: ', acc_y_mean)
    print('acc_y median: ', acc_y_median)
    print('acc_y std deviation: ', acc_y_std_dev)
    print('acc_z mean: ', acc_z_mean)
    print('acc_z median: ', acc_z_median)
    print('acc_z std deviation: ', acc_z_std_dev)

    # Plot the data
    # plot_histogram(gyr_x, gyr_x_mean, gyr_x_std_dev, 'gyr_x')
    # plot_histogram(gyr_y, gyr_y_mean, gyr_y_std_dev, 'gyr_y')
    # plot_histogram(gyr_z, gyr_z_mean, gyr_z_std_dev, 'gyr_z')
    # plot_histogram(acc_x, acc_x_mean, acc_x_std_dev, 'acc_x')
    # plot_histogram(acc_y, acc_y_mean, acc_y_std_dev, 'acc_y')

    plot_histograms(gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z)


if __name__ == '__main__':
    main()