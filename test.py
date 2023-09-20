# import matplotlib.pyplot as plt
#
# # Data for the wireless communication technologies
# technologies = ['Bluetooth', 'ZigBee', 'DSRC', 'WiFi      ', 'LTE-V', '5G']
# frequency = [2.4e9, 2.4e9, 5.9e9, 5.8e9, 5.9e9, 28e9]  # Frequency in Hz
# bandwidth = [1e6, 250e3, 54e6, 600e6, 1e9, 10e9]  # Bandwidth in bits/s
#
# # Create a logarithmic plot
# plt.figure(figsize=(10, 6))
# plt.scatter(frequency, bandwidth, color='b', marker='o')
# plt.xscale('log')
# plt.yscale('log')
#
# # Add labels for each point
# for i, tech in enumerate(technologies):
#     plt.annotate(tech, (frequency[i], bandwidth[i]), textcoords="offset points", xytext=(0,10), ha='center')
#
# # Set axis labels and title
# plt.xlabel('Frequency (Hz)')
# plt.ylabel('Bandwidth (bits/s)')
# plt.title('Wireless Communication Technologies')
#
# # Show the plot
# plt.grid(True, which="both", ls="--")
# plt.tight_layout()
# plt.show()