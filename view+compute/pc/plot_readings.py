import pandas as pd
import io
import matplotlib.pyplot as plt

READINGS_FILE = "spo2_readings_table_jpeg.csv"

# Read the data into a pandas DataFrame
# If you were reading from a file named 'data.csv', you would use:
# df = pd.read_csv('data.csv')
df = pd.read_csv(READINGS_FILE)

# --- Plotting Code ---

# Select the column to plot
column_to_plot = 'Ratio Red'

# Calculate the mean of the column
mean_value = df[column_to_plot].mean()

# Create a figure and axis
plt.figure(figsize=(10, 6))

# Plot the individual 'Ratio Red' values
# We use the DataFrame's index for the x-axis
plt.plot(df.index, df[column_to_plot], marker='o', linestyle='--', label=f'{column_to_plot} Values')

# Plot the mean line
# plt.axhline draws a horizontal line across the plot
plt.axhline(y=mean_value, color='red', linestyle='-', linewidth=2,
            label=f'Mean = {mean_value:.4f}')

# Add titles and labels
plt.title(f'Plot of {column_to_plot} Values and their Mean')
plt.xlabel('Data Point Index')
plt.ylabel(column_to_plot)

# Add a legend to identify the plots
plt.legend()

# Add a grid for easier reading
plt.grid(True)

# Display the plot
# In many Python environments (like Jupyter or a local script),
# plt.show() will open a window with your plot.
# If saving to a file, you would use:
# plt.savefig('ratio_red_plot.png')
plt.show()