import pandas as pd
import tkinter as tk
from tkinter import filedialog

def process_file(file_path):
    # Load data from CSV
    data = pd.read_csv(file_path)

    # Filter the rows where Latitude and Longitude are both 1000
    valid_data = data[(data['Latitude'] != 1000) | (data['Longitude'] != 1000)].copy()

    # Re-index the time column, starting from the first valid row
    if not valid_data.empty:
        time_offset = valid_data.iloc[0]['time']
        valid_data['time'] = valid_data['time'] - time_offset

    # Replace all 0's with 0.00001
    valid_data = valid_data.replace(0, 0.00001)

    # Overwrite the original file with the filtered data
    valid_data.to_csv(file_path, index=False)

# Create the GUI
root = tk.Tk()
root.withdraw()  # Hide the main window

# Open file dialog to select the file
file_path = filedialog.askopenfilename(title="Select a CSV file", filetypes=[("CSV files", "*.csv")])

# Check if a file was selected
if file_path:
    process_file(file_path)
else:
    print("No file selected")
