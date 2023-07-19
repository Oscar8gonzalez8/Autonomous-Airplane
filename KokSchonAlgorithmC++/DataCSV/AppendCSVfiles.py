import pandas as pd
import tkinter as tk
from tkinter import filedialog

def select_files():
    root = tk.Tk()
    root.withdraw()

    file_paths = filedialog.askopenfilenames(title="Select CSV files", filetypes=[("CSV files", "*.csv")])

    return file_paths

def combine_csv_files(file_paths):
    combined_data = None

    # Read and merge dataframes based on 'Time (s)' column
    for file_path in file_paths:
        data = pd.read_csv(file_path)
        if combined_data is None:
            combined_data = data
        else:
            combined_data = pd.merge(combined_data, data, on='Time (s)', how='outer')

    # Sort by 'Time (s)' after merging
    combined_data = combined_data.sort_values(by='Time (s)')

    # Fill in empty cells with the previous data until new data is received
    combined_data.fillna(method='ffill', inplace=True)

    # Removing rows with negative 'Time (s)'
    combined_data = combined_data[combined_data['Time (s)'] >= 0]

    return combined_data

def save_combined_data(combined_data):
    if combined_data is not None:
        root = tk.Tk()
        root.withdraw()
        
        directory = filedialog.askdirectory(title="Select directory to save the combined data")
        
        if directory:
            file_path = f"{directory}/PhoneData.csv"
            combined_data.to_csv(file_path, index=False)
            print(f"Combined data saved to {file_path}")
        else:
            print("Save operation was cancelled.")
    else:
        print("No data to save.")

if __name__ == "__main__":
    print("Select the CSV files to combine:")
    file_paths = select_files()

    if file_paths:
        combined_data = combine_csv_files(file_paths)
        print("Select the directory to save the combined data:")
        save_combined_data(combined_data)
    else:
        print("No files were selected.")
