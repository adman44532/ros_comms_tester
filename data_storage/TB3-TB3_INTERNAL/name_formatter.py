import os

def rename_files_in_directory():
    # Get the current directory
    current_directory = os.getcwd()
    
    # Iterate over all the files in the current directory
    for filename in os.listdir(current_directory):
        # Only process files (not directories)
        if os.path.isfile(filename):
            # Replace all colons ':' with underscores '_'
            new_filename = filename.replace(':', '_')
            
            # If the filename was changed, rename the file
            if new_filename != filename:
                os.rename(filename, new_filename)
                print(f"Renamed: {filename} -> {new_filename}")

if __name__ == "__main__":
    rename_files_in_directory()

