import os

# Define the main folder path
main_folder_path = "/home/iac_user/POST_PROCESS/SubjectAnn/70-Alcohol/Parking"  # Replace with the path to your main folder

# Define the prefix you want to add
prefix = "1"  # Replace with your desired prefix

def add_prefix_to_subfolders(folder_path, prefix):

    # Walk through the directory tree starting from the main folder
    for root, dirs, files in os.walk(folder_path):
        for file in files:
            # Construct the full path to the file
            file_path = os.path.join(root, file)

            # Create the new file name with the added prefix
            new_file_name = prefix + file

            # Construct the full path to the new file
            new_file_path = os.path.join(root, new_file_name)

            # Rename the file with the added prefix
            os.rename(file_path, new_file_path)

    # Rename the current folder with the added prefix
    new_folder_name = prefix + os.path.basename(folder_path)
    new_folder_path = os.path.join(os.path.dirname(folder_path), new_folder_name)
    os.rename(folder_path, new_folder_path)

    # Recursively process subfolders
    for item in os.listdir(new_folder_path):
        item_path = os.path.join(new_folder_path, item)
        if os.path.isdir(item_path):
            add_prefix_to_subfolders(item_path, prefix)

# Call the function to add the prefix to subfolders
add_prefix_to_subfolders(main_folder_path, prefix)


print(f"Added the prefix '{prefix}' to all file names in the main folder and its subfolders.")