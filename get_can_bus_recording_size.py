import os
import sys

def main(subject):
    home_dir = os.getenv("HOME")
    data_collection_path = os.path.join(home_dir, "DATA_COLLECTION", subject)

    # Change the current working directory
    os.chdir(data_collection_path)

    # Find the latest record directory
    latest_rec = max(os.listdir(), key=lambda x: os.path.getctime(x))
    latest_rec_path = os.path.join(data_collection_path, latest_rec, "can")

    # Change the current working directory to the 'can' directory
    os.chdir(latest_rec_path)

    # List all CSV files and get their sizes
    csv_files = [f for f in os.listdir() if f.endswith('.csv')]
    file_sizes = {filename: os.path.getsize(filename) for filename in csv_files}

    # Print the file sizes
    for filename, size in file_sizes.items():
        print(f"size: {size}")
        print(f"filename: {filename}")
        break
    
if __name__ == "__main__":
    if len(sys.argv) != 2:
        # print("Usage: python script.py SUBJECT")
        sys.exit(1)

    subject = sys.argv[1]

    main(subject)
