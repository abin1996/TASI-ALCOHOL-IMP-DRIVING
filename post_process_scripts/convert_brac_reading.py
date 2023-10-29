import pandas as pd
import ast
import datetime
#Read the Brac sensor csv file from the input path and delete all rows except the one with non value for signal column
#This is done to remove all the rows with no signal value
def filter_brac_sensor_and_save(input_path, output_path):
    df = pd.read_csv(input_path)
    df['signals'] = df['signals'].apply(ast.literal_eval)
    df = df[df['signals'].apply(lambda x: len(x) > 2 and x[5][1] != 0.0)]
    #Remove all sublists in the signals column except the 6th sublist
    df['signals'] = df['signals'].apply(lambda x: x[5])
    #Make the second element of list of the signals column as the value of the signals column
    df['signals'] = df['signals'].apply(lambda x: x[1])
    #Rename the signals column to BRAC Value
    df.rename(columns={'signals':'brac_value'}, inplace=True)
    #Add a new column which has two values, first reading for the first row and second reading
    df['measurement number'] = ['first toyota brac reading', 'second toyota brac reading']
    df = df.pivot(columns='measurement number', values='brac_value').bfill().iloc[[0],:]
    df.columns = ['first toyota brac reading', 'second toyota brac reading']
    df['time'] = [datetime.datetime.now().strftime('%d-%m-%y_%H:%M:%S.%f')]
    df['before or after'] = ['before']
    df['subject id'] = ['subject01']
    df['session name'] = ['80-Alcohol']
    df['ground truth'] = ['fill reading here']
    #Rename the columns to first reading and .second reading
    print(df)
    df.to_csv(output_path,columns=['time', 'before or after', 'first toyota brac reading', 'second toyota brac reading','ground truth'], index=False)

if __name__ == '__main__':
    input_path = "/home/iac_user/data_collection_scripts/brac_test/brac_16-10-23_18-58-43.csv"
    output_path = "brac_sensor_processed.csv"
    filter_brac_sensor_and_save(input_path, output_path)
