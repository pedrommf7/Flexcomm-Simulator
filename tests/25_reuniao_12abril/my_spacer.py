#!/usr/bin/env python3

import sys
import os

def filter_lines_with_ApplyRoutingFromPath(file_path):
    lines_to_keep = []

    with open(file_path, 'r') as file:
        for line in file:
            # Check if the line contains the word "ApplyRoutingFromPath"
            if 'ApplyRoutingFromPath' in line:
                lines_to_keep.append(line)

    # Write the filtered lines back to the file
    with open(file_path, 'w') as file:
        file.writelines(lines_to_keep)

def insert_newline_inplace(file_path):
    prev_number = None
    lines_to_write = []

    with open(file_path, 'r') as file:
        for line in file:
            # Extract the number before the first space
            number = line.split(' ')[0]

            # If it's different from the previous one, insert a newline
            if prev_number is not None and prev_number != number:
                lines_to_write.append('\n')

            # Add the line to the list of lines to write
            lines_to_write.append(line)

            # Update the previous number
            prev_number = number

    # Write the modified lines back to the file
    with open(file_path, 'w') as file:
        file.writelines(lines_to_write)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Usage: {} <output_file_path>'.format(sys.argv[0]))
        sys.exit(1)
    
    sortedFilename = 'sorted_' + sys.argv[1]
    
    os.system('sort {} -n -o {}'.format(sys.argv[1], sortedFilename))

    filter_lines_with_ApplyRoutingFromPath(sortedFilename)

    insert_newline_inplace(sortedFilename)