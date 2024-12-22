from bs4 import BeautifulSoup
import csv
import os

def main(html_file_path):

    # Read the HTML content from the file
    with open(html_file_path, 'r', encoding='utf-8') as file:
        html_content = file.read()

    # Parse the HTML content with BeautifulSoup
    soup = BeautifulSoup(html_content, 'html.parser')

    # Find the table containing the data
    table = soup.find('table')

    # Extract the header (column names), excluding the first column
    header = [
        th.text.strip() for th in table.find('thead').find('tr').find_all('th')[1:]
    ]

    # Extract the rows of data, excluding the first column in each row
    rows = []
    for tr in table.find_all('tr')[1:]:  # Skip the header row
        row = [td.text.strip() for td in tr.find_all('td')[1:]]
        rows.append(row)

    # Write to CSV file
    with open('output.csv', 'a', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(header)
        csv_writer.writerows(rows)

#remove the file if it exists
if os.path.exists('output.csv'):
    os.remove('output.csv')
for i in range(1, 15):
    html_file_path = f'file{i}.txt'
    main(html_file_path)

print("CSV file created successfully.")
