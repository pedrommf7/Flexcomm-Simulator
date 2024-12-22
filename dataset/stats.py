import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
csv_file_path = 'output.csv'  # Specify the path to your CSV file
df = pd.read_csv(csv_file_path)

# Remove " w" suffix and convert the columns to numeric
df['Normal'] = pd.to_numeric(df['Normal'].str.rstrip(' w'), errors='coerce')
df['Max'] = pd.to_numeric(df['Max'].str.rstrip(' w'), errors='coerce')

# Calculate mean, minimum, and maximum for "Normal" column
normal_mean = round(df['Normal'].mean(), 2)
normal_min = df['Normal'].min()
normal_max = df['Normal'].max()

# Calculate mean, minimum, and maximum for "Max" column
max_mean = round(df['Max'].mean(), 2)
max_min = df['Max'].min()
max_max = df['Max'].max()

# Display the results
print(f"Normal Column: Mean={normal_mean}, Min={normal_min}, Max={normal_max}")
print(f"Max Column: Mean={max_mean}, Min={max_min}, Max={max_max}\n\n\n")






# Plot scatter plots
plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.scatter(range(len(df['Normal'].dropna())), df['Normal'].dropna(), color='blue', alpha=0.7)
plt.title('Values in Normal Column')
plt.xlabel('Index')
plt.ylabel('Values')

plt.subplot(1, 2, 2)
plt.scatter(range(len(df['Max'].dropna())), df['Max'].dropna(), color='green', alpha=0.7)
plt.title('Values in Max Column')
plt.xlabel('Index')
plt.ylabel('Values')

plt.tight_layout()
plt.show()


