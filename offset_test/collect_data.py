import serial
import csv
import os

PORT = "/dev/cu.usbserial-5A6D0173541"
BAUD_RATE = 115200
SAMPLES_PER_TRIAL = 100

DATA_FOLDER = "offset_data"
os.makedirs(DATA_FOLDER, exist_ok=True)

current_trial = None
csv_file = None
writer = None

with serial.Serial(PORT, BAUD_RATE, timeout=1) as ser:

    print("Waiting for data...")

    while True:

        line = ser.readline().decode("utf-8", errors="ignore").strip()

        if not line:
            continue

        data = line.split(",")

        # Only save offset-test data
        if len(data) != 6 or data[0] != "offsetTest_white":
            continue

        # Get trial and sample numbers
        trial_num = int(data[1])
        sample_num = int(data[2])

        # Start a new CSV when the trial number changes
        if trial_num != current_trial:

            # Close the previous CSV
            if csv_file is not None:
                csv_file.close()

            current_trial = trial_num

            filename = f"{DATA_FOLDER}/trial_{trial_num:02d}.csv"

            csv_file = open(filename, "w", newline="")
            writer = csv.writer(csv_file)

            # Add the required column headings
            writer.writerow([
                "test_name",
                "trial_num",
                "sample_num",
                "sample_time",
                "sensor_num",
                "sensor_reading"
            ])

            print(f"\nStarting Trial {trial_num}")
            print(f"Saving to {filename}")

        # Only save the first 100 samples
        if sample_num <= SAMPLES_PER_TRIAL:

            writer.writerow(data)
            csv_file.flush()

            print(line)

        # Tell us when 100 samples have been collected
        if sample_num == SAMPLES_PER_TRIAL:
            print(
                f"Trial {trial_num} complete: "
                f"{SAMPLES_PER_TRIAL} samples"
            )
