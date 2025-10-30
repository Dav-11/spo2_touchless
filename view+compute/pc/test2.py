import serial
import time
import sqlite3
from collections import deque


class HighSpeedDAQ:
    def __init__(self, port, db_file="daq_data.db"):
        self.arduino = serial.Serial(port, 115200, timeout=0.1)
        # self.db_file = db_file
        self.buffer = deque(maxlen=10000)
        # self.init_database()
        time.sleep(2)

    # def init_database(self):
    #     conn = sqlite3.connect(self.db_file)
    #     conn.execute('''
    #                  CREATE TABLE IF NOT EXISTS readings
    #                  (
    #                      timestamp
    #                      INTEGER,
    #                      temperature
    #                      INTEGER,
    #                      light
    #                      INTEGER,
    #                      pressure
    #                      INTEGER
    #                  )
    #                  ''')
    #     conn.commit()
    #     conn.close()

    def parse_line(self, line):
        """Parse CSV data from Arduino"""
        try:
            parts = line.split(',')
            return {
                'timestamp': int(parts[0]),
                'temperature': int(parts[1]),
                'light': int(parts[2]),
                'pressure': int(parts[3])
            }
        except (ValueError, IndexError):
            return None

    # def save_batch(self, batch):
    #     """Save batch to database"""
    #     print(batch)

    #     conn = sqlite3.connect(self.db_file)
    #     conn.executemany(
    #         'INSERT INTO readings VALUES (?, ?, ?, ?)',
    #         [(d['timestamp'], d['temperature'], d['light'], d['pressure'])
    #          for d in batch]
    #     )
    #     conn.commit()
    #     conn.close()

    def collect_data(self, duration=60):
        """High-speed data collection"""
        start_time = time.time()
        batch = []

        print(f"Collecting data for {duration} seconds...")

        while time.time() - start_time < duration:
            if self.arduino.in_waiting:
                line = self.arduino.readline().decode().strip()
                print(f"LINE: {line}")
                # if line:
                #     data = self.parse_line(line)
                #     if data:
                #         batch.append(data)
                #         self.buffer.append(data)
                #
                #         # Save every 100 readings
                #         if len(batch) >= 100:
                #             self.save_batch(batch)
                #             print(f"Saved {len(batch)} readings")
                #             batch = []

        # Save remaining data
        if batch:
            self.save_batch(batch)

        print(f"Collection complete. {len(self.buffer)} total readings")

    # def get_stats(self):
    #     """Calculate statistics"""
    #     if not self.buffer:
    #         return None
    #
    #     temps = [d['temperature'] for d in self.buffer]
    #     return {
    #         'count': len(self.buffer),
    #         'temp_avg': sum(temps) / len(temps),
    #         'temp_min': min(temps),
    #         'temp_max': max(temps),
    #         'sample_rate': len(self.buffer) / 60  # samples per second
    #     }

    def close(self):
        self.arduino.close()


# Run high-speed acquisition
daq = HighSpeedDAQ('/dev/cu.usbserial-110')
daq.collect_data(120)  # 2 minutes
# stats = daq.get_stats()
# if stats:
#     print(f"Sample rate: {stats['sample_rate']:.1f} Hz")
#     print(f"Temperature range: {stats['temp_min']}-{stats['temp_max']}")
daq.close()