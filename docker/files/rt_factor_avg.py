#!/usr/bin/env python3
import subprocess
import re
import time
from collections import deque

# Time window for rolling average (seconds)
WINDOW_SIZE_SECONDS = 10
# Display interval (seconds)
DISPLAY_INTERVAL = 1.0
# Maximum window capacity
MAX_WINDOW = 1000

def parse_real_time_factor(line):
    match = re.search(r'real_time_factor:\s+([0-9.]+)', line)
    if match:
        return float(match.group(1))
    return None

def main():
    window = deque(maxlen=MAX_WINDOW)
    last_print = time.time()

    # Start 'gz topic' command
    process = subprocess.Popen(
        ['gz', 'topic', '-e', '-t', '/world/default/stats'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    print(f"Average Gazebo real time factor over a {WINDOW_SIZE_SECONDS} seconds rolling window")

    try:
        for line in process.stdout:
            rtf = parse_real_time_factor(line)
            if rtf is not None:
                timestamp = time.time()
                window.append((timestamp, rtf))

                # Remove old values
                current_time = time.time()
                while window and (current_time - window[0][0] > WINDOW_SIZE_SECONDS):
                    window.popleft()

                # Print each DISPLAY_INTERVAL
                if time.time() - last_print >= DISPLAY_INTERVAL:
                    if window:
                        values = [x[1] for x in window]
                        avg = sum(values) / len(values)
                        print(f"real_time_factor : {avg:.4f}")
                    last_print = time.time()

    except KeyboardInterrupt:
        print("\nThe end.")
    finally:
        process.terminate()

if __name__ == "__main__":
    main()
