"""
arm_serial.py
--------------
Serial transport layer between Python and the Arduino. This file's
only job is talking to the Arduino over USB — no IK math, no Pygame,
no UI. If the arm ever moves to a different transport (e.g. Bluetooth,
a different board), this is the only file that should need to change.
 
Protocol (matches arm_controller.ino):
    - Baud: 115200
    - Send:    "base,shoulder,elbow\n"   (integers, 0-180)
    - Receive: "OK base,shoulder,elbow\n"  on success
               "ERR bad line: ...\n"       on malformed input
               "READY\n"                   once, right after boot/reset
 
Known hardware quirk this file accounts for:
    Opening the serial port causes the CH340 chip to reset the Arduino
    (same as a physical reset button press). That means setup() runs
    again, re-homing the arm and re-printing "READY" — but it takes
    the board a moment to boot. Sending a command too early gets lost.
    This file waits for "READY" (with a timeout fallback) rather than
    using a blind sleep, so it's robust even if boot time varies.
"""
 
import time
import serial
 
 
class ArmConnection:
    def __init__(self, port, baud=115200, boot_timeout=5.0, min_send_interval=0.05):
        """
        Args:
            port: serial device path, e.g. '/dev/cu.wchusbserial10'
            baud: must match Serial.begin() in the Arduino sketch
            boot_timeout: max seconds to wait for "READY" after connecting
                          before giving up and proceeding anyway
            min_send_interval: minimum seconds between sends (throttle).
                          0.05s = ~20Hz, matches the noted serial throttle
                          limit to avoid overrunning the Arduino's buffer.
        """
        self.port = port
        self.baud = baud
        self.min_send_interval = min_send_interval
        self._last_send_time = 0.0
 
        try:
            self._serial = serial.Serial(port, baud, timeout=1)
        except serial.SerialException as e:
            raise ConnectionError(
                f"Could not open serial port '{port}'. Is the Arduino "
                f"plugged in, and is Serial Monitor / another program "
                f"closed? Original error: {e}"
            )
 
        self._wait_for_ready(boot_timeout)
 
    def _wait_for_ready(self, timeout):
        """Wait for the Arduino's boot handshake, without blocking forever."""
        start = time.time()
        while time.time() - start < timeout:
            line = self._serial.readline().decode(errors="replace").strip()
            if line == "READY":
                return
            if line:
                # Some other startup noise — keep waiting, but don't hang silently
                print(f"[arm_serial] (waiting for READY, got: '{line}')")
        print("[arm_serial] WARNING: never saw READY — proceeding anyway. "
              "Commands sent now may be lost if the board is still booting.")
 
    def send_angles(self, base_deg, shoulder_deg, elbow_deg):
        """
        Sends a "base,shoulder,elbow\\n" command, throttled to
        min_send_interval. Returns the Arduino's response line
        (e.g. "OK 90,45,120"), or None if throttled/skipped.
        """
        now = time.time()
        if now - self._last_send_time < self.min_send_interval:
            return None  # throttled — caller can just try again next frame
 
        command = f"{int(base_deg)},{int(shoulder_deg)},{int(elbow_deg)}\n"
        self._serial.write(command.encode())
        self._last_send_time = now
 
        response = self._serial.readline().decode(errors="replace").strip()
        if response.startswith("ERR"):
            print(f"[arm_serial] Arduino reported an error: {response}")
        return response
 
    def close(self):
        if self._serial.is_open:
            self._serial.close()
 
    def __enter__(self):
        return self
 
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
 
 
if __name__ == "__main__":
    # Quick manual test — run `python3 arm_serial.py` with the Arduino
    # plugged in (and Serial Monitor CLOSED) to confirm the connection
    # works end-to-end before wiring this into main.py.
    PORT = "/dev/cu.wchusbserial10"
 
    print(f"Connecting to {PORT}...")
    with ArmConnection(PORT) as arm:
        print("Connected. Sending test sequence...")
 
        test_moves = [
            (90, 90, 90),
            (45, 90, 90),
            (135, 90, 90),
            (90, 60, 120),
            (90, 90, 90),
        ]
 
        for base, shoulder, elbow in test_moves:
            response = arm.send_angles(base, shoulder, elbow)
            print(f"sent ({base},{shoulder},{elbow}) -> {response}")
            time.sleep(0.5)  # slower than the throttle, just for readability here
 
    print("Done, connection closed.")