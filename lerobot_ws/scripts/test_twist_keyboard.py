import subprocess
import time


class TeleopTwistKeyboard:
    def __init__(self):
        self.process = None

    def start(self):
        if self.process is not None and self.process.poll() is None:
            print("Keyboard teleop is already running.")
            return

        command = [
            "ros2", "run",
            "teleop_twist_keyboard",
            "teleop_twist_keyboard",
            "--ros-args",
            "-p", "stamped:=true",
            "-p", "frame_id:=base_link",
            "-p", "speed:=0.05", 
            "-p", "turn:=0.1", 
            "-r", "cmd_vel:=/twist_keyboard/cmd_vel",
        ]

        self.process = subprocess.Popen(command)

        print("Keyboard teleop started.")

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            self.process.wait()

        self.process = None
        print("Keyboard teleop stopped.")
        

def main():
    keyboard = TeleopTwistKeyboard()
    
    try:
        keyboard.start()
        
        while(True):
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nCtrl+C detected.")
    
    finally:
        keyboard.stop()


if __name__ == "__main__":
    main()