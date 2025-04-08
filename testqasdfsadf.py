#!/usr/bin/env python3

class GPIOSimulator:
    """Simulates GPIO functionality for testing without actual hardware"""
    BCM = "BCM Mode"
    OUT = "Output Mode"
    HIGH = 1
    LOW = 0
    
    def __init__(self):
        self.pins = {}
        self.mode = None
    
    def setmode(self, mode):
        self.mode = mode
        print(f"GPIO Mode set to: {mode}")
    
    def setup(self, pin, mode):
        self.pins[pin] = {"mode": mode, "state": self.LOW}
        print(f"Pin {pin} set up as {mode}")
    
    def output(self, pin, state):
        if pin in self.pins:
            self.pins[pin]["state"] = state
            print(f"Pin {pin} set to {'HIGH' if state else 'LOW'}")
        else:
            print(f"Error: Pin {pin} not set up")
    
    def cleanup(self):
        self.pins = {}
        print("GPIO cleaned up")

# Replace RPi.GPIO with our simulator
GPIO = GPIOSimulator()

# Define GPIO pins for each relay (using the same pins as in the original code)
RELAY_PINS = [31, 33, 35, 37]

def setup_gpio():
    GPIO.setmode(GPIO.BCM)  # Use BCM numbering
    for pin in RELAY_PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)  # Start all relays OFF

def relay_control(input_str):
    """Process relay control command string"""
    try:
        states = list(map(int, input_str.strip().split(',')))
        if len(states) != 4:
            print(f"Expected 4 values, got: {input_str}")
            return
            
        # Validate each state is 0 or 1
        for state in states:
            if state not in [0, 1]:
                print(f"Invalid state: {state}. Must be 0 or 1")
                return
                
        # Set each relay ON (HIGH) or OFF (LOW)
        for i in range(4):
            GPIO.output(RELAY_PINS[i], GPIO.HIGH if states[i] else GPIO.LOW)
            
        print(f"Relays set to: {states}")
        print_relay_status()
        
    except Exception as e:
        print(f"Invalid input: {input_str}")
        print(f"Error: {e}")

def print_relay_status():
    """Print the current status of all relays"""
    status = []
    for pin in RELAY_PINS:
        status.append("ON" if GPIO.pins[pin]["state"] == GPIO.HIGH else "OFF")
    
    print("\nCurrent Relay Status:")
    for i, (pin, state) in enumerate(zip(RELAY_PINS, status)):
        print(f"Relay {i+1} (Pin {pin}): {state}")
    print()

def main():
    print("Relay Control Test Node")
    print("======================")
    setup_gpio()
    print("\nRelay node ready. Enter relay states as comma-separated values (0,0,0,0)")
    print("Enter 'q' to quit\n")
    
    try:
        while True:
            user_input = input("Enter relay states (0,1,0,1): ")
            if user_input.lower() == 'q':
                break
            relay_control(user_input)
    except KeyboardInterrupt:
        print("\nProgram interrupted")
    finally:
        GPIO.cleanup()
        print("Program ended")

if __name__ == '__main__':
    main()