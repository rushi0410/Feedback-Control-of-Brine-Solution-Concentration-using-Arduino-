Automated PI-Controlled TDS/PPM Mixing System

An Arduino-based precision chemical dosing system designed to maintain a target Total Dissolved Solids (TDS) concentration in a solution. This project utilizes a closed-loop feedback system with a Proportional-Integral (PI) controller to manage dual peristaltic pumps (Brine and Fresh Water).

🚀 Key Features

PI Control Algorithm: Implements a Proportional-Integral controller to eliminate steady-state error and ensure the solution reaches the exact setpoint.

Anti-Windup Logic: Includes integral clamping to prevent "windup" and overshoot during large concentration changes.

EMI Noise Mitigation (Pulsed Logic): Features a custom state machine that synchronizes sensor readings with pump "off" states, eliminating electrical interference from the 12V motors.

Real-time Telemetry: Outputs live data to an I2C 16x2 LCD and supports the Arduino Serial Plotter for visual tuning of concentration gradients.

Safety Clamps: Software-defined limits for pump speeds and concentration safety ranges.

🛠️ Hardware Requirements

Microcontroller: Arduino Uno (or compatible)

Driver: L298N Dual H-Bridge Motor Driver

Sensor: Analog TDS Sensor (0-1000 PPM range)

Pumps: 2x 12V Peristaltic Pumps (Brine and Fresh Water)

Display: 16x2 I2C LCD Module

Power: 12V 2A DC Power Supply

🔌 Wiring Diagram

TDS Sensor

Signal Pin: Arduino A0

VCC: Arduino 5V

GND: Arduino GND

I2C LCD Display

SDA: Arduino A4

SCL: Arduino A5

VCC: Arduino 5V

GND: Arduino GND

L298N Motor Driver & Pumps

Brine Pump Speed (ENA): Arduino D3

Brine Pump Direction (IN1, IN2): Arduino D7, D6

Water Pump Speed (ENB): Arduino D9

Water Pump Direction (IN3, IN4): Arduino D5, D4

Note: Ensure a common ground (GND) is shared between the 12V supply, the L298N, and the Arduino.

⚙️ Control Logic: The "Pulsed Cycle"

To ensure accurate readings, the system operates in a timed state machine:

Reading State: Pumps are stopped. The system waits for the solution to mix and for electrical noise to dissipate before taking a clean TDS reading.

Logic State: The PI controller calculates the error and determines the required pump duration and speed.

Pumping State: The selected pump runs for a short burst (default 500ms).

Mixing Delay: The system pauses (default 1500ms) to allow the new liquid to integrate before the next reading.

📈 Usage

Clone the repository.

Open the .ino file in the Arduino IDE.

Install the LiquidCrystal_I2C library.

Set your desired SETPOINT_PPM in the code.

Upload the code and open the Serial Plotter (9600 baud) to view the tracking.

📄 License

This project is open-source. Feel free to modify and use it for your own dosing applications
