package main;

// Import necessary EV3 libraries for motor, sensor, button, and LCD display
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

// Define the LineFollower class that implements Runnable (so it can run in a thread)
public class LineFollower implements Runnable {
    datashare Data; // Shared object to get distance data from ultrasonic sensor

    // Constructor to link shared data
    public LineFollower(datashare Data) {
        this.Data = Data;
    }

    // The method that is executed when the thread starts
    @Override
    public void run() {
        // Initialize the color sensor on port S4
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
        // Set the sensor to red mode (light intensity)
        SensorMode redMode = colorSensor.getRedMode();
        float[] lightSample = new float[redMode.sampleSize()]; // Create a sample array to store light values

        // Calibration: Get black surface value
        float blackValue, whiteValue;
        LCD.clear();
        LCD.drawString("Put on BLACK", 0, 0); // Ask user to put sensor on black line
        Button.waitForAnyPress();
        redMode.fetchSample(lightSample, 0);
        blackValue = lightSample[0]; // Store black value

        // Calibration: Get white surface value
        LCD.clear();
        LCD.drawString("Put on WHITE", 0, 0); // Ask user to put sensor on white surface
        Button.waitForAnyPress();
        redMode.fetchSample(lightSample, 0);
        whiteValue = lightSample[0]; // Store white value

        // Calculate threshold to differentiate black line and white background
        float threshold = (blackValue + whiteValue) / 2;

        // Set up simple proportional (P) control values
        float kp = 900;  // Proportional gain (how strongly to react to error)
        int baseSpeed = 200; // Default speed of motors

        // Start moving both motors forward
        Motor.A.forward();
        Motor.B.forward();

        // Main loop: runs until user presses ESCAPE button
        while (!Button.ESCAPE.isDown()) {
            // Read the current brightness from the sensor
            redMode.fetchSample(lightSample, 0);
            float lightValue = lightSample[0];

            // Calculate error from the threshold
            float error = lightValue - threshold;
            int correction = (int) (kp * error); // Apply proportional gain

            // Adjust left and right motor speeds based on error
            int leftSpeed = baseSpeed - correction;
            int rightSpeed = baseSpeed + correction;

            // Clamp motor speeds between 0 and 900 (to stay safe)
            leftSpeed = Math.max(0, Math.min(900, leftSpeed));
            rightSpeed = Math.max(0, Math.min(900, rightSpeed));

            // Set the motor speeds
            Motor.A.setSpeed(leftSpeed);
            Motor.B.setSpeed(rightSpeed);
            Motor.A.forward();
            Motor.B.forward();

            // Get the current distance reading from the ultrasonic sensor
            float distance = Data.getDistance();

            // If the object is extremely close (less than 5 cm)
            if(distance < 0.05f) {
                // Stop both motors immediately
                Motor.A.stop();
                Motor.B.stop();
                LCD.clear();
                LCD.drawString("Object ahead", 0, 0);
                Delay.msDelay(500); // Wait half a second

                // Backup the robot
                Motor.A.setSpeed(200);
                Motor.B.setSpeed(200);
                Motor.A.backward();
                Motor.B.backward();
                Delay.msDelay(1000); // Reverse for 1 second

                // Turn robot to the right
                Motor.A.setSpeed(300);
                Motor.B.setSpeed(100);
                Motor.A.forward();
                Motor.B.forward();
                Delay.msDelay(600); // Turn for 0.6 seconds

                // Resume moving forward
                Motor.A.forward();
                Motor.B.forward();
                continue; // Skip the rest of loop and restart
            }
            // If object is detected within 15 cm
            else if (distance < 0.15f) {
                // Stop motors for a moment
                Motor.A.stop();
                Motor.B.stop();
                LCD.clear();
                LCD.drawString("Object detected", 0, 0);
                Delay.msDelay(500);

                // Pivot turn sharply
                Motor.A.setSpeed(300);
                Motor.B.setSpeed(300);
                Motor.A.rotate(180, true); // Right wheel forward
                Motor.B.rotate(-180);       // Left wheel backward

                // Move forward to bypass the obstacle
                Motor.A.setSpeed(150);
                Motor.B.setSpeed(200);
                Motor.A.forward();
                Motor.B.forward();
                Delay.msDelay(1000); // Move forward longer

                // Start searching for black line again
                boolean lineFound = false;
                while (!lineFound) {
                    redMode.fetchSample(lightSample, 0);
                    lightValue = lightSample[0];

                    if (lightValue < threshold) {
                        // Found black line
                        Motor.A.stop();
                        Motor.B.stop();
                        LCD.clear();
                        LCD.drawString("Line found", 0, 0);

                        // Align robot properly
                        Motor.A.setSpeed(200);
                        Motor.B.setSpeed(200);
                        Motor.A.rotate(60, true);  // Right turn
                        Motor.B.rotate(-60);       // Left turn
                        Delay.msDelay(100); // Allow small adjustment

                        lineFound = true; // Stop searching
                    } else {
                        // Keep searching by moving forward
                        Motor.A.forward();
                        Motor.B.forward();
                        Delay.msDelay(30); // Small delay for stability
                    }
                }
                continue; // Resume normal line following
            }

            // Update LCD to show sensor values
            synchronized(LCD.class) {
                LCD.clear();
                LCD.drawString("Light: " + (int)(lightValue * 100) + "%", 0, 0);
                LCD.drawString("Distance: " + (int)(distance * 100) + " cm", 0, 1);
            }

            // Short delay before reading sensors again
            Delay.msDelay(50);
        }

        // When ESCAPE is pressed, stop motors completely
        Motor.A.stop();
        Motor.B.stop();
    }
}
