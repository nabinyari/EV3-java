package main;

import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

public class LineFollower implements Runnable{
    datashare Data;

    public LineFollower(datashare Data) {
        this.Data = Data;
    }
    @Override
    public void run() {
        // Initialize motors, sensors, and screen
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
        // Get sensor modes
        SensorMode redMode = colorSensor.getRedMode();
        float[] lightSample = new float[redMode.sampleSize()];

        // Calibration of line threshold
        float blackValue, whiteValue;
        LCD.clear();
        LCD.drawString("Put on BLACK", 0, 0);
        Button.waitForAnyPress();
        redMode.fetchSample(lightSample, 0);
        blackValue = lightSample[0];

        LCD.clear();
        LCD.drawString("Put on WHITE", 0, 0);
        Button.waitForAnyPress();
        redMode.fetchSample(lightSample, 0);
        whiteValue = lightSample[0];

        // Set up threshold for line following
        float threshold = (blackValue + whiteValue) / 2;

        // PID constants (Proportional control)
        float kp = 700;  // Proportional gain
        int baseSpeed = 200;

        // Start moving the robot
        Motor.A.forward();
        Motor.B.forward();

        while (!Button.ESCAPE.isDown()) {
            // Read light sensor value
            redMode.fetchSample(lightSample, 0);
            float lightValue = lightSample[0];

            // Calculate error and correction
            float error = lightValue - threshold;
            int correction = (int) (kp * error);

            // Calculate motor speeds
            int leftSpeed = baseSpeed - correction;
            int rightSpeed = baseSpeed + correction;

            // Clamp motor speeds
            leftSpeed = Math.max(0, Math.min(900, leftSpeed));
            rightSpeed = Math.max(0, Math.min(900, rightSpeed));

            // Set motor speeds
            Motor.A.setSpeed(leftSpeed);
            Motor.B.setSpeed(rightSpeed);
            Motor.A.forward();
            Motor.B.forward();

            // Check for obstacles using ultrasonic sensor
            float distance = Data.getDistance();

            if(distance < 0.05f)
            {
                    // Stop
                Motor.A.stop();
                Motor.B.stop();
                LCD.clear();
                LCD.drawString("Object ahead", 0, 0);
                Delay.msDelay(500);

                // Backup
                Motor.A.setSpeed(200);
                Motor.B.setSpeed(200);
                Motor.A.backward();
                Motor.B.backward();
                Delay.msDelay(1000);

                // Now turn
                Motor.A.setSpeed(300);
                Motor.B.setSpeed(100);
                Motor.A.forward();
                Motor.B.forward();
                Delay.msDelay(600);

                // Resume
                Motor.A.forward();
                Motor.B.forward();
                continue; 
            }
            // If an object is detected within 12 cm, bypass it
            else if (distance < 0.15f) {  // Object within 15 cm
                // Stop both motors momentarily to stop the robot
                Motor.A.stop();
                Motor.B.stop();
                LCD.clear();
                LCD.drawString("Object detected", 0, 0);
                Delay.msDelay(500);  // Wait a bit before turning
            
                // Perform a gentle curved turn to bypass the object (without excessive turn)
                // Turn a bit more clearly:
                Motor.A.setSpeed(300);  // Keep A steady
                Motor.B.setSpeed(50);  // Slow B down much more
            
                // Perform a slight curve turn: both motors forward but one slightly slower
                Motor.A.forward();  // Move forward with Motor A
                Motor.B.forward();  // Move forward with Motor B (for a slight curve, you can change speeds if needed)
                
                Delay.msDelay(700);  // Adjust this value for the desired arc size and bypass time
            
                // Continue line following
                continue;  // Skip the rest of the code and check again
            }

            //  // Lost the line (very bright floor) → Assume 90° turn
            // else if(lightValue > (threshold + 0.2f)) {  
            // // Stop
            //     Motor.A.stop();
            //     Motor.B.stop();
            //     LCD.clear();
            //     LCD.drawString("90° Turn!", 0, 0);
            //     Delay.msDelay(500);

            //     // Pivot turn (90 degrees) - Example: Right turn
            //     Motor.A.setSpeed(300);
            //     Motor.B.setSpeed(300);
            //     Motor.A.rotate(140, true);   // Rotate left wheel forward 140°
            //     Motor.B.rotate(-140);        // Rotate right wheel backward 140°

            //     // Move a little forward to catch line again
            //     Motor.A.setSpeed(200);
            //     Motor.B.setSpeed(200);
            //     Motor.A.forward();
            //     Motor.B.forward();
            //     Delay.msDelay(300);  

            //     continue;  // Skip the rest and recheck
            // }
                
            // Continue following the line
            Motor.A.forward();
            Motor.B.forward();

            // Update the LCD screen with current values
            synchronized(LCD.class) {
                LCD.clear();
                LCD.drawString("Light: " + (int)(lightValue * 100) + "%", 0, 0);
                LCD.drawString("Distance: " + (int)(distance * 100) + " cm", 0, 1);
            }

            // Delay for a short time before next loop iteration
            Delay.msDelay(50);
        }

        // Stop motors when ESC is pressed
        Motor.A.stop();
        Motor.B.stop();
    }
}
