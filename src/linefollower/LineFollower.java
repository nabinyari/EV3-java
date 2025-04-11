package linefollower;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {

    public static void main(String[] args) {
        // Create and configure the color sensor
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);

        // You can try & use other modes to see the difference & feel free to use the mode that
        // suites your needs the best, for instance, getAmbientMode(), getRedMode(), etc
        // Here we are setting the sensor to red mode (measures reflected red light)
        SampleProvider light = colorSensor.getRedMode();  // Use red mode for reflected light intensity

        // Create an array to hold the sensor data
        float[] sample = new float[light.sampleSize()];

        // Set motor speeds
        Motor.A.setSpeed(300);
        Motor.B.setSpeed(300);

        // Start motors moving forward
        Motor.A.forward();
        Motor.B.forward();

        // Continuously follow the line until a button is pressed
        while (!Button.ESCAPE.isDown()) {
            // Get the current red light intensity reading from the sensor
            light.fetchSample(sample, 0);  // 0 is the index where data will be stored
            
            // for debugging purposes, better to display light intensity on LCD
            LCD.clear();
            LCD.drawString("Red Light Intensity: " + (int)(sample[0] * 100) + "%", 0, 0);
            
            // Threshold for detecting the black line
            // NOTE: You'll most probably have to fine tune the threshold value
            float threshold = 0.2f;  // Adjusted threshold value for the black line detection

            // If the light intensity is low (black line), the robot is on the line
            if (sample[0] < threshold)
            { 
                Motor.A.setSpeed(300);
                Motor.B.setSpeed(300);
                Motor.A.forward();
                Motor.B.forward();
            }
            else                        // Off the black line, adjust to turn towards the line
            {
                // Work on these logics. they are just for illustration purposes.
                // The turns defined below can be inverted turns. So test yourselves and rectify accordingly
                if (sample[0] > 0.6)
                {
                    // If it's very bright (white surface), turn left
                    Motor.A.setSpeed(300);
                    Motor.B.setSpeed(150);
                }
                else
                {
                    // If it's somewhat bright (near the edge of the line), turn right
                    Motor.A.setSpeed(150);
                    Motor.B.setSpeed(300);
                }
                Motor.A.forward();
                Motor.B.forward();
            }

            // Add a small delay to reduce the frequency of updates
            Delay.msDelay(50);
        }

        // Stop the motors before exiting
        Motor.A.stop();
        Motor.B.stop();
        
        // Remember to close the sensor before exiting
        colorSensor.close();
    }
}