package ultrasonic;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.EV3LargeRegulatedMotor;  // For motor control
import lejos.hardware.port.MotorPort;

public class UltrasonicSensor {

    public static void main(String[] args) {
        // Initialize ultrasonic sensor at port S2
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
        SampleProvider distance = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distance.sampleSize()];

        // Initialize motors (assuming two motors for differential drive)
        EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
        EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.B);

        // Set initial motor speed (degrees per second)
        int fullSpeed = 300;  // Full speed
        int slowSpeed = 150;  // Half speed for slowing down

        // Start moving forward at full speed
        leftMotor.setSpeed(fullSpeed);
        rightMotor.setSpeed(fullSpeed);
        leftMotor.forward();
        rightMotor.forward();

        // Loop until ESCAPE button is pressed
        while (!Button.ESCAPE.isDown()) {
            // Read distance continuously
            distance.fetchSample(sample, 0);
            float distanceMeters = sample[0];  // Distance in meters
            float distanceCm = distanceMeters * 100;  // Convert to cm for easier comparison

            // Display distance on LCD
            LCD.clear();
            LCD.drawString("Dist: " + distanceCm + " cm", 0, 0);

            // Task: Slow down at 30 cm (0.3 meters)
            if (distanceCm <= 30 && distanceCm > 10) {
                leftMotor.setSpeed(slowSpeed);
                rightMotor.setSpeed(slowSpeed);
                leftMotor.forward();
                rightMotor.forward();
            }
            // Task: Stop at 10 cm (0.1 meters)
            else if (distanceCm <= 10) {
                // Stop motors
                leftMotor.stop(true);
                rightMotor.stop(true);

                // Task: Move aside to bypass object
                // Example: Turn left slightly, move forward, then turn right to bypass
                leftMotor.setSpeed(fullSpeed);
                rightMotor.setSpeed(fullSpeed);
                leftMotor.rotate(-360, true);  // Turn left (adjust angle as needed)
                rightMotor.rotate(360, false);
                leftMotor.forward();
                rightMotor.forward();
                try {
                    Thread.sleep(1000);  // Move forward for 1 second to bypass
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                leftMotor.rotate(360, true);  // Turn right to realign
                rightMotor.rotate(-360, false);

                // Resume moving forward
                leftMotor.setSpeed(fullSpeed);
                rightMotor.setSpeed(fullSpeed);
                leftMotor.forward();
                rightMotor.forward();

            }
            // If distance > 30 cm, continue at full speed
            else {
                leftMotor.setSpeed(fullSpeed);
                rightMotor.setSpeed(fullSpeed);
                leftMotor.forward();
                rightMotor.forward();
            }

            // Refresh every 100 ms
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        // Cleanup
        leftMotor.close();
        rightMotor.close();
        ultrasonicSensor.close();
    }
}
