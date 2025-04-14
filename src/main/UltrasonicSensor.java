package main;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;

public class UltrasonicSensor implements Runnable {
    public void run() {
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
        SampleProvider distance = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distance.sampleSize()];

        EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
        EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.B);

        int fullSpeed = 300;
        int slowSpeed = 150;

        leftMotor.setSpeed(fullSpeed);
        rightMotor.setSpeed(fullSpeed);
        leftMotor.forward();
        rightMotor.forward();

        while (!Button.ESCAPE.isDown()) {
            distance.fetchSample(sample, 0);
            float distanceMeters = sample[0];
            float distanceCm = distanceMeters * 100;

            synchronized (LCD.class) {
                LCD.clear();
                LCD.drawString("Dist: " + distanceCm + " cm", 0, 0);
            }

            if (distanceCm <= 12 && distanceCm > 5) {
                // Slow down
                leftMotor.setSpeed(slowSpeed);
                rightMotor.setSpeed(slowSpeed);
                leftMotor.forward();
                rightMotor.forward();
            } else if (distanceCm <= 5) {
                // Stop
                leftMotor.stop(true);
                rightMotor.stop();

                // Avoid the object (basic right-avoid-left bypass logic)
                try {
                    // Turn right
                    leftMotor.setSpeed(fullSpeed);
                    rightMotor.setSpeed(fullSpeed);
                    leftMotor.forward();
                    rightMotor.backward();
                    Thread.sleep(500); // Adjust timing for turn

                    // Move forward to bypass
                    leftMotor.forward();
                    rightMotor.forward();
                    Thread.sleep(1000); // Adjust based on object size

                    // Turn left to find the line again
                    leftMotor.backward();
                    rightMotor.forward();
                    Thread.sleep(500);

                    // Continue forward
                    leftMotor.forward();
                    rightMotor.forward();
                    Thread.sleep(1000);

                    // Restore full speed
                    leftMotor.setSpeed(fullSpeed);
                    rightMotor.setSpeed(fullSpeed);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                // Default full speed
                leftMotor.setSpeed(fullSpeed);
                rightMotor.setSpeed(fullSpeed);
                leftMotor.forward();
                rightMotor.forward();
            }

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        leftMotor.close();
        rightMotor.close();
        ultrasonicSensor.close();
    }
}
