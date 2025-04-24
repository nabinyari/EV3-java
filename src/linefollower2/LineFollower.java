package linefollower2;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {
    public static void main(String[] args) {
        // Sensor setup
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
        SampleProvider light = colorSensor.getRedMode();
        float[] lightSample = new float[light.sampleSize()];

        // Define thresholds
        float blackValue = 0.1f;
        float whiteValue = 0.6f;
        float threshold = (blackValue + whiteValue) / 2;

        // PID constants (you can tune these)
        float kp = 800; // Proportional gain

        while (!Button.ESCAPE.isDown()) {
            light.fetchSample(lightSample, 0);
            float error = lightSample[0] - threshold;

            // Proportional control
            int correction = (int)(kp * error);

            int baseSpeed = 200;
            int leftSpeed = baseSpeed + correction;
            int rightSpeed = baseSpeed - correction;
            


            // Clamp motor speeds to avoid negative or too high values
            leftSpeed = Math.max(0, Math.min(900, leftSpeed));
            rightSpeed = Math.max(0, Math.min(900, rightSpeed));

            Motor.A.setSpeed(leftSpeed);
            Motor.B.setSpeed(rightSpeed);
            Motor.A.forward();
            Motor.B.forward();

            // Debug info
            LCD.clear();
            LCD.drawString("Light: " + (int)(lightSample[0] * 100) + "%", 0, 0);
            LCD.drawString("Error: " + error, 0, 1);
            LCD.drawString("L: " + leftSpeed, 0, 2);
            LCD.drawString("R: " + rightSpeed, 0, 3);

            Delay.msDelay(50);
        }

        Motor.A.stop();
        Motor.B.stop();
        colorSensor.close();
    }
}
