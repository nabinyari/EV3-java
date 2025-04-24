package main;

import lejos.hardware.motor.Motor;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class LineFollower implements Runnable {
    dataShare Data;

    public LineFollower(dataShare Data) {
        this.Data = Data;
    }
    public void run() {

        float black = Data.getBlack();
        float white = Data.getWhite();
        // Define thresholds
        float threshold = (black + white)/2;
        // PID constants
        float kp = 800; // Proportional gain

        while (!Button.ESCAPE.isDown()) {

            float lightSample = Data.getIntensity();
            float error = lightSample - threshold;

            // Proportional control
            int correction = (int)(kp * error);

            int baseSpeed = 200;
            int leftSpeed = baseSpeed - correction;
            int rightSpeed = baseSpeed + correction;

            // Clamp motor speeds to avoid negative or too high values
            leftSpeed = Math.max(0, Math.min(900, leftSpeed));
            rightSpeed = Math.max(0, Math.min(900, rightSpeed));

            Motor.A.setSpeed(leftSpeed);
            Motor.B.setSpeed(rightSpeed);
            Motor.A.forward();
            Motor.B.forward();

        synchronized(LCD.class) {
                    LCD.clear();
                    LCD.drawString("Light: " + (int)(lightSample * 100) + "%", 0, 0);
                    LCD.drawString("Black: " +  black, 0, 1);
                    LCD.drawString("White: " + white, 0, 2);
                    LCD.drawString("Threshold: " + (int)(threshold * 100) + "%", 0, 3);
                }

            Delay.msDelay(50);
        }

        Motor.A.stop();
        Motor.B.stop();
    }
}
