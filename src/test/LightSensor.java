package test;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;

import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LightSensor implements Runnable {

    public void run() {
            EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
            SampleProvider lightIntensity = colorSensor.getRedMode();
            float[] lightSample = new float[lightIntensity.sampleSize()];

                // Calibration phase
                LCD.clear();
                LCD.drawString("Put on BLACK", 0, 0);
                Button.waitForAnyPress();
                lightIntensity.fetchSample(lightSample, 0);
                float blackValue = lightSample[0];

                Button.waitForAnyPress();

                LCD.clear();
                LCD.drawString("Put on WHITE", 0, 0);
                Button.waitForAnyPress();
                lightIntensity.fetchSample(lightSample, 0);
                float whiteValue = lightSample[0];

                TextWrap("Calibration done! Now running...", 0);
                TextWrap("Press ESC to exit.", 1);
                Button.ENTER.waitForPressAndRelease();
                float threshold = (blackValue + whiteValue)/2;

            // PID constants
            float kp = 700; // Proportional gain
            // Proceed with normal sensor loop
            while (!Button.ESCAPE.isDown()) 
            {

            lightIntensity.fetchSample(lightSample, 0);
            float lightValue = lightSample[0];
            float error = lightValue - threshold;

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
                    LCD.drawString("Light: " + (int)(lightValue * 100) + "%", 0, 0);
                    LCD.drawString("Black: " +  blackValue, 0, 1);
                    LCD.drawString("White: " + whiteValue, 0, 2);
                    LCD.drawString("Threshold: " + (int)(threshold * 100) + "%", 0, 3);
                }

            Delay.msDelay(50);
        }
        
        Motor.A.stop();
        Motor.B.stop();
    }

    public static void TextWrap(String msg, int startY) {
        int maxLength = 16;
        int y = startY;
        
        for (int i = 0; i < msg.length() && y < 8; i += maxLength) {
            String line = msg.substring(i, Math.min(i + maxLength, msg.length()));
            LCD.drawString(line, 0, y++);
        }
    }
}