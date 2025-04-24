package linefollowertest;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class SmartLineFollower {
    public static void main(String[] args) {
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
        SampleProvider light = colorSensor.getRedMode();
        float[] sample = new float[light.sampleSize()];

        float threshold = 30; // adjusted based on your readings
        int baseSpeed = 200;
        float kp = 2.0f; // scaled down due to raw sensor range

        boolean goingForward = true;
        int lostCount = 0;
        int lostLimit = 3;

        while (!Button.ESCAPE.isDown()) {
            light.fetchSample(sample, 0);
            float lightValue = sample[0] * 100; // convert to percentage scale

            // Debugging info
            LCD.clear();
            LCD.drawString("Light: " + lightValue, 0, 0);
            LCD.drawString("LostCount: " + lostCount, 0, 1);

            if (lightValue < threshold) {
                // Line detected
                lostCount = 0;
                float error = lightValue - threshold;
                int correction = (int)(kp * error);

                int leftSpeed = baseSpeed - correction;
                int rightSpeed = baseSpeed + correction;

                Motor.A.setSpeed(Math.max(0, Math.min(900, leftSpeed)));
                Motor.B.setSpeed(Math.max(0, Math.min(900, rightSpeed)));

                if (goingForward) {
                    Motor.A.forward();
                    Motor.B.forward();
                } else {
                    Motor.A.backward();
                    Motor.B.backward();
                }

            } else {
                // Line lost
                lostCount++;
                if (lostCount < lostLimit) {
                    Motor.A.setSpeed(baseSpeed);
                    Motor.B.setSpeed(baseSpeed);
                    Motor.A.forward();
                    Motor.B.forward();
                } else {
                    // Start recovery
                    Motor.A.stop(true);
                    Motor.B.stop();
                    Delay.msDelay(200);

                    // Try right
                    Motor.A.setSpeed(150);
                    Motor.B.setSpeed(150);
                    Motor.A.forward();
                    Motor.B.backward();
                    Delay.msDelay(450);

                    light.fetchSample(sample, 0);
                    if ((sample[0] * 100) < threshold) {
                        goingForward = true;
                        lostCount = 0;
                        continue;
                    }

                    // Try left
                    Motor.A.backward();
                    Motor.B.forward();
                    Delay.msDelay(900);

                    light.fetchSample(sample, 0);
                    if ((sample[0] * 100) < threshold) {
                        goingForward = true;
                        lostCount = 0;
                        continue;
                    }

                    // Still not found — go backward along path
                    goingForward = false;
                    lostCount = 0;
                }
            }

            Delay.msDelay(50);
        }

        Motor.A.stop();
        Motor.B.stop();
        colorSensor.close();
    }
}
