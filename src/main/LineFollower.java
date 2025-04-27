package main;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower implements Runnable {
    private datashare Data;
    private final int baseSpeed = 200; // Normal line following speed

    public LineFollower(datashare Data) {
        this.Data = Data;
    }

    @Override
    public void run() {
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
        SensorMode redMode = colorSensor.getRedMode();
        float[] lightSample = new float[redMode.sampleSize()];

        // --- Calibration Step (only black) ---
        LCD.clear();
        LCD.drawString("Put on BLACK", 0, 0);
        Button.waitForAnyPress();
        redMode.fetchSample(lightSample, 0);
        float blackValue = lightSample[0];

        // Set a threshold slightly higher than black
        float threshold = blackValue + 0.05f;

        // PID control variables
        float kp = 800; // proportional gain (higher = more aggressive turns)
        Motor.A.forward();
        Motor.B.forward();

        while (!Button.ESCAPE.isDown()) {
            // Read color sensor
            redMode.fetchSample(lightSample, 0);
            float lightValue = lightSample[0];

            // Calculate error and correction
            float error = lightValue - threshold;
            int correction = (int) (kp * error);

            // Calculate motor speeds
            int leftSpeed = baseSpeed - correction;
            int rightSpeed = baseSpeed + correction;

            // Clamp motor speeds
            leftSpeed = Math.max(100, Math.min(900, leftSpeed));
            rightSpeed = Math.max(100, Math.min(900, rightSpeed));

            // Set motor speeds
            Motor.A.setSpeed(leftSpeed);
            Motor.B.setSpeed(rightSpeed);
            Motor.A.forward();
            Motor.B.forward();

            float distance=Data.getDistance();
            // --- Ultrasonic Sensor Check ---
            if (distance < 0.12f && distance > 0.01f) {
                Motor.A.setSpeed(150);
                Motor.B.setSpeed(150);
                Motor.A.forward();
                Motor.B.forward();
            
                long startTime = System.currentTimeMillis();
            
                while (distance < 0.12f && !Button.ESCAPE.isDown()) {
                    distance = Data.getDistance();
                    Delay.msDelay(50);
                }
            
                long elapsed = System.currentTimeMillis() - startTime;
            
                Motor.A.stop(true);
                Motor.B.stop();
                Delay.msDelay(300);
            
                LCD.clear();
                LCD.drawString("Bypassing...", 0, 0);
            
                if (elapsed > 2000) {
                    // Wide Object → BIG bypass
                    Motor.A.setSpeed(300);
                    Motor.B.setSpeed(100);
                    Motor.A.forward();
                    Motor.B.forward();
                    Delay.msDelay(1300);
            
                    Motor.A.setSpeed(300);
                    Motor.B.setSpeed(300);
                    Motor.A.forward();
                    Motor.B.forward();
                    Delay.msDelay(2000);
            
                    Motor.A.setSpeed(100);
                    Motor.B.setSpeed(300);
                    Motor.A.forward();
                    Motor.B.forward();
                    Delay.msDelay(1300);
                } else {
                    // Small Object → normal bypass
                    Motor.A.setSpeed(300);
                    Motor.B.setSpeed(100);
                    Motor.A.forward();
                    Motor.B.forward();
                    Delay.msDelay(1000);
            
                    Motor.A.setSpeed(300);
                    Motor.B.setSpeed(300);
                    Motor.A.forward();
                    Motor.B.forward();
                    Delay.msDelay(1500);
            
                    Motor.A.setSpeed(100);
                    Motor.B.setSpeed(300);
                    Motor.A.forward();
                    Motor.B.forward();
                    Delay.msDelay(1000);
                }
            
                Motor.A.setSpeed(baseSpeed);
                Motor.B.setSpeed(baseSpeed);
                Motor.A.forward();
                Motor.B.forward();
                Delay.msDelay(300);
            }
            


            Delay.msDelay(30); // Small delay for loop stability
        }

        // Cleanup
        colorSensor.close();
        Motor.A.stop();
        Motor.B.stop();
    }
}
