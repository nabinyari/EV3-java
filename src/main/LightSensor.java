package main;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;

public class LightSensor implements Runnable {
    dataShare Data;

    public LightSensor(dataShare Data) {
        this.Data = Data;
    }

    public void run() {
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
        try {
            SampleProvider lightIntensity = colorSensor.getRedMode();
            float[] lightSample = new float[lightIntensity.sampleSize()];

            synchronized (LCD.class) {
                // Calibration phase
                LCD.clear();
                LCD.drawString("Put on BLACK", 0, 0);
                Button.waitForAnyPress();
                lightIntensity.fetchSample(lightSample, 0);
                LCD.clear();
                LCD.drawString("Black value: " + lightSample[0], 0, 0);  // Show the raw value
                Data.setBlack(lightSample[0]);

                Button.waitForAnyPress();

                LCD.clear();
                LCD.drawString("Put on WHITE", 0, 0);
                Button.waitForAnyPress();
                lightIntensity.fetchSample(lightSample, 0);
                LCD.clear();
                LCD.drawString("White value: " + lightSample[0], 0, 0);  // Show the raw value
                Data.setWhite(lightSample[0]);

                LCD.clear();
                TextWrap("Calibration done! Now running...", 0);
                TextWrap("Press ESC to exit.", 1);
                Button.ENTER.waitForPressAndRelease();
            }


            // Proceed with normal sensor loop
            while (!Button.ESCAPE.isDown()) {
                lightIntensity.fetchSample(lightSample, 0);
                Data.setIntensity(lightSample[0]);
                
                Thread.sleep(100);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
            LCD.clear();
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