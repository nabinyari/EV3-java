package main;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;

public class LightSensor implements Runnable{
        dataShare Data;

        public LightSensor(dataShare Data) {
            this.Data = Data;
        }
    public void run()
    {
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);

        SampleProvider lighIntensity = colorSensor.getRedMode(); // // Measures reflected light intensity 
        // Create an array to hold the sensor data
        float[] lighIntensitys = new float[lighIntensity.sampleSize()];

        SampleProvider colorId = colorSensor.getColorIDMode(); // using the getColorIDMode() for detection of basic color as ID
        float[] colorSample = new float[colorId.sampleSize()];

        while (!Button.ESCAPE.isDown()) // Exit if the ESCAPE button is pressed
        {
            // Get the current light intensity reading from the sensor
            lighIntensity.fetchSample(lighIntensitys, 0);
            Data.setIntensity(lighIntensitys[0]);
            // Get the current color Id reading from the sensor
            colorId.fetchSample(colorSample, 0);
            int colorDetector = (int)colorSample[0]; // convert the 0 index float value to integer
            
            synchronized(LCD.class)
            {
                LCD.clear(); // Clear the LCD screen
                // Light intensity display
                String lightMsg = "Light: " + (int)(lighIntensitys[0] * 100) + "%";
                TextWrap(lightMsg, 0); // start from line 0
                String colorName = getcolorName(colorDetector);
                // Color name display
                String colorMsg = "Color: " + colorName;
                TextWrap(colorMsg, 2); // start from line 2 (leave one line space)
            
            }

            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        colorSensor.close();
    }

    public static String getcolorName(int cD)  // passind the colorDetector value in cD variable
    {
        switch(cD) // switch the Cd values
        {
            case 0:
                    return "Red";
            case 1: 
                    return "Green";
            case 2:
                    return "Blue";
            case 3: 
                    return "Yellow";
            case 4: 
                    return "Magenta";
            case 5: 
                    return "Orange";
            case 6: 
                    return "White";
            case 7: 
                    return "Black";
            default:
                    return "This id color is not mentioned.";
  }
}

        public static void TextWrap(String msg, int startY) 
        {
                int maxLength = 16; // max chars per line on EV3 LCD
                int localX = 0;
                int y = startY;
    
                for (int i = 0; i < msg.length(); i += maxLength) 
                {
                        String line = msg.substring(i, Math.min(i + maxLength, msg.length()));
                        LCD.drawString(line, localX, y++);
                }
        }
    

}