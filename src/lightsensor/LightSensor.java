package lightsensor;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;

public class LightSensor{
    public static void main(String[] args)
    {
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);

        SampleProvider ambientSample = colorSensor.getAmbientMode(); // using the getAmbientMode() too measures ambient light level 
        // Create an array to hold the sensor data
        float[] ambientSample1 = new float[ambientSample.sampleSize()];

        SampleProvider colorId = colorSensor.getColorIDMode(); // using the getColorIDMode() for detection of basic color as ID
        float[] colorSample2 = new float[colorId.sampleSize()];

        while (!Button.ESCAPE.isDown()) // Exit if the ESCAPE button is pressed
        {
            // Get the current light intensity reading from the sensor
            ambientSample.fetchSample(ambientSample1, 0);

            // Get the current color Id reading from the sensor
            colorId.fetchSample(colorSample2, 0);
            int colorDetector = (int)colorSample2[0]; // convert the 0 index float value to integer

            LCD.clear(); // Clear the LCD screen
            LCD.drawString("Light Intensity: " + (int)(ambientSample1[0] * 100) + "%", 0, 0); // Display the light intensity value on the LCD screen as percentage
            String colorName = getcolorName(colorDetector);
            LCD.drawString("Color name: " + colorName, 0, 1);


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

}