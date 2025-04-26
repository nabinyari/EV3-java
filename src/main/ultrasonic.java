package main;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;

public class ultrasonic implements Runnable{
    datashare Data;

    public ultrasonic(datashare Data) {
        this.Data = Data;
    }

    @Override
    public void run() {
        // Initialize ultrasonic sensor at port S2
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
        SampleProvider distance = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distance.sampleSize()];

        // Loop until ESCAPE button is pressed
        while (!Button.ESCAPE.isDown()) {
            // Read distance continuously
            distance.fetchSample(sample, 0);
            float distanceMeters = sample[0];  // Distance in meters
            Data.setDistance(distanceMeters);
            // Refresh every 100 ms
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        ultrasonicSensor.close();
    }
}
