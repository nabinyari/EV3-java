package main;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower implements Runnable{
    public void run() 
    { 
        lineSearch search = new lineSearch();
        // creating colorSensor named object and assigined S4 port
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
        // Set the sensor to read the reflected red light intensity
        SampleProvider light = colorSensor.getRedMode();
        float[] lightSample = new float[light.sampleSize()];

        float blackValue = 0.1f; // sensor value on black line
        float whiteValue = 0.6f; // sensor value on white surface

        // The threshold is the midpoint between black and white light values
        float threshold = (blackValue + whiteValue) / 2;

        // Initialize PID controller with constants (proportional, integral, derivative) and the target threshold
        PIDController pid = new PIDController(0.8f, 0.01f, 0.3f, threshold);

        // Set the motor speeds for a forward motion
        Motor.A.setSpeed(300);
        Motor.B.setSpeed(150);
        Motor.A.forward();
        Motor.B.forward();

        // Main loop: runs until the ESCAPE button is pressed
        while (!Button.ESCAPE.isDown()) 
        {
            // Fetch the current light intensity reading from the sensor
            light.fetchSample(lightSample, 0);

            // Display the current light intensity and threshold value on the LCD screen for debugging
            LCD.clear();
            LCD.drawString("Light: " + (int)(lightSample[0] * 100) + "%", 0, 0);
            LCD.drawString("Threshold: " + (int)(threshold * 100) + "%", 0, 1);
            
            if( lightSample[0] > 0.2f)
            {
                search.spiralSearch(colorSensor, light);
                continue;
            }
            // Use the PID controller to calculate the correction value for the motors
            float correctionValue = pid.calculatePID(lightSample[0]);

            //setting the max speed
            int maxSpeed = 500;
            Motor.A.forward();
            Motor.B.forward();
            // Adjust the motor speeds based on the correction from the PID controller
            Motor.A.setSpeed(Math.min(300 + correctionValue, maxSpeed));
            Motor.B.setSpeed(Math.min(300 - correctionValue, maxSpeed));

            // making delay to prevent excessive data updates to motors
            Delay.msDelay(50);
        }
        // Stop the motors when the ESCAPE button is pressed
        Motor.A.stop();
        Motor.B.stop();

        colorSensor.close();
    }
}

class lineSearch 
{
    public void spiralSearch(EV3ColorSensor sensor, SampleProvider mode)
    {
        float[] sample = new float[mode.sampleSize()];

        int firstSearchRadius = 100; // setting the speed to search 1st radius 
        int lastSearchRadius = 500;  // setting the speed up to 500 to search the radius

        while (!Button.ESCAPE.isDown())
        {
            mode.fetchSample(sample, 0);
            // moving the motor in round pattern 
            Motor.A.forward();
            Motor.B.backward();
            Motor.A.setSpeed(firstSearchRadius);
            Motor.B.setSpeed(firstSearchRadius);

            if(sample[0] < 0.3f && sample[0] > 0.05f) // giving the condition if the sensor detect the black like 
            {
                Motor.A.stop();
                Motor.B.stop();
                return; // Exit the method
            }

            firstSearchRadius = firstSearchRadius + 50; // if not then increasing speed by 50 

            if (firstSearchRadius > lastSearchRadius) 
            {
                firstSearchRadius = 100; // again making the speed to it's normal
            }

            Delay.msDelay(100);
        }
    }
}

class PIDController
{
    float Kp;
    float Ki;
    float Kd;
    float setPoint;
    float integral;
    float enderror;

    // creating a constructor that set the value to calculate pid
    public PIDController( float Kp, float Ki, float Kd, float setpoint)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.setPoint = setpoint;
    }
    // creating the method that to calculate the correction based on the current sensor value
    float calculatePID(float value)
    {
        // Calculate the error (difference between set point and current value)
        float error = setPoint - value;

        // Update the integral (sum of errors over time)
        integral = integral + error;

        // Calculate the derivative (rate of change of the error)
        float derivative = error - enderror;
        
        // assine the current error for the next iteration
        enderror = error;

        float correctionValue = Kp * error + Ki * integral + Kd * derivative;
        return correctionValue; // Return the PID correction value
    }
}
