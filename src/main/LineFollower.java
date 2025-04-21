package main;

import lejos.hardware.motor.Motor;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.utility.Delay;

public class LineFollower implements Runnable{
    dataShare Data;
    lineSearch search;

    public LineFollower(dataShare Data) {
        this.Data = Data;
        this.search = new lineSearch(Data);
    }

    public void run() 
    { 
        float blackValue = 0.1f; // sensor value on black line
        float whiteValue = 0.6f; // sensor value on white surface

        // The threshold is the midpoint between black and white light values
        float threshold = (blackValue + whiteValue) / 2;

        // Initialize PID controller with constants (proportional, integral, derivative) and the target threshold
        PIDController pid = new PIDController(1.2f, 0f, 0.3f, threshold);

        // Set the motor speeds for a forward motion
        Motor.A.setSpeed(300);
        Motor.B.setSpeed(150);
        Motor.A.forward();
        Motor.B.forward();
        int lostCount = 0;

        // Main loop: runs until the ESCAPE button is pressed
        while (!Button.ESCAPE.isDown()) 
        {
            float lightSample = Data.getIntensity();
            // Display the current light intensity and threshold value on the LCD screen for debugging
            synchronized(LCD.class)
            {
            LCD.clear();
            LCD.drawString("Threshold: " + (int)(threshold * 100) + "%", 0, 1);
            }

            if (lightSample < 0.05f || lightSample > 0.7f) {
                lostCount++;
            } else {
                lostCount = 0;
            }

            if (lostCount > 5) {
                search.revolveSearch();
                lostCount = 0;
                continue;
            }
        
            // Use the PID controller to calculate the correction value for the motors
            float correctionValue = pid.calculatePID(lightSample);

            int baseSpeed = 200;
            int maxSpeed = 400;
            
            Motor.A.forward();
            Motor.B.forward();

            // Adjust the motor speeds based on the correction from the PID controller
            Motor.A.setSpeed(Math.max(0, Math.min(baseSpeed + correctionValue, maxSpeed)));
            Motor.B.setSpeed(Math.max(0, Math.min(baseSpeed - correctionValue, maxSpeed)));
            

            // making delay to prevent excessive data updates to motors
            Delay.msDelay(20);
        }
        // Stop the motors when the ESCAPE button is pressed
        Motor.A.stop();
        Motor.B.stop();
    }
}

class lineSearch 
{
    dataShare Data;

    public lineSearch(dataShare Data) {
        this.Data = Data;
    }

    public void revolveSearch() {
        int speed = 200;

        // Continuously rotate in place to the left
        Motor.A.setSpeed(speed);
        Motor.B.setSpeed(speed);
        Motor.A.backward();
        Motor.B.forward();

        while (!Button.ESCAPE.isDown()) {
            float sample = Data.getIntensity();

            // If the line is found, stop and return
            if (sample < 0.3f && sample > 0.05f) {
                Motor.A.stop();
                Motor.B.stop();
                return;
            }

            Delay.msDelay(50); // Small delay to reduce sensor flooding
        }

        // Stop if ESCAPE is pressed
        Motor.A.stop();
        Motor.B.stop();
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
