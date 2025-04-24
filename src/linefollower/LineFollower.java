package linefollower;

import lejos.hardware.motor.Motor;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.utility.Delay;

public class LineFollower implements Runnable {
    private final dataShare Data;
    private final PIDController pid;
    private static final int BASE_SPEED = 250;
    private static final int MAX_SPEED = 500;
    private static final int MIN_SPEED = 100;
    private static final int DELAY_MS = 20;
    private static final float CURVE_ADJUSTMENT = 1.8f; // Increased for better curve handling

    public LineFollower(dataShare Data) {
        this.Data = Data;
        float threshold = Data.getThreshold();
        this.pid = new PIDController(1.8f, 0.02f, 0.8f, threshold); // Tuned PID values
    }

    public void run() {
        try {
            initializeMotors();
            followLine();
        } finally {
            Motor.A.stop();
            Motor.B.stop();
        }
    }

    private void initializeMotors() {
        Motor.A.setSpeed(BASE_SPEED);
        Motor.B.setSpeed(BASE_SPEED);
        Motor.A.forward();
        Motor.B.forward();
    }

    private void followLine() {
        while (!Button.ESCAPE.isDown()) {
            float lightSample = Data.getIntensity();
            float correctionValue = pid.calculatePID(lightSample);
            
            // Enhanced curve handling with dynamic speed adjustment
            adjustMotorSpeeds(correctionValue * CURVE_ADJUSTMENT);
            
            Delay.msDelay(DELAY_MS);
        }
    }

    private void adjustMotorSpeeds(float scaledCorrection) {
        int speedA = (int) Math.max(MIN_SPEED, Math.min(BASE_SPEED + scaledCorrection, MAX_SPEED));
        int speedB = (int) Math.max(MIN_SPEED, Math.min(BASE_SPEED - scaledCorrection, MAX_SPEED));
        
        Motor.A.setSpeed(speedA);
        Motor.B.setSpeed(speedB);
        
        // Maintain forward motion
        if (speedA > 0) Motor.A.forward();
        else Motor.A.backward();
        
        if (speedB > 0) Motor.B.forward();
        else Motor.B.backward();
    }
}

class PIDController {
    private final float Kp;
    private final float Ki;
    private final float Kd;
    private final float setPoint;
    private float integral;
    private float previousError;
    private static final float INTEGRAL_LIMIT = 100; // Anti-windup

    public PIDController(float Kp, float Ki, float Kd, float setPoint) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.setPoint = setPoint;
    }

    float calculatePID(float value) {
        float error = setPoint - value;
        
        // Anti-windup for integral term
        integral += error;
        if (Math.abs(integral) > INTEGRAL_LIMIT) {
            integral = INTEGRAL_LIMIT * Math.signum(integral);
        }
        
        float derivative = error - previousError;
        previousError = error;
        
        return Kp * error + Ki * integral + Kd * derivative;
    }
}