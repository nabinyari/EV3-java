package main;

public class Main {
    public static void main(String[] args) {
        datashare sharedData = new datashare(); 
        
         // Set up the threads for line following and ultrasonic sensor
        Thread t1 = new Thread(new LineFollower(sharedData));
        Thread t2 = new Thread(new ultrasonic(sharedData));

        // Start both threads
        t1.start();
        t2.start();
    }
}
