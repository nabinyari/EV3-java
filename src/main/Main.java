package main;

public class Main {
    public static void main(String[] args) {
        datashare sharedData = new datashare(); // one shared instance
        
        // Create sensor and follower threads, passing the same sharedData
        Thread t1 = new Thread(new LineFollower(sharedData));
        Thread t2 = new Thread(new ultrasonic(sharedData));

        // Start both threads
        t1.start();
        t2.start();
    }
}
