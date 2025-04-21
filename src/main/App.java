package main;

public class App {
    public static void main(String[] args) {
        dataShare sharedData = new dataShare(); // one shared instance
        
        // Create sensor and follower threads, passing the same sharedData
        Thread t1 = new Thread(new LightSensor(sharedData));
        Thread t2 = new Thread(new LineFollower(sharedData));

        // Start both threads
        t1.start();
        t2.start();
    }
}
