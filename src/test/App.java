package test;

public class App {
    
        public static void main(String[] args) {
            
        Thread t1 = new Thread(new LightSensor());

        // Start both threads
        t1.start();
    }
}
