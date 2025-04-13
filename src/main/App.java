package main;
public class App {
    public static void main(String[] args) {
        LightSensor light = new LightSensor();
        LineFollower line = new LineFollower();

        Thread t1 = new Thread(light);
        Thread t2 = new Thread(line);

        t1.start();
        t2.start();
    }
}
