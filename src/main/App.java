package main;
public class App {
    public static void main(String[] args) {
        LightSensor light = new LightSensor();
        LineFollower line = new LineFollower();
        UltrasonicSensor ultra = new UltrasonicSensor();

        Thread t1 = new Thread(light);
        Thread t2 = new Thread(line);
        Thread t3 = new Thread(ultra);

        t1.start();
        t2.start();
        t3.start();
    }
}
