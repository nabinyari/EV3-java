package main;

public class datashare {
    private float distance;

    public synchronized void setDistance(float value) {
        this.distance = value;
    }

    public synchronized float getDistance() {
        return distance;
    }
}