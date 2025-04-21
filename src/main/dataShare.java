package main;

public class dataShare {
    private float lightIntensity;

    public synchronized void setIntensity(float value) {
        this.lightIntensity = value;
    }

    public synchronized float getIntensity() {
        return lightIntensity;
    }

}
