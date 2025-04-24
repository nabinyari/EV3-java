package main;

    public class dataShare {
        private float lightIntensity;
        private float blackvalue;
        private float whitevalue;
    
        public void setIntensity(float value) 
        {
            this.lightIntensity = value;
        }
        public float getIntensity() 
        {
            return lightIntensity;
        }
        public void setBlack(float bvalue) 
        {
            this.blackvalue = bvalue;
        }
        public float getBlack() 
        {
            return blackvalue;
        }
        public void setWhite(float wvalue) 
        {
            this.whitevalue = wvalue;
        }
        public float getWhite() 
        {
            return whitevalue;
        }
    }
