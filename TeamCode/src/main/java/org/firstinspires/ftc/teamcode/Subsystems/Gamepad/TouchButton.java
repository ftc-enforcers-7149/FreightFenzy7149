package org.firstinspires.ftc.teamcode.Subsystems.Gamepad;


public class TouchButton {

    Touchpad touchpad;
    String name;
    double leftX, rightX, bottomY, topY;
    private VectorPacket v1 = new VectorPacket(), v2 = new VectorPacket();

    public TouchButton(String name, double leftX, double rightX, double bottomY, double topY) {

        this.name = name;
        this.leftX = leftX;
        this.rightX = rightX;
        this.bottomY = bottomY;
        this.topY = topY;

    }

    public void setTouchButton(Touchpad touch) {
        touchpad = touch;
    }

    public boolean isRange() {

        switch(touchpad.getNumFingers()) {
            case 1:
                if((touchpad.getFingerOneX() <= rightX && touchpad.getFingerOneX() >= leftX)
                        && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY))
                    return true;
                break;

            case 2:
                if(((touchpad.getFingerOneX() <= rightX && touchpad.getFingerOneX() >= leftX)
                        && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY))
                        ||
                        ((touchpad.getFingerTwoX() <= rightX && touchpad.getFingerTwoX() >= leftX)
                                && (touchpad.getFingerTwoY() <= topY && touchpad.getFingerTwoY() >= bottomY)))
                    return true;
                break;
            default:
                return false;
        }

        return false;

    }

    public String getName() {
        return name;
    }

    public boolean isRange(double x, double y) {

        if((x <= rightX && x >= leftX)
                && (y <= topY && y >= bottomY)) {
            return true;
        }

        return false;

    }

    public VectorPacket getV1() {
        return v1;
    }

    public VectorPacket getV2() {
        return v2;
    }

    public class VectorPacket {

        private double x, y, lastX, lastY, time, lastTime;

        public VectorPacket(double x, double y, double lastX, double lastY, double time, double lastTime) {

            this.x = x; this.y = y; this.lastX = lastX; this.lastY = lastY; this.time = time; this.lastTime = lastTime;

        }

        public VectorPacket() {
            x = 0; y = 0; lastX = 0; lastY = 0; time = 0; lastTime = 0;
        }

        public void updateVector(double x, double y, double time) {
            lastX = this.x; lastY = this.y; lastTime = this.time;
            this.x = x; this.y = y; this.time = time;
        }

        public double getDistance() {
            return Math.sqrt((Math.pow((x - lastX), 2) + Math.pow((y - lastY), 2)));
        }

        public double getVelocity() {
            return Math.sqrt(Math.pow(getXVel(), 2) + Math.pow(getYVel(), 2));
        }

        public double getAngle() {
            return Math.toDegrees(Math.atan((y - lastY) /(x - lastX)));
        }

        public void setX(double x) {
            this.x = x;
        }

        public void setLastX(double lastX) {
            this.lastX  = x;
        }

        public void setY(double y) {
            this.y = y;
        }

        public void setLastY(double lastY) {
            this.lastY  = y;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getXVel() {
            return (x - lastX) / (time - lastTime);
        }

        public double getYVel() {
            return (y - lastY) / (time - lastTime);
        }

        public double getTime() {
            return time;
        }

        public void setTime(double time) {
            this.time = time;
        }

        public double getLastTime() {
            return lastTime;
        }

        public void setLastTime(double lastTime) {
            this.lastTime = lastTime;
        }

    }

}

/*
    public VectorPacket swipeVector() {

        VectorPacket v = new VectorPacket();
        boolean fingerOneInRange, fingerTwoInRange;

        switch(touchpad.getNumFingers()) {

            case 1:

                fingerOneInRange = (touchpad.getFingerOneX() <= rightX && touchpad.getFingerOneX() >= leftX)
                        && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY);

                if(fingerOneInRange) {

                    v.setTime(time);
                    v.setLastTime(lastTime);
                    v.setX(touchpad.getFingerOneX());
                    v.setY(touchpad.getFingerOneY());
                    v.setLastX(touchpad.getLastFingerOneX());
                    v.setLastY(touchpad.getLastFingerOneY());

                }
                break;

            case 2:

                //check for both first
                fingerOneInRange = (touchpad.getFingerOneX() <= rightX && touchpad.getFingerOneX() >= leftX)
                        && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY);
                fingerTwoInRange = ((touchpad.getFingerTwoX() <= rightX && touchpad.getFingerTwoX() >= leftX)
                        && (touchpad.getFingerTwoY() <= topY && touchpad.getFingerTwoY() >= bottomY));

                if(fingerOneInRange && fingerTwoInRange) {

                    v.setTime(time);
                    v.setLastTime(lastTime);
                    v.setX(touchpad.getFingerOneX());
                    v.setY(touchpad.getFingerOneY());
                    v.setLastX(touchpad.getLastFingerOneX());
                    v.setLastY(touchpad.getLastFingerOneY());

                }
                else if (fingerTwoInRange) {

                    v.setTime(time);
                    v.setLastTime(lastTime);
                    v.setX(touchpad.getFingerTwoX());
                    v.setY(touchpad.getFingerTwoY());
                    v.setLastX(touchpad.getLastFingerTwoX());
                    v.setLastY(touchpad.getLastFingerTwoY());

                }
                else if(fingerOneInRange) {

                    v.setTime(time);
                    v.setLastTime(lastTime);
                    v.setX((Math.abs(touchpad.getFingerTwoX() - touchpad.getFingerOneX())) / 2);
                    v.setY((Math.abs(touchpad.getFingerTwoY() - touchpad.getFingerOneY())) / 2);
                    v.setLastX((Math.abs(touchpad.getLastFingerTwoX() - touchpad.getLastFingerOneX())) / 2);
                    v.setLastY((Math.abs(touchpad.getLastFingerTwoY() - touchpad.getLastFingerOneY())) / 2);

                }

                break;

            default:
                break;

        }

        return v;

    }*/