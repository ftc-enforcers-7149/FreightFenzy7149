package org.firstinspires.ftc.teamcode.Subsystems.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Touchpad {

    private Gamepad gamepad;

    private boolean touchButton, lastTouchButton, fingerOn, lastFingerOn;
    private int numFingers, lastNumFingers;

    private double fingerOneX, lastFingerOneX, fingerOneY, lastFingerOneY,
            fingerTwoX, lastFingerTwoX, fingerTwoY, lastFingerTwoY;

    double time, lastTime = 0;
    double pollingTime;
    private VectorPacket v1 = new VectorPacket(), v2 = new VectorPacket();
    private double standardMult = 100;

    public Touchpad(Gamepad gamepad) {

        this.gamepad = gamepad;

    }

    public void update() {

        time = System.currentTimeMillis();
        getVelocity();

        touchButton = gamepad.touchpad;

        if(gamepad.touchpad_finger_1 && gamepad.touchpad_finger_2) {
            numFingers = 2;
            lastFingerOn = fingerOn;
            fingerOn = true;
        }
        else if(gamepad.touchpad_finger_1) {
            numFingers = 1;
            lastFingerOn = fingerOn;
            fingerOn = true;
        }
        else {
            numFingers = 0;
            lastFingerOn = fingerOn;
            fingerOn = false;
        }

        if(numFingers >= 1) {
            lastFingerOneX = fingerOneX;
            fingerOneX = standardMult * gamepad.touchpad_finger_1_x;
            lastFingerOneY = fingerOneY;
            fingerOneY = standardMult * gamepad.touchpad_finger_1_y;
        }

        if(numFingers == 2) {
            lastFingerTwoX = fingerTwoX;
            fingerTwoX = standardMult * gamepad.touchpad_finger_2_x;
            lastFingerTwoY = fingerTwoY;
            fingerTwoY = standardMult * gamepad.touchpad_finger_2_y;
        }

        lastTouchButton = touchButton;
        lastNumFingers = numFingers;
        lastTime = time;

    }

    public void rumble (Gamepad.RumbleEffect rumbleEffect) {
        gamepad.runRumbleEffect(rumbleEffect);
    }

    public void rumble (int duration) {
        gamepad.rumble(duration);
    }

    public void rumble(double rumble1, double rumble2, int duration) {
        gamepad.rumble(rumble1, rumble2, duration);
    }

    public void rumbleBlips(int blips) {
        gamepad.rumbleBlips(blips);
    }

    public void startRumble() {
        gamepad.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
    }

    public void stopRumble() {
        gamepad.stopRumble();
    }

    public boolean isTouchButton() {
        return touchButton;
    }

    public void setTouchButton(boolean touchButton) {
        this.touchButton = touchButton;
    }

    public boolean isLastTouchButton() {
        return lastTouchButton;
    }

    public void setLastTouchButton(boolean lastTouchButton) {
        this.lastTouchButton = lastTouchButton;
    }

    public int getNumFingers() {
        return numFingers;
    }

    public void setNumFingers(int numFingers) {
        this.numFingers = numFingers;
    }

    public int getLastNumFingers() {
        return lastNumFingers;
    }

    public void setLastNumFingers(int lastNumFingers) {
        this.lastNumFingers = lastNumFingers;
    }

    public double getFingerOneX() {
        return fingerOneX;
    }

    public void setFingerOneX(double fingerOneX) {
        this.fingerOneX = fingerOneX;
    }

    public double getLastFingerOneX() {
        return lastFingerOneX;
    }

    public void setLastFingerOneX(double lastFingerOneX) {
        this.lastFingerOneX = lastFingerOneX;
    }

    public double getFingerOneY() {
        return fingerOneY;
    }

    public void setFingerOneY(double fingerOneY) {
        this.fingerOneY = fingerOneY;
    }

    public double getLastFingerOneY() {
        return lastFingerOneY;
    }

    public void setLastFingerOneY(double lastFingerOneY) {
        this.lastFingerOneY = lastFingerOneY;
    }
//
    public double getFingerTwoX() {
        return fingerTwoX;
    }

    public void setFingerTwoX(double fingerTwoX) {
        this.fingerTwoX = fingerTwoX;
    }

    public double getLastFingerTwoX() {
        return lastFingerTwoX;
    }

    public void setLastFingerTwoX(double lastFingerTwoX) {
        this.lastFingerTwoX = lastFingerTwoX;
    }

    public double getFingerTwoY() {
        return fingerTwoY;
    }

    public void setFingerTwoY(double fingerTwoY) {
        this.fingerTwoY = fingerTwoY;
    }

    public double getLastFingerTwoY() {
        return lastFingerTwoY;
    }

    public void setLastFingerTwoY(double lastFingerTwoY) {
        this.lastFingerTwoY = lastFingerTwoY;
    }

    public boolean getLastFingerOn() {
        return lastFingerOn;
    }

    public boolean getFingerOn() {
        return fingerOn;
    }

    /*public boolean isRange() {

        switch(numFingers) {
            case 1:
                if((fingerOneX <= rightX && gFun.getFingerOneX() >= leftX)
                        && (gFun.getFingerOneY() <= topY && gFun.getFingerOneY() >= bottomY))
                    return true;
                break;

            case 2:
                if(((gFun.getFingerOneX() <= rightX && gFun.getFingerOneX() >= leftX)
                        && (gFun.getFingerOneY() <= topY && gFun.getFingerOneY() >= bottomY))
                        ||
                        ((gFun.getFingerTwoX() <= rightX && gFun.getFingerTwoX() >= leftX)
                                && (gFun.getFingerTwoY() <= topY && gFun.getFingerTwoY() >= bottomY)))
                    return true;
                break;
            default:
                return false;
        }

        return false;

    }*/

    public boolean getVelocity() {

        switch(numFingers) {

            case 1:
                v1.updateVector(fingerOneX, fingerOneY, time - lastTime);
                v2.updateVector(0, 0, time - lastTime);
                break;

            case 2:

                v1.updateVector(fingerOneX, fingerOneY, time - lastTime);
                v2.updateVector(fingerTwoX, fingerTwoY, time - lastTime);
                break;

            default:
                v1.updateVector(0, 0, time-lastTime);
                v2.updateVector(0, 0, time-lastTime);
                break;
        }

        return Math.abs(v1.getVelocity()) > 0 || Math.abs(v2.getVelocity()) > 0;

    }

/*    public boolean isRange(double x, double y) {

        if((x <= rightX && x >= leftX)
                && (y <= topY && y >= bottomY)) {
            return true;
        }

        return false;

    }*/

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

        public void updateVector(double newX, double newY, double newTime) {
            lastX = this.x; lastY = this.y; lastTime = this.time;
            this.x = newX; this.y = newY; this.time = newTime;
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
