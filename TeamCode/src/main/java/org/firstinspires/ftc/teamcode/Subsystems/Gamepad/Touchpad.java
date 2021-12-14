package org.firstinspires.ftc.teamcode.Subsystems.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

public class Touchpad {

    //TODO: fix angle handling

    // Gamepad for handling input
    private Gamepad gamepad;

    // touchButton: touchpad "press"
    // fingerOn: true - finger on the touchpad

    private boolean touchButton, lastTouchButton, fingerOn, lastFingerOn;

    // number of fingers on the touchpad
    private int numFingers, lastNumFingers;

    // positional variables
    private double fingerOneX, lastFingerOneX = 0, fingerOneY, lastFingerOneY = 0,
            fingerTwoX, lastFingerTwoX = 0, fingerTwoY, lastFingerTwoY = 0;

    // time variables
    double time, lastTime = 0;

    // how often the touchpad "updates"
    double pollingTime = 200;

    // vector packets for storing velocity data
    private VectorPacket v1 = new VectorPacket(), v2 = new VectorPacket();

    // standard unit multiplier
    private double standardMult = 100;

    // storage for touchButtons (zoned "buttons" on the gamepad)
    ArrayList<TouchObject> touchObjects = new ArrayList<>();

    // finger deadzone
    double fingerDeadzone = 2;


    // Constructor. pass in the tele-op gamepad

    public Touchpad(Gamepad gamepad) {

        this.gamepad = gamepad;

    }

    // Update method

    public void update() {

        // Updates time

        time = System.currentTimeMillis();

        // If we've hit the poll time

        if(time - lastTime >= 200) {

            // Checks if touchpad press

            touchButton = gamepad.touchpad;

            // Checks how many fingers are on the touchpad

            if (gamepad.touchpad_finger_1 && gamepad.touchpad_finger_2) {
                numFingers = 2;
                fingerOn = true;
            } else if (gamepad.touchpad_finger_1) {
                numFingers = 1;
                fingerOn = true;
            } else {
                numFingers = 0;
                fingerOn = false;
            }

            // Checks the position of fingers and updates

            if (numFingers >= 1) {
                if(Math.abs((standardMult * gamepad.touchpad_finger_1_x) - lastFingerOneX) > fingerDeadzone)
                    fingerOneX = Math.round(standardMult * gamepad.touchpad_finger_1_x);

                if(Math.abs((standardMult * gamepad.touchpad_finger_1_y) - lastFingerOneY) > fingerDeadzone)
                    fingerOneY = Math.round(standardMult * gamepad.touchpad_finger_1_y);
            }

            if (numFingers == 2) {
                if(Math.abs((standardMult * gamepad.touchpad_finger_2_x) - lastFingerTwoX) > fingerDeadzone)
                    fingerTwoX = Math.round(standardMult * gamepad.touchpad_finger_2_x);

                if(Math.abs((standardMult * gamepad.touchpad_finger_2_y) - lastFingerOneY) > fingerDeadzone)
                    fingerTwoY = Math.round(standardMult * gamepad.touchpad_finger_2_y);
            }

            // Updates the vectors of the fingers if applicable

            switch(numFingers) {

                case 1:
                    v1 = new VectorPacket(fingerOneX, fingerOneY, lastFingerOneX, lastFingerOneY, time, lastTime);
                    v2 = new VectorPacket();
                    break;

                case 2:
                    v1 = new VectorPacket(fingerOneX, fingerOneY, lastFingerOneX, lastFingerOneY, time / 1000, lastTime / 1000);
                    v2 = new VectorPacket(fingerTwoX, fingerTwoY, lastFingerTwoX, lastFingerTwoY, time / 1000, lastTime / 1000);
                    break;

                default:
                    v1 = new VectorPacket();
                    v2 = new VectorPacket();
                    break;
            }

            //Updates lasts

            if (numFingers >= 1) {
                lastFingerOneX = fingerOneX;
                lastFingerOneY = fingerOneY;
            }
            if (numFingers == 2) {
                lastFingerTwoX = fingerTwoX;
                lastFingerTwoY = fingerTwoY;
            }

            lastTouchButton = touchButton;
            lastNumFingers = numFingers;
            lastFingerOn = fingerOn;
            lastTime = time;
        }

    }


    /*
    // TODO: implement TouchZone support
    public boolean swipe (int finger, SwipeType s, String name) {
        return false;
    }

    // TODO: implement finger two
    public double axis(int finger, SwipeType s, Scale scale) {

        switch(s) {

            case HORIZ_AXIS:

                return scale.output(fingerOneX);

            case VERT_AXIS:
                return scale.output(fingerOneY);

            default:
                return 0;
        }

    }*/

    /*public double axis(int finger, SwipeType s, String name) {

        return 0;

    }*/

    // Adds a TouchButton to the map of touchbuttons

    public void add(TouchObject implement) {

        touchObjects.add(implement);

    }

    public TouchObject getObject(String name) {

        for(int i = 0; i < touchObjects.size(); i++) {

            if(touchObjects.get(i).getName().equals(name)) return touchObjects.get(i);

        }

        return null;

    }

    // Returns the HashMap of TouchButtons

    public ArrayList<TouchObject> getTouchObjects() {
        return touchObjects;
    }

    // rumble handling functions

    public void rumble (Gamepad.RumbleEffect rumbleEffect) { gamepad.runRumbleEffect(rumbleEffect); }
    public void rumble (int duration) {
        gamepad.rumble(duration);
    }
    public void rumble(double rumble1, double rumble2, int duration) { gamepad.rumble(rumble1, rumble2, duration); }
    public void rumbleBlips(int blips) {
        gamepad.rumbleBlips(blips);
    }
    public void startRumble() {
        gamepad.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
    }
    public void stopRumble() {
        gamepad.stopRumble();
    }

    // standard getter/setters
    public boolean isTouchButton() {
        return touchButton;
    }
    public void setTouchButton(boolean touchButton) {
        this.touchButton = touchButton;
    }
    public boolean isLastTouchButton() {
        return lastTouchButton;
    }
    public void setLastTouchButton(boolean lastTouchButton) { this.lastTouchButton = lastTouchButton; }
    public int getNumFingers() {
        return numFingers;
    }
    public void setNumFingers(int numFingers) {
        this.numFingers = numFingers;
    }
    public double getStandardMult() {
        return standardMult;
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
    public double getFingerTwoX() { return fingerTwoX; }
    public void setFingerTwoX(double fingerTwoX) { this.fingerTwoX = fingerTwoX; }
    public double getLastFingerTwoX() { return lastFingerTwoX; }
    public void setLastFingerTwoX(double lastFingerTwoX) { this.lastFingerTwoX = lastFingerTwoX; }
    public double getFingerTwoY() { return fingerTwoY; }
    public void setFingerTwoY(double fingerTwoY) { this.fingerTwoY = fingerTwoY; }
    public double getLastFingerTwoY() { return lastFingerTwoY; }
    public void setLastFingerTwoY(double lastFingerTwoY) { this.lastFingerTwoY = lastFingerTwoY; }
    public boolean getLastFingerOn() { return lastFingerOn; }
    public boolean getFingerOn() { return fingerOn; }

    // Returns the finger vector packets

    public VectorPacket getV1() {
        return v1;
    }

    public VectorPacket getV2() {
        return v2;
    }

    // Vector packet handling class. Sure it may be redundant, but it's fun!

    public class VectorPacket {

        // Important storage variables

        private double x, y, lastX, lastY, time, lastTime;

        // deadzone var; if less than, v = 0;
        private double deadzone = .005;

        // Standard constructor when there's a starting point

        public VectorPacket(double x, double y, double lastX, double lastY, double time, double lastTime) {

            this.x = x; this.y = y; this.lastX = lastX; this.lastY = lastY; this.time = time; this.lastTime = lastTime;

        }

        // Zeroed constructor

        public VectorPacket() {
            x = 0; y = 0; lastX = 0; lastY = 0; time = 0; lastTime = 0;
        }

        public double getDistance() {
            return Math.sqrt((Math.pow((x - lastX), 2) + Math.pow((y - lastY), 2)));
        }

        public double getVelocity() {

            double velocity = Math.sqrt(Math.pow(getXVel(), 2) + Math.pow(getYVel(), 2));
            return velocity > deadzone ? velocity : 0;

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

        public double getLastX() {
            return lastX;
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