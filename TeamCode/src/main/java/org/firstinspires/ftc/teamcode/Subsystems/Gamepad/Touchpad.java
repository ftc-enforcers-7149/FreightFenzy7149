package org.firstinspires.ftc.teamcode.Subsystems.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Point;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public class Touchpad implements Input {

    //TODO: fix angle handling

    // Gamepad for handling input
    private Gamepad gamepad;

    // touchButton: touchpad "press"
    // fingerOn: true - finger on the touchpad
    private boolean touchButton, lastTouchButton, fingerOn, lastFingerOn;

    // number of fingers on the touchpad
    private int numFingers, lastNumFingers;

    // positional variables
    private Point fingerOne, lastFingerOne, fingerTwo, lastFingerTwo;

    private double lastTime = 0;

    // how often the touchpad "updates"
    private final double pollingTime = 200;

    // vector packets for storing velocity data
    private VectorPacket v1 = new VectorPacket(), v2 = new VectorPacket();

    // standard unit multiplier
    private final double standardMult = 100;

    // finger deadzone
    private final double fingerDeadzone = 2;

    // Constructor. pass in the tele-op gamepad
    public Touchpad(Gamepad gamepad) {
        this.gamepad = gamepad;

        fingerOne = new Point(); lastFingerOne = new Point();
        fingerTwo = new Point(); lastFingerTwo = new Point();
    }

    // Update method
    @Override
    public void updateInput() {

        // Updates time

        // time variables
        double time = System.currentTimeMillis();

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
            double f1X = Math.round(10 * standardMult * gamepad.touchpad_finger_1_x) / 10.0;
            double f1Y = Math.round(10 * standardMult * gamepad.touchpad_finger_1_y) / 10.0;

            fingerOne.setPoint(f1X, f1Y);

            if (numFingers == 2) {
                double f2X = Math.round(10 * standardMult * gamepad.touchpad_finger_2_x) / 10.0;
                double f2Y = Math.round(10 * standardMult * gamepad.touchpad_finger_2_y) / 10.0;

                fingerTwo.setPoint(f2X, f2Y);
            }
        }

        // If we've hit the poll time
        if(time - lastTime >= pollingTime) {
            // Updates the vectors of the fingers if applicable

            switch(numFingers) {

                case 1:
                    v1 = new VectorPacket(fingerOne, lastFingerOne, time, lastTime);
                    v2 = new VectorPacket();
                    break;

                case 2:
                    v1 = new VectorPacket(fingerOne, lastFingerOne, time / 1000, lastTime / 1000);
                    v2 = new VectorPacket(fingerTwo, lastFingerTwo, time / 1000, lastTime / 1000);
                    break;

                default:
                    v1 = new VectorPacket();
                    v2 = new VectorPacket();
                    break;
            }

            //Updates lasts
            lastTime = time;

        }

        if (numFingers >= 1) {
            lastFingerOne.setPoint(fingerOne);
        }
        if (numFingers == 2) {
            lastFingerTwo.setPoint(fingerTwo);
        }

        lastTouchButton = touchButton;
        lastNumFingers = numFingers;
        lastFingerOn = fingerOn;

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
    public boolean isTouchButton() { return touchButton; }
    public int getNumFingers() { return numFingers; }
    public Point getFingerOne() { return fingerOne; }
    public Point getLastFingerOne() { return lastFingerOne; }
    public Point getFingerTwo() { return fingerTwo; }
    public Point getLastFingerTwo() { return lastFingerTwo; }
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
        private Point pos, lastPos;
        private double time, lastTime;

        // deadzone var; if less than, v = 0;
        private double deadzone = .005;

        // Standard constructor when there's a starting point

        public VectorPacket(Point pos, Point lastPos, double time, double lastTime) {
            this.pos = pos; this.lastPos = lastPos; this.time = time; this.lastTime = lastTime;
        }

        // Zeroed constructor

        public VectorPacket() {
            pos = new Point(); lastPos = new Point(); time = 0; lastTime = 0;
        }

        public double getDistance() {
            return pos.distanceTo(lastPos);
        }

        public double getVelocity() {
            double velocity = Math.sqrt(Math.pow(getXVel(), 2) + Math.pow(getYVel(), 2));
            return velocity > deadzone ? velocity : 0;
        }

        public double getAngle(AngleUnit angleUnit) {
            return lastPos.angleTo(pos, angleUnit);
        }

        public double getXVel() {
            return (pos.getX() - lastPos.getX()) / (time - lastTime);
        }

        public double getYVel() {
            return (pos.getY() - lastPos.getY()) / (time - lastTime);
        }

        public double getTime() {
            return time;
        }

        public void setTime(double time) {
            this.time = time;
        }

    }
}