package org.firstinspires.ftc.teamcode.Subsystems.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Point;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.VectorPacket;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public class Touchpad implements Input {

    //TODO: fix angle handling

    // Gamepad for handling input
    private final Gamepad gamepad;

    // touchButton: touchpad "press"
    private boolean touchButton, lastTouchButton;

    // number of fingers on the touchpad
    private int numFingers;

    // positional variables
    private final Point fingerOne, lastFingerOne, fingerTwo, lastFingerTwo;

    private long lastTime = 0;

    // how often the touchpad "updates"
    private final double pollingTime = 200;

    // vector packets for storing velocity data
    private final VectorPacket v1, v2;

    // standard unit multiplier
    private final double standardMult = 100;

    // finger deadzone
    private final double fingerDeadzone = 2;

    // Constructor. pass in the tele-op gamepad
    public Touchpad(Gamepad gamepad) {
        this.gamepad = gamepad;

        fingerOne = new Point(); lastFingerOne = new Point();
        fingerTwo = new Point(); lastFingerTwo = new Point();

        v1 = new VectorPacket();
        v2 = new VectorPacket();
    }

    // Update method
    @Override
    public void updateInput() {

        // Updates time

        // time variables
        long time = System.currentTimeMillis();

        // Checks if touchpad press
        touchButton = gamepad.touchpad;

        // Checks how many fingers are on the touchpad

        numFingers = 0;
        if (gamepad.touchpad_finger_1) {
            numFingers++;

            double f1X = Math.round(10 * standardMult * gamepad.touchpad_finger_1_x) / 10.0;
            double f1Y = Math.round(10 * standardMult * gamepad.touchpad_finger_1_y) / 10.0;

            fingerOne.setPoint(f1X, f1Y);
        }
        if (gamepad.touchpad_finger_2) {
            numFingers++;

            double f2X = Math.round(10 * standardMult * gamepad.touchpad_finger_2_x) / 10.0;
            double f2Y = Math.round(10 * standardMult * gamepad.touchpad_finger_2_y) / 10.0;

            fingerTwo.setPoint(f2X, f2Y);
        }
        else
            numFingers = 0;

        // Checks the position of fingers and updates

        // If we've hit the poll time
        if(time - lastTime >= pollingTime) {
            // Updates the vectors of the fingers if applicable

            switch(numFingers) {
                case 1:
                    v1.setVectorPacket(new VectorPacket(fingerOne, lastFingerOne, time / 1000d, lastTime / 1000d));
                    v2.setVectorPacket(new VectorPacket());
                    break;
                case 2:
                    v1.setVectorPacket(new VectorPacket(fingerOne, lastFingerOne, time / 1000d, lastTime / 1000d));
                    v2.setVectorPacket(new VectorPacket(fingerTwo, lastFingerTwo, time / 1000d, lastTime / 1000d));
                    break;

                default:
                    v1.setVectorPacket(new VectorPacket());
                    v2.setVectorPacket(new VectorPacket());
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
    public boolean getFingerOn() { return numFingers >= 1; }

    // Returns the finger vector packets
    public VectorPacket getV1() {
        return v1;
    }
    public VectorPacket getV2() {
        return v2;
    }
}