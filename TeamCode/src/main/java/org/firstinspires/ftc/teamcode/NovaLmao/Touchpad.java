package org.firstinspires.ftc.teamcode.NovaLmao;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Touchpad {

    private Gamepad gamepad;

    private boolean touchButton, lastTouchButton, fingerOn, lastFingerOn;
    private int numFingers, lastNumFingers;

    private double fingerOneX, lastFingerOneX, fingerOneY, lastFingerOneY,
            fingerTwoX, lastFingerTwoX, fingerTwoY, lastFingerTwoY;

    private double standardMult = 100;

    public Touchpad(Gamepad gamepad) {

        this.gamepad = gamepad;

    }

    public void update() {

        lastTouchButton = touchButton;
        touchButton = gamepad.touchpad;

        lastNumFingers = numFingers;

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

}
