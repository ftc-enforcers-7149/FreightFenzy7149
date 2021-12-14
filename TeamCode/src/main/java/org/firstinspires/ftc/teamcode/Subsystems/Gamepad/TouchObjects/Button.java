package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;

public class Button extends TouchObject<Boolean> {

    private final double leftX, rightX, bottomY, topY;
    private final boolean click;

    public Button(Touchpad touchpad, Boolean defaultValue, boolean click, double leftX, double rightX, double bottomY, double topY) {
        super(touchpad, defaultValue);
        this.click = click;
        this.leftX = leftX;
        this.rightX = rightX;
        this.bottomY = bottomY;
        this.topY = topY;
    }

    @Override
    public void updateInput() {
        if(click) {
            switch (touchpad.getNumFingers()) {
                case 1:
                    if ((touchpad.getFingerOneX() <= rightX
                            && touchpad.getFingerOneX() >= leftX)
                            && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY)
                            && touchpad.isTouchButton()) {
                        value = true;
                        return;
                    }
                    break;
                case 2:
                    if (((touchpad.getFingerOneX() <= rightX && touchpad.getFingerOneX() >= leftX)
                            && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY)
                            && touchpad.isTouchButton())
                            ||
                            ((touchpad.getFingerTwoX() <= rightX && touchpad.getFingerTwoX() >= leftX)
                                    && (touchpad.getFingerTwoY() <= topY && touchpad.getFingerTwoY() >= bottomY)
                            && touchpad.isTouchButton())) {
                        value = true;
                        return;
                    }
                    break;
                default:
                    value = false;
            }
        }
        else {
            switch (touchpad.getNumFingers()) {
                case 1:
                    if ((touchpad.getFingerOneX() <= rightX && touchpad.getFingerOneX() >= leftX)
                            && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY)) {
                        value = true;
                        return;
                    }
                    break;
                case 2:
                    if (((touchpad.getFingerOneX() <= rightX && touchpad.getFingerOneX() >= leftX)
                            && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY))
                            ||
                            ((touchpad.getFingerTwoX() <= rightX && touchpad.getFingerTwoX() >= leftX)
                                    && (touchpad.getFingerTwoY() <= topY && touchpad.getFingerTwoY() >= bottomY))) {
                        value = true;
                        return;
                    }
                    break;
                default:
                    value = false;
            }
        }

        value = false;
    }
}