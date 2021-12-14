package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;


import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;

public class Zone extends TouchObject<Boolean> {

    private double leftX, rightX, bottomY, topY;
    private boolean click;

    public Zone(String name, Touchpad touchpad, boolean click, double leftX, double rightX, double bottomY, double topY) {

        super(name, touchpad);
        this.click = click;
        this.leftX = leftX;
        this.rightX = rightX;
        this.bottomY = bottomY;
        this.topY = topY;

    }


    public Boolean get() {

        if(click) {
            switch (touchpad.getNumFingers()) {
                case 1:
                    if ((touchpad.getFingerOneX() <= rightX
                            && touchpad.getFingerOneX() >= leftX)
                            && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY)
                            && touchpad.isTouchButton())
                        return true;
                    break;

                case 2:
                    if (((touchpad.getFingerOneX() <= rightX && touchpad.getFingerOneX() >= leftX)
                            && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY)
                            && touchpad.isTouchButton())
                            ||
                            ((touchpad.getFingerTwoX() <= rightX && touchpad.getFingerTwoX() >= leftX)
                                    && (touchpad.getFingerTwoY() <= topY && touchpad.getFingerTwoY() >= bottomY)
                            && touchpad.isTouchButton()))
                        return true;
                    break;
                default:
                    return false;
            }
        }
        else {
            switch (touchpad.getNumFingers()) {
                case 1:
                    if ((touchpad.getFingerOneX() <= rightX && touchpad.getFingerOneX() >= leftX)
                            && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY))
                        return true;
                    break;

                case 2:
                    if (((touchpad.getFingerOneX() <= rightX && touchpad.getFingerOneX() >= leftX)
                            && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY))
                            ||
                            ((touchpad.getFingerTwoX() <= rightX && touchpad.getFingerTwoX() >= leftX)
                                    && (touchpad.getFingerTwoY() <= topY && touchpad.getFingerTwoY() >= bottomY)))
                        return true;
                    break;
                default:
                    return false;
            }
        }

        return false;

    }

}