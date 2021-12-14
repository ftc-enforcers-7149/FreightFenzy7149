package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Bounds;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Point;

public class Button extends TouchObject<Boolean> {

    private final Bounds bounds;
    private final boolean click;

    public Button(Touchpad touchpad, Boolean defaultValue, boolean click, double leftX, double rightX, double bottomY, double topY) {
        super(touchpad, defaultValue);
        this.click = click;
        bounds = new Bounds(leftX, rightX, bottomY, topY);
    }

    @Override
    public void updateInput() {
        if(click) {
            switch (touchpad.getNumFingers()) {
                case 1:
                    if (bounds.contains(new Point(touchpad.getFingerOneX(), touchpad.getFingerOneY()))
                            && touchpad.isTouchButton()) {
                        value = true;
                        return;
                    }
                    break;
                case 2:
                    if ((bounds.contains(new Point(touchpad.getFingerOneX(), touchpad.getFingerOneY()))
                            && touchpad.isTouchButton())
                            ||
                            (bounds.contains(new Point(touchpad.getFingerTwoX(), touchpad.getFingerTwoY()))
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
                    if (bounds.contains(new Point(touchpad.getFingerOneX(), touchpad.getFingerOneY()))) {
                        value = true;
                        return;
                    }
                    break;
                case 2:
                    if (bounds.contains(new Point(touchpad.getFingerOneX(), touchpad.getFingerOneY()))
                            || bounds.contains(new Point(touchpad.getFingerTwoX(), touchpad.getFingerTwoY()))) {
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