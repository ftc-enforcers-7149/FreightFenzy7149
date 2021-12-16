package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Bounds.Bounds;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Bounds.RectBounds;

public class Button extends TouchObject<Boolean> {

    private final Bounds bounds;
    private final boolean requireClick;

    public Button(Touchpad touchpad, boolean defaultValue, double leftX, double rightX, double bottomY, double topY, boolean requireClick) {
        super(touchpad, defaultValue);
        bounds = new RectBounds(leftX, rightX, bottomY, topY);
        this.requireClick = requireClick;
    }

    public Button(Touchpad touchpad, boolean defaultValue, Bounds bounds, boolean requireClick) {
        super(touchpad, defaultValue);
        this.bounds = bounds;
        this.requireClick = requireClick;
    }

    @Override
    public void updateInput() {
        switch (touchpad.getNumFingers()) {
            case 1:
                if (bounds.contains(touchpad.getFingerOne())
                        && (touchpad.isButtonPressed() || !requireClick)) {
                    value = true;
                    return;
                }
                break;
            case 2:
                if ((bounds.contains(touchpad.getFingerOne()) || bounds.contains(touchpad.getFingerTwo()))
                        && (touchpad.isButtonPressed() || !requireClick)) {
                    value = true;
                    return;
                }
                break;
            default:
                value = false;
        }
    }

}