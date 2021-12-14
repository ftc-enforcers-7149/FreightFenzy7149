package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public abstract class TouchObject<T> implements Input {

    protected Touchpad touchpad;
    protected T value;

    public enum Type {
        BOOLEAN,
        LEFT_SWIPE,
        RIGHT_SWIPE,
        UP_SWIPE,
        DOWN_SWIPE,
        X_AXIS,
        Y_AXIS
    }

    public TouchObject(Touchpad touchpad, T defaultValue) {
        this.touchpad = touchpad;
        value = defaultValue;
    }

    @Override
    public abstract void updateInput();
    public final T get() {
        return value;
    }
}
