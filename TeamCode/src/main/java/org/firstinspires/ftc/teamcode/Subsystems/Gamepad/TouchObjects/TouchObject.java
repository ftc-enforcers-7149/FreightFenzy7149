package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public abstract class TouchObject<T> implements Input {

    protected Touchpad touchpad;
    protected T value, defaultValue;

    public TouchObject(Touchpad touchpad, T defaultValue) {
        this.touchpad = touchpad;
        this.defaultValue = defaultValue;
        value = defaultValue;
    }

    @Override
    public abstract void updateInput();
    public final T get() {
        return value;
    }
}
