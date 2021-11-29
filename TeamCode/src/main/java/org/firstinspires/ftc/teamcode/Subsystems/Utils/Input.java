package org.firstinspires.ftc.teamcode.Subsystems.Utils;

public interface Input extends Subsytem {
    void updateInput();

    default void startInput() {}
    default void stopInput() {}
}
