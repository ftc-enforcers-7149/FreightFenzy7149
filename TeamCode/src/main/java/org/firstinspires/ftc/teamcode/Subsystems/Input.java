package org.firstinspires.ftc.teamcode.Subsystems;

public interface Input extends Subsytem {
    void updateInput();

    default void startInput() {}
    default void stopInput() {}
}
