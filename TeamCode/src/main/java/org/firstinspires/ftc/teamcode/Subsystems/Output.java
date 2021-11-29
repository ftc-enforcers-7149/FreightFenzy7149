package org.firstinspires.ftc.teamcode.Subsystems;

public interface Output extends Subsytem {
    void updateOutput();

    default void startOutput() {}
    default void stopOutput() {}
}
