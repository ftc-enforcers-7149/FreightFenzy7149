package org.firstinspires.ftc.teamcode.Subsystems.Utils;

public interface Output extends Subsytem {
    void updateOutput();

    default void startOutput() {}
    default void stopOutput() {}
}
