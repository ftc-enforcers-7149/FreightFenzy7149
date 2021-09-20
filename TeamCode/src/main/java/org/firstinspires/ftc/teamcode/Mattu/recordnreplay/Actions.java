package org.firstinspires.ftc.teamcode.Mattu.recordnreplay;

/**
 * Actions are important because they are one of the simplest ways to serialize data about what
 * the robot is doing.
 * Each action has some important information with it, used in the Replay step. This is about what
 * kind of accuracy is needed in the robot's position before performing the action. For instance,
 * it doesn't exactly matter where a shooter stops spinning after it's shot, but it matters a lot
 * that it shoots at exactly the right position AND angle.
 */
public enum Actions implements java.io.Serializable {
    DO_NOTHING(false, false),
    DELAY(false, false),
    START_INTAKE(true, false),
    STOP_INTAKE(false, false);

    public boolean move, rotate;

    Actions(boolean move, boolean rotate) {
        this.move = move; this.rotate = rotate;
    }
}
