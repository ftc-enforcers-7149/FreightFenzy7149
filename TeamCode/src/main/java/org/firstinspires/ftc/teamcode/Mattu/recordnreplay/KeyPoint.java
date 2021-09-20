package org.firstinspires.ftc.teamcode.Mattu.recordnreplay;

/**
 * The data class used for Record 'N Replay. It stores various information, like the robot's
 * position when the point was recorded, any specific actions it was doing, and whether the robot
 * had moved or turned significantly prior to arriving at this point.
 */
public class KeyPoint implements java.io.Serializable {

    //The robot's position
    public final double x, y, heading;

    //Any significant action
    public final Actions action;

    //If the robot moved or turned prior to arriving at this point
    //This is necessary in cases where multiple actions are done consecutively with no movement
    public final boolean moved, turned;

    /**
     * The default constructor, assumes robot is at (0, 0, 0 rad) and is sitting still
     */
    public KeyPoint() {
        x = 0; y = 0; heading = 0;
        action = Actions.DO_NOTHING;
        moved = false; turned = false;
    }

    /**
     * @param x The robot's x coordinate (f-b)
     * @param y The robot's y coordinate (l-r)
     * @param heading The robot's heading in radians (0 is forward)
     * @param action The action the robot is doing
     * @param moved If the robot moved to get to this point
     * @param turned If the robot turned to get to this point
     */
    public KeyPoint(double x, double y, double heading, Actions action,
                    boolean moved, boolean turned) {
        this.x = x; this.y = y; this.heading = heading;
        this.action = action;
        this.moved = moved;
        this.turned = turned;
    }


    @Override
    public String toString() {
        return "KeyPoint(" +
                x + ", " +
                y + ", " +
                heading + ", " +
                action + ", " +
                moved + ", " +
                turned + ")";
    }
}
