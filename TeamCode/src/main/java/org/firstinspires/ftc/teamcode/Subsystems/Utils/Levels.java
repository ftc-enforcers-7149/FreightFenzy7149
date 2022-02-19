package org.firstinspires.ftc.teamcode.Subsystems.Utils;

import static org.firstinspires.ftc.teamcode.GlobalData.CAP_OFFSET;
import static org.firstinspires.ftc.teamcode.GlobalData.LEVEL_OFFSET;

public enum Levels {
    GROUND(0),
    LOW(3 + LEVEL_OFFSET),
    MIDDLE(8.5 + LEVEL_OFFSET),
    HIGH(14.75 + LEVEL_OFFSET - 1),
    CAP(20.25 + CAP_OFFSET),
    MAX(22);

    public final double height;

    Levels(double height) {
        this.height = height;
    }

    public Levels goUp() {
        if (this == MAX) return MAX;
        return values()[this.ordinal() + 1];
    }

    public Levels goDown() {
        if (this == GROUND) return GROUND;
        return values()[this.ordinal() - 1];
    }
}
