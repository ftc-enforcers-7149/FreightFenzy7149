package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Odometry.Util.Encoder;

public class  Flywheel {

    //Shooter motor and attached REV Encoder
    public DcMotorEx flywheel;
    public Encoder encoder;

    //Converts counts per second to rotations per minute
    private final double toRPM = 60/8192.0;

    //Velocity to set to flywheel
    private double setVelocity;

    //Whether or not the motor should be powered
    private boolean running;

    //Power to set if not running PIDF
    private double setPower, lastSetPower;

    public Flywheel(HardwareMap hardwareMap, String motorName) {
        //Initialize flywheel motor
        flywheel = hardwareMap.get(DcMotorEx.class, motorName);
        encoder = new Encoder(flywheel);

        //Initialize class variables
        setVelocity = 0;
        running = false;
        setPower = 0;
        lastSetPower = 0;
    }

    public void setPower(double power) {
        setPower = power;

        setVelocity = 0;
        running = false;
    }

    public void update() {

        if (!running) {
            if (setPower != lastSetPower) {
                flywheel.setPower(setPower);
            }
        }

        lastSetPower = setPower;
    }
}
