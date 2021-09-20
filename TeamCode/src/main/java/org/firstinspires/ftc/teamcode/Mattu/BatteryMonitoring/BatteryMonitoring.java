package org.firstinspires.ftc.teamcode.Mattu.BatteryMonitoring;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Battery Monitoring")
@Disabled
public class BatteryMonitoring extends OpMode {

    private BatteryMonitor batteryMonitor;

    private DcMotorEx m1, m2, m3, m4;

    @Override
    public void init() {
        batteryMonitor = new BatteryMonitor(hardwareMap, "Expansion Hub");

        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m4 = hardwareMap.get(DcMotorEx.class, "m4");

        batteryMonitor.addMainHubMotor(m1);
        batteryMonitor.addMainHubMotor(m2);
        batteryMonitor.addMainHubMotor(m3);
        batteryMonitor.addMainHubMotor(m4);
    }

    @Override
    public void loop() {
        /*m1.setPower(gamepad1.left_stick_y);
        m2.setPower(gamepad1.right_stick_y);
        m3.setPower(gamepad1.left_trigger);
        m4.setPower(gamepad1.right_trigger);*/

        m1.setPower(1);
        m2.setPower(1);
        m3.setPower(1);
        m4.setPower(1);

        telemetry.addLine(batteryMonitor.toString());
    }

    @Override
    public void stop() {
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}
