package org.firstinspires.ftc.teamcode.Testing.Prototyping;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.Encoder;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.Gyroscope;

@TeleOp(name="Adam Four Bar")
public class AdamFourBar extends OpMode {

    protected Gyroscope gyro;

    private final double TICKS_PER_REV = 8092;
    private final double CHAIN_GEARING = 1d/1d;
    private final double ANGLE_PER_TICK = TICKS_PER_REV * 2 * Math.PI / CHAIN_GEARING;
    private final double TICKS_PER_ANGLE = CHAIN_GEARING / (TICKS_PER_REV * 2 * Math.PI);

    private final double turretTicksPerRot = 1425.1;

    private DcMotorEx rotate, lift;
    private DcMotor fLeft, fRight, bLeft, bRight;
    private CRServo intake;
    private Encoder intakeEnc;

    private double ticksPer120 = TICKS_PER_REV / 3;

    double rotateInput, liftInput;
    boolean inIntake, outIntake;

    boolean holdUp;

    public void init() {
        gyro = new Gyroscope(hardwareMap);

        rotate = hardwareMap.get(DcMotorEx.class, "turret");
        lift = hardwareMap.get(DcMotorEx.class, "elevator");

        fLeft = hardwareMap.get(DcMotorEx.class, "fLeft");
        fRight = hardwareMap.get(DcMotorEx.class, "fRight");
        bLeft = hardwareMap.get(DcMotorEx.class, "bLeft");
        bRight = hardwareMap.get(DcMotorEx.class, "bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.crservo.get("intake");

        rotate.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeEnc = new Encoder(lift);
        intakeEnc.setDirection(Encoder.Direction.REVERSE);

        holdUp = false;
    }

    @Override
    public void start() {
        gyro.startInput();
    }

    public void loop() {
        updateInput();
        gyro.updateInput();
        driveTankArcade();
        //driveTankHeadless();

        if (gamepad2.a) {
            rotate.setPower(rotateInput);
        }
        else if (Math.abs(gamepad1.right_stick_x) >= 0.3 && rotateInput == 0) {
            rotateInput = -0.7 * gamepad1.right_stick_x;
            rotate.setPower(rotateInput);
        }
        else if (Math.abs(gamepad1.right_stick_x) >= 0.3 && rotateInput != 0) {
            rotateInput = (rotateInput - 0.7 * gamepad1.right_stick_x) / 2;
            rotate.setPower(rotateInput);
        }
        else {
            rotate.setPower(rotateInput);
        }

        if (liftInput != 0)
            lift.setPower(liftInput);
        else if (holdUp)
            lift.setPower(0.08);
        else
            lift.setPower(0);

        if(inIntake) intake.setPower(1);
        else if(outIntake) intake.setPower(-1);
        else {
            double ticksFromStop = intakeEnc.getCurrentPosition() % ticksPer120;

            if (ticksFromStop > TICKS_PER_REV / 30 && ticksFromStop < ticksPer120 - (TICKS_PER_REV / 180))
                intake.setPower(0.1);
            else
                intake.setPower(0);
        }

        telemetry.addData("Intake From Stop: ", intakeEnc.getCurrentPosition() % ticksPer120);
        telemetry.addData("Move? ", (intakeEnc.getCurrentPosition() % ticksPer120 > TICKS_PER_REV / 30 && intakeEnc.getCurrentPosition() % ticksPer120 < ticksPer120 - (TICKS_PER_REV / 30)));
    }

    @Override
    public void stop() {
        setMotorPowers(0, 0, 0, 0);
        rotate.setPower(0);
        lift.setPower(0);
        gyro.stopInput();
    }

    public void updateInput() {
        if (gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1)
            rotateInput = (gamepad2.right_trigger - gamepad2.left_trigger) * 0.75;
        else
            rotateInput = 0;

        if (-gamepad2.left_stick_y > 0.1) {
            liftInput = -gamepad2.left_stick_y * 0.7;
            holdUp = true;
        }
        else if (-gamepad2.left_stick_y < -0.1) {
            liftInput = -gamepad2.left_stick_y * 0.7;
            holdUp = false;
        }
        else
            liftInput = 0;

        inIntake = gamepad2.right_bumper;
        outIntake = gamepad2.left_bumper;
    }

    protected void driveTankArcade() {
        double drivePower = gamepad1.left_stick_y;
        double turnPower = -gamepad1.right_stick_x;

        double vL = drivePower + turnPower;
        double vR = drivePower - turnPower;

        double max = Math.max(Math.abs(vL), Math.abs(vR));

        if (max > 1) {
            vL /= max;
            vR /= max;
        }

        setMotorPowers(vL, vR, vL, vR);
    }

    protected void driveTankHeadless() {
        double driveX = gamepad1.left_stick_x;
        double driveY = -gamepad1.left_stick_y;
        double turnInPlace = gamepad1.right_stick_x;

        double heading = Math.toRadians(gyro.getYaw());
        double destHeading = Math.atan2(driveY, driveX);
        double delta = deltaHeading(heading, destHeading);

        double leftPower = 0, rightPower = 0;

        leftPower += Math.signum(delta) * Math.hypot(driveX, driveY);
        rightPower -= Math.signum(delta) * Math.hypot(driveX, driveY);

        leftPower += Math.hypot(driveX, driveY);
        rightPower += Math.hypot(driveX, driveY);

        leftPower += turnInPlace;
        rightPower -= turnInPlace;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (max > 1) {
            leftPower /= max;
            rightPower /= max;
        }

        setMotorPowers(leftPower, rightPower, leftPower, rightPower);
    }

    /**
     * @param robotH Robot heading in radians
     * @param destH Destination heading in radians
     * @return Shortest heading difference in radians
     */
    protected double deltaHeading(double robotH, double destH) {
        double diff = destH - robotH;

        if (diff < -Math.PI) diff += Math.PI * 2;
        if (diff > Math.PI) diff -= Math.PI * 2;

        return diff;
    }

    protected void setMotorPowers(double vFL, double vFR, double vBL, double vBR) {
        fLeft.setPower(vFL);
        fRight.setPower(vFR);
        bLeft.setPower(vBL);
        bRight.setPower(vBR);
    }
}