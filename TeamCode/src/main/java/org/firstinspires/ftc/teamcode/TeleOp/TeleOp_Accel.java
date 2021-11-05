package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.NovaLmao.CurveHandling;
import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

import static org.firstinspires.ftc.teamcode.Subsystems.FixedRoadrunner.createPose2d;

public abstract class TeleOp_Accel extends OpMode {

    //Drive
    protected MecanumDrive drive;
    protected DcMotorEx fLeft, fRight, bLeft, bRight;
    protected Gyroscope gyro;
    private boolean initializedMotors = false, initializedDrive = false, initializedGyro = false;

    //Control objects
    protected BulkRead bReadCH, bReadEH;
    private boolean hasCH, hasEH;

    //State machine logic
    protected double leftX, leftY, rightX, rightY;
    protected double lastLeftX, lastLeftY, lastRightX, lastRightY;
    protected double time, lastTime, startTimeL, startTimeR, startTime;
    protected double velL, velR, lastVelL = 0, lastVelR = 0;
    boolean tankInit = false;

    //Headless
    protected double offset, lim;

    protected CurveHandling curve, curveL, curveR;

    //Configurable CurveHandle variables
    public static double revZone = .2, revTime = 150, maxTime = 500;

    public enum AccelState {

        TURNING,
        DRIVING,
        POST_TURN_RAMP,
        RESTING

    }

    protected AccelState aStateL = AccelState.RESTING, aStateR = AccelState.RESTING, aState = AccelState.RESTING, lastAState, lastAStateL, lastAStateR;

    //Initialization
    protected void initializeDrive() {
        if (!initializedDrive) {
            fLeft = hardwareMap.get(DcMotorEx.class, "fLeft");
            fRight = hardwareMap.get(DcMotorEx.class, "fRight");
            bLeft = hardwareMap.get(DcMotorEx.class, "bLeft");
            bRight = hardwareMap.get(DcMotorEx.class, "bRight");

            fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            fRight.setDirection(DcMotorSimple.Direction.FORWARD);
            bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            bRight.setDirection(DcMotorSimple.Direction.FORWARD);

            fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            fLeft = drive.fLeft;
            fRight = drive.fRight;
            bLeft = drive.bLeft;
            bRight = drive.bRight;
        }

        curve = new CurveHandling(System.currentTimeMillis(), revZone, revTime, maxTime);
        initializedMotors = true;
    }
    protected void initializeBulkRead() {
        try {
            bReadCH = new BulkRead(hardwareMap, "Control Hub");
            hasCH = true;
        } catch (Exception e) {
            hasCH = false;
        }
        try {
            bReadEH = new BulkRead(hardwareMap, "Expansion Hub");
            hasEH = true;
        } catch (Exception e) {
            hasEH = false;
        }
    }
    protected void initializeGyro() {
        if (!initializedDrive)
            gyro = new Gyroscope(hardwareMap);
        else
            gyro = drive.gyro;

        initializedGyro = true;
    }
    protected void initializeOdometry() throws Exception {
        if (!hasCH || !hasEH) throw new Exception("Missing \"Control Hub\". Check configuration file naming");
        if (initializedMotors && initializedGyro)
            drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH, fLeft, fRight, bLeft, bRight, gyro);
        else if (initializedMotors)
            drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH, fLeft, fRight, bLeft, bRight);
        else if (initializedGyro)
            drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH, gyro);
        else
            drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH);
        drive.setPoseEstimate(createPose2d(0, 0, 0));

        initializedDrive = true;
    }
    protected void initializeVars() {
        lastLeftX = 0; lastLeftY = 0; lastRightX = 0; lastRightY = 0;
        offset = 0; lim = 1;
    }

    //Loop updates
    protected void updateBulkRead() {
        if (hasCH) bReadCH.update();
        if (hasEH) bReadEH.update();
    }

    //Driving
    protected void driveArcade() {
        //Simple arcade math
        if (leftY != lastLeftY && leftX != lastLeftX && rightX != lastRightX) {
            double vFL = -leftY + leftX - rightX;
            double vFR = -leftY - leftX + rightX;
            double vBL = -leftY - leftX - rightX;
            double vBR = -leftY + leftX + rightX;

            //Getting the max value can assure that no motor will be set to a value above a certain point.
            double max = Math.max(Math.max(Math.abs(vFL), Math.abs(vFR)), Math.max(Math.abs(vBL), Math.abs(vBR)));

            //In this case, no motor can go above lim power by scaling them all down if such a thing might occur.
            if (max > lim) {
                vFL /= max / lim;
                vFR /= max / lim;
                vBL /= max / lim;
                vBR /= max / lim;
            }

            setMotorPowers(vFL, vFR, vBL, vBR);
        }
    }

    //TODO: rewrite
    protected void driveAccelHeadless(double angle, boolean reset, double accelTime, double turnFactor, boolean disregard) {

        // Resets the robot's angle
        if (reset) {
            offset = angle;
        }

        // A little bit of fun math to determine whether we've made a significant change in joystick position.
        // Have fun messing around with this to your liking. This is a complete guesstimate.

        if(Math.abs(leftX - lastLeftX) >= .15 || Math.abs(leftY - lastLeftY) >= .15) startTime = System.currentTimeMillis();

        // If our controllers are zeroed, set the motor powers to zero

        if ((leftY == 0 && lastLeftY != 0) &&
                (leftX == 0 && lastLeftX != 0) &&
                (rightX == 0 && lastRightX != 0)) {
            setMotorPowers(0, 0, 0, 0);
            return;
        }

        // Ratio for the wheels

        double r = Math.hypot(leftX, leftY) * Math.sqrt(2);

        //Robot angle

        double robotAngle = Math.atan2(leftY, leftX) - Math.toRadians(Gyroscope.cvtTrigAng(angle - offset)) - 3 * Math.PI / 4;

        // Sine and cosine factors. These ramp based on a time-based acceleration curve.

        double sinFactor = (time - startTime <= accelTime) && !disregard ? ((time - startTime) / accelTime) * Math.sin(robotAngle) : Math.sin(robotAngle);
        double cosFactor = (time - startTime <= accelTime) && !disregard ? ((time - startTime) / accelTime) * Math.cos(robotAngle) : Math.cos(robotAngle);

        // Sets the motor powers

        double vFL = r * sinFactor + rightX * turnFactor;
        double vFR = r * cosFactor - rightX * turnFactor;
        double vBL = r * cosFactor + rightX * turnFactor;
        double vBR = r * sinFactor - rightX * turnFactor;

        // Calculates the maximum moter powers

        double max = Math.max(
                Math.max(
                        Math.abs(vFL),
                        Math.abs(vFR)),
                Math.max(
                        Math.abs(vBL),
                        Math.abs(vBR))
        );

        // Normalizes the wheel powers

        if (max > lim) {
            vFL /= max / lim;
            vFR /= max / lim;
            vBL /= max / lim;
            vBR /= max / lim;
        }

        // Sets the motor powers

        setMotorPowers(vFL, vFR, vBL, vBR);
    }

    protected void driveAccelTank(boolean disregard) {

        if(!tankInit)  {
            curveL = curve; curveR = curve;
            tankInit = true;
        }

        //TODO: make this a bit fancier
        if(leftY != 0 || rightY != 0) {

            curveL.run(time, leftY);
            curveR.run(time, rightY);

            if(Math.abs(leftY - rightY) > .1) {

                velL = curveL.getProjVelocity() * 0.6;
                velR = curveR.getProjVelocity() * 0.6;

            }
            else {

                velL = curveL.getProjVelocity();
                velR = curveR.getProjVelocity();

            }

            setMotorPowers(velL, velR, velL, velR);

        }

        lastVelL = velL;
        lastVelR = velR;

    }

    /*protected void driveAccelTank(double accelTime, boolean disregard) {

        lastAStateL = aStateL;
        lastAStateR = aStateR;

        if(!tankInit)  {
            curveL = curve; curveR = curve;
            tankInit = true;
        }


        // Checks to see if we need to update anything.

        *//*if (leftY != lastLeftY || rightY != lastRightY || time - startTimeL <= accelTime || time - startTimeR <= accelTime
            || aStateL != AccelState.RESTING || aStateR != AccelState.RESTING) {*//*
        if(leftY != 0 || rightY != 0) {

            // Checks for significant change in left joystick position. This value is an estimate.

            if(Math.abs(leftY - lastLeftY) >= .15) {

                startTimeL = time;
                velL = 0;

            }

            // Checks for significant change in right joystick position. This value is an estimate.

            if(Math.abs(rightY - lastRightY) >= .15) {

                startTimeR = time;
                velR = 0;

            }

            boolean accelLeft = (time - startTimeL <= accelTime) && !disregard;
            boolean accelRight = (time - startTimeR <= accelTime) && !disregard;
            boolean turning = Math.abs(leftY - rightY) > .1;

            if(turning) {

                aStateL = AccelState.TURNING;
                aStateR = AccelState.TURNING;

                if(leftY == 0) aStateL = AccelState.RESTING;
                if(rightY == 0) aStateR = AccelState.RESTING;

            }
            else {

                if(accelLeft) {
                    aStateL = (lastAStateL == AccelState.TURNING
                            || lastAStateL == AccelState.POST_TURN_RAMP) ?
                            AccelState.POST_TURN_RAMP : AccelState.DRIVING;
                }
                else {
                    aStateL = (leftY == 0) ? AccelState.RESTING : AccelState.DRIVING;
                }

                if(accelRight) {
                    aStateR = (lastAStateR == AccelState.TURNING
                            || lastAStateR == AccelState.POST_TURN_RAMP) ?
                            AccelState.POST_TURN_RAMP : AccelState.DRIVING;
                }
                else {
                    aStateR = (rightY == 0) ? AccelState.RESTING : AccelState.DRIVING;
                }

            }

            boolean lCurve = curveL.run(time - startTimeL, leftY);

            switch(aStateL) {

                case TURNING:
                    velL = curveL.getProjVelocity() * 0.6;
                    break;

                case DRIVING:
                    velL = curveL.getProjVelocity();
                    break;

                case RESTING:
                    velL = 0;
                    break;
                default:
                    break;

            }

            boolean rCurve;
            if(aStateR != AccelState.RESTING) rCurve = curveR.run(time - startTimeR, rightY);

            switch(aStateR) {

                case TURNING:
                    velR = curveR.run(time - startTimeR, rightY) ? rightY * 0.6 : curve.getProjVelocity() * 0.6;
                    break;


                case ACCELERATING:
                    velR = rightY * (time - startTimeR) / accelTime;
                    break;

                case POST_ACCELERATING:
                    velR = rightY;
                    break;

                case RESTING:
                    velR = 0;
                    break;

                default:
                    break;

            }

            // If one wheel has already ramped up and we're not turning, we ramp the other motor up with it in proportion.

            if(aStateL == AccelState.POST_TURN_RAMP) {
                velL = aStateL != lastAStateL ? velR : leftY * (velR / rightY);
            }
            else if(aStateR == AccelState.POST_TURN_RAMP) {
                velR = aStateR != lastAStateR ? velL : rightY * (velL / leftY);
            }

            // Ensures we don't start at 0 power.

            //TODO: tune
            if(Math.abs(velL) < .15 && leftY != 0) velL = Math.copySign(0.15, velL);
            if(Math.abs(velR) < .15 && rightY != 0) velR = Math.copySign(0.15, velR);

            // Calculates the maximum arbitrary power value.

            double max = Math.max(Math.abs(velL), Math.abs(velR));

            // Standardizes the power on a scale of 1.

            if (max > lim) {
                velL /= max / lim;
                velR /= max / lim;
            }

            // Sets motor powers.

            setMotorPowers(velL, velR, velL, velR);

        }
        *//*else if (leftY == 0 || rightY == 0) {*//*
        else {

            aStateL = AccelState.RESTING;
            aStateR = AccelState.RESTING;

            if(aStateL != lastAStateL || aStateR != lastAStateR) setMotorPowers(0, 0, 0, 0);

//            if(aStateL != lastAStateL) {
//                lastAStateL = aStateL;
//                aStateL = AccelState.RESTING;
//            }
//            if(aStateR != lastAStateR) {
//                lastAStateR = aStateR;
//                aStateR = AccelState.RESTING;
//            }

        }

        lastVelL = velL;
        lastVelR = velR;
        *//*lastAStateL = aStateL;
        lastAStateR = aStateR;*//*
    }*/

    //State machine logic
    protected abstract void getInput();
    protected abstract void updateStateMachine();


    //Useful functions

    /**
     * Curves an input so that 0->0, -1->-1, and 1->1,
     * but in between values curve slowly up by input^exp
     * @param input The function input
     * @param exp The exponent for the curve
     * @return The result of curving the input
     */
    protected double curveInput(double input, double exp) {
        return Math.signum(input) * Math.abs(Math.pow(input, exp));
    }

    protected void setMotorPowers(double vFL, double vFR, double vBL, double vBR) {
        fLeft.setPower(vFL);
        fRight.setPower(vFR);
        bLeft.setPower(vBL);
        bRight.setPower(vBR);
    }

}
