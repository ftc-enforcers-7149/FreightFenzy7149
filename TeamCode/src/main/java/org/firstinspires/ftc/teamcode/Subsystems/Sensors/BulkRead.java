package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

import java.util.HashMap;

public class BulkRead implements Input {

    //Declares LynxModule
    private LynxModule lynxModule;

    //Declares HardwareMap
    private HardwareMap hardwareMap;

    //Value to help determine the true value of encoders (used to help with encoder overload)
    private final static int CPS_STEP = 0x10000;

    //Clock to help manually determine velocity; used to crosscheck values with "corrected" velocity
    private NanoClock clock;

    //BulkData
    private LynxModule.BulkData bData;

    //HashMap remembers motors for calculating velocity
    private HashMap<DcMotorEx, Double[]> velocityMotors;

    /**
     * Constructor for the BulkRead class
     * @param hardwareMap OpMode's hardwareMap
     * @param lynxModule LynxModule; the Control Hub interface
     */
    public BulkRead(HardwareMap hardwareMap, LynxModule lynxModule) {

        //Instantiates control hub
        this.lynxModule = lynxModule;

        //Instantiates map
        this.hardwareMap = hardwareMap;

        //Instantiates clock
        clock = NanoClock.system();

        velocityMotors = new HashMap<DcMotorEx, Double[]>();

        updateInput();
    }

    /**
     * Constructor for the BulkRead class
     * @param hardwareMap hardware map used to instantiate the BulkRead class
     * @param moduleName the name of the lynx module to map
     */
    public BulkRead(HardwareMap hardwareMap, String moduleName) {

        //Instantiates hub
        this.lynxModule = hardwareMap.get(LynxModule.class, moduleName);

        //Instantiates map
        this.hardwareMap = hardwareMap;

        //Instantiates clock
        clock = NanoClock.system();

        velocityMotors = new HashMap<DcMotorEx, Double[]>();

        updateInput();
    }

    /**
     * Saves bulk read data from the hub. Please call this every loop.
     */
    @Override
    public void updateInput() {
        bData = lynxModule.getBulkData();
    }

    /**
     * Returns the position of a motor in encoder ticks
     * @param motor the motor to find the position of
     * @return returns the value of the motor's position in encoder ticks
     */
    public synchronized int getMotorPos(DcMotorEx motor) {

        try
        {
            //Gets the motor port number
            int portNum = motor.getPortNumber();

            //Checks if motors are reversed; if so, stores the encoder value
            int mPos = (motor.getDirection() == DcMotorSimple.Direction.REVERSE) ? -bData.getMotorCurrentPosition(portNum) : bData.getMotorCurrentPosition(portNum);

            //Returns the position
            return mPos;
        }
        //If we have an error, returns a null array
        catch (Exception e)
        {
            return 0;
        }

    }

    /**
     * Returns the position of a motor in encoder ticks
     * @param mName the name of the motor to read
     * @return returns the value of the motor's position in encoder ticks
     */
    public synchronized int getMotorPos(String mName) {

        //Sets the motor to read
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, mName);

        try
        {
            //Gets the motor port number
            int portNum = motor.getPortNumber();

            //Checks if motors are reversed; if so, stores the encoder value
            int mPos = (motor.getDirection() == DcMotorSimple.Direction.REVERSE) ? -bData.getMotorCurrentPosition(portNum) : bData.getMotorCurrentPosition(portNum);

            //Returns the position
            return mPos;
        }
        //If we have an error, returns a null array
        catch (Exception e)
        {
            return 0;
        }

    }

    /**
     * Returns the velocity of a motor
     * @param motor the motor to read
     * @return returns a double value of the encoder's velocity
     */
    public synchronized double getMotorVel(DcMotorEx motor) {

        try
        {
            double lastPosition, lastTime;

            //If the motor has already been used, get its last values
            //If it's new, add it to the HashMap and default to 0
            if (velocityMotors.containsKey(motor)) {
                lastPosition = velocityMotors.get(motor)[0];
                lastTime = velocityMotors.get(motor)[1];
            }
            else {
                velocityMotors.put(motor, new Double[]{0.0, 0.0, 0.0});
                lastPosition = 0;
                lastTime = 0;
            }

            //Cross-references the position with prior position to create velocity estimate
            double currentPosition = getMotorPos(motor);
            double lastUpdateTime = lastTime;
            double velocityEstimate = velocityMotors.get(motor)[2];

            double currentTime = clock.seconds();
            double dt = currentTime - lastUpdateTime;

            if (currentPosition != lastPosition) {
                velocityEstimate = (currentPosition - lastPosition) / dt;

                //Store new lastPos and lastTime
                velocityMotors.replace(motor, new Double[]{currentPosition, currentTime, velocityEstimate});
            }

            //Gets the motor port number
            int portNum = motor.getPortNumber();

            //Stores the motor velocity
            double mVel = (motor.getDirection() == DcMotorSimple.Direction.REVERSE) ? -bData.getMotorVelocity(portNum) : bData.getMotorVelocity(portNum);

            //Returns the position
            return inverseOverflow(mVel, velocityEstimate);
        }
        //If we have an error, returns null
        catch (Exception e)
        {
            return 0;
        }

    }

    /**
     * Returns the velocity of a motor
     * @param mName the name of the motor to read
     * @return returns a double value of the encoder's velocity
     */
    public synchronized double getMotorVel(String mName) {

        //Sets the motor to read
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, mName);

        try
        {
            double lastPosition, lastTime;

            //If the motor has already been used, get its last values
            //If it's new, add it to the HashMap and default to 0
            if (velocityMotors.containsKey(motor)) {
                lastPosition = velocityMotors.get(motor)[0];
                lastTime = velocityMotors.get(motor)[1];
            }
            else {
                velocityMotors.put(motor, new Double[]{0.0, 0.0, 0.0});
                lastPosition = 0;
                lastTime = 0;
            }

            //Cross-references the position with prior position to create velocity estimate
            double currentPosition = getMotorPos(motor);
            double lastUpdateTime = lastTime;
            double velocityEstimate = velocityMotors.get(motor)[2];

            double currentTime = clock.seconds();
            double dt = currentTime - lastUpdateTime;

            if (currentPosition != lastPosition) {
                velocityEstimate = (currentPosition - lastPosition) / dt;

                //Store new lastPos and lastTime
                velocityMotors.replace(motor, new Double[]{currentPosition, currentTime, velocityEstimate});
            }

            //Gets the motor port number
            int portNum = motor.getPortNumber();

            //Stores the motor velocity
            double mVel = (motor.getDirection() == DcMotorSimple.Direction.REVERSE) ? -bData.getMotorVelocity(portNum) : bData.getMotorVelocity(portNum);

            //Returns the position
            return inverseOverflow(mVel, velocityEstimate);
        }
        //If we have an error, returns null
        catch (Exception e)
        {
            return 0;
        }

    }

    /**
     * Returns the corrected velocity of an encoder
     * @param input the raw velocity
     * @param estimate the estimated velocity
     * @return returns the corrected velocity
     */
    private static double inverseOverflow(double input, double estimate) {
        double real = input;
        while (Math.abs(estimate - real) > CPS_STEP / 2.0) {
            real += Math.signum(estimate - real) * CPS_STEP;
        }
        return real;
    }

    /**
     * Gets the LynxModule currently in use
     * @return returns a LynxModule
     */
    public LynxModule getLynxModule() {
        return lynxModule;
    }

    /**
     * Returns the NanoClock currently in use
     * @return the clock as a NanoClock
     */
    public NanoClock getClock() {
        return clock;
    }

    /**
     * Gets the bulk data of the class
     * @return the data as a BulkData class
     */
    public LynxModule.BulkData getBulkData() {
        return bData;
    }

    @Override
    public void stop() {

    }
}
