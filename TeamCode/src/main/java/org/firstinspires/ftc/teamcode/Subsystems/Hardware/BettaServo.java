package org.firstinspires.ftc.teamcode.Subsystems.Hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

public class BettaServo {

    private double startPosition;
    private long movingStartTime;

    public ServoImplEx servo;

    private long FULL_RANGE_TIME = 0;

    public BettaServo(ServoControllerEx controller, int portNumber, @NonNull ServoConfigurationType servoType) {
        servo = new ServoImplEx(controller, portNumber, servoType);
    }

    public BettaServo(ServoControllerEx controller, int portNumber, Servo.Direction direction, @NonNull ServoConfigurationType servoType) {
        servo = new ServoImplEx(controller, portNumber, direction, servoType);
    }

    public BettaServo(HardwareMap h, String servoName) {

        servo = h.get(ServoImplEx.class, servoName);

    }

    public void setFullRangeTime(long fullRangeTime) {
        this.FULL_RANGE_TIME = fullRangeTime;
    }

    public synchronized void setPosition(double position) {
        servo.setPosition(position);

        startPosition = position;
        movingStartTime = System.currentTimeMillis();
    }

    /**
     * Uses a time estimate to approximate the current position of the servo, even when moving
     * @return the approximate position of the servo
     */

    public synchronized double getPosition() {
        if (FULL_RANGE_TIME == 0 ||
                (System.currentTimeMillis() - movingStartTime) >
                        (servo.getPosition() - startPosition) * FULL_RANGE_TIME)
            return servo.getPosition();

        return (servo.getPosition() - startPosition) *
                ((double)(System.currentTimeMillis() - movingStartTime) / FULL_RANGE_TIME) +
                startPosition;
    }
}