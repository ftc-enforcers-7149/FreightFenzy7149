package org.firstinspires.ftc.teamcode.Subsystems.Utils;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

public class BettaServo extends ServoImplEx {

    private double startPosition;
    private long movingStartTime;

    private long FULL_RANGE_TIME = 0;

    public BettaServo(ServoControllerEx controller, int portNumber, @NonNull ServoConfigurationType servoType) {
        super(controller, portNumber, servoType);
    }

    public BettaServo(ServoControllerEx controller, int portNumber, Direction direction, @NonNull ServoConfigurationType servoType) {
        super(controller, portNumber, direction, servoType);
    }

    public void setFullRangeTime(long fullRangeTime) {
        this.FULL_RANGE_TIME = fullRangeTime;
    }

    @Override
    public synchronized void setPosition(double position) {
        super.setPosition(position);

        startPosition = position;
        movingStartTime = System.currentTimeMillis();
    }

    /**
     * Uses a time estimate to approximate the current position of the servo, even when moving
     * @return the approximate position of the servo
     */
    @Override
    public synchronized double getPosition() {
        if (FULL_RANGE_TIME == 0 ||
                (System.currentTimeMillis() - movingStartTime) >
                        (super.getPosition() - startPosition) * FULL_RANGE_TIME)
            return super.getPosition();

        return (super.getPosition() - startPosition) *
                ((double)(System.currentTimeMillis() - movingStartTime) / FULL_RANGE_TIME) +
                startPosition;
    }
}