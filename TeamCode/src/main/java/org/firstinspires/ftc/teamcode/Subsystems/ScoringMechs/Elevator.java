package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public class Elevator implements Input {

    //Elevator motor
    public DcMotorEx elevator;

    //Bulk Read
    public BulkRead bRead;
    private final boolean useBR;

    public Elevator(HardwareMap hardwareMap, String elevatorName, BulkRead bRead) {
        elevator = hardwareMap.get(DcMotorEx.class, elevatorName);
        this.bRead = bRead;

        useBR = true;
    }

    public Elevator(HardwareMap hardwareMap, String elevatorName) {
        elevator = hardwareMap.get(DcMotorEx.class, elevatorName);

        useBR = false;
    }

    @Override
    public void updateInput() {

    }
}
