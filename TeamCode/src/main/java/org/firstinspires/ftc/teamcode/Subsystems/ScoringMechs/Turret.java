package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BulkRead;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public class Turret implements Input {

    //Turret motor
    public DcMotorEx turret;

    //Bulk Read
    public BulkRead bRead;
    private final boolean useBR;

    public Turret(HardwareMap hardwareMap, String turretName, BulkRead bRead) {
        turret = hardwareMap.get(DcMotorEx.class, turretName);
        this.bRead = bRead;

        useBR = true;
    }

    public Turret(HardwareMap hardwareMap, String turretName) {
        turret = hardwareMap.get(DcMotorEx.class, turretName);

        useBR = false;
    }

    @Override
    public void updateInput() {

    }
}
