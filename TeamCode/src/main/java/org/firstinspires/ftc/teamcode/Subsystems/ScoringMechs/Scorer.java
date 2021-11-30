package org.firstinspires.ftc.teamcode.Subsystems.ScoringMechs;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Output;

public class Scorer implements Output {

    //Inputs
    public Elevator elevator;
    public Turret turret;

    //Motors
    public DcMotorEx eleMotor, turMotor;

    public Scorer(Elevator elevator, Turret turret) {
        this.elevator = elevator;
        this.turret = turret;

        eleMotor = elevator.elevator;
        turMotor = turret.turret;
    }

    @Override
    public void updateOutput() {

    }

    @Override
    public void stopOutput() {

    }
}
