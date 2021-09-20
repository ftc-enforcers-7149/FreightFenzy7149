package org.firstinspires.ftc.teamcode.Mattu.BatteryMonitoring;

import com.qualcomm.hardware.lynx.LynxCommExceptionHandler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

public class BatteryMonitor extends LynxCommExceptionHandler {

    public LynxModule mainHub, secondaryHub;
    private final boolean hasSecondary;

    private DcMotorEx[] motors;

    public BatteryMonitor(HardwareMap hardwareMap, String mainHub) {
        this.mainHub = hardwareMap.get(LynxModule.class, mainHub);
        hasSecondary = false;

        motors = new DcMotorEx[4];
    }

    public BatteryMonitor(HardwareMap hardwareMap, String mainHub, String secondaryHub) {
        this.mainHub = hardwareMap.get(LynxModule.class, mainHub);
        this.secondaryHub = hardwareMap.get(LynxModule.class, secondaryHub);
        hasSecondary = true;

        motors = new DcMotorEx[8];
    }

    public void addMainHubMotor(DcMotorEx motor) {
        motors[motor.getPortNumber()] = motor;
    }

    public void addSecondaryHubMotor(DcMotorEx motor) {
        if (hasSecondary) motors[motor.getPortNumber() + 4] = motor;
    }

    public double getMotorCurrent(DcMotorEx motor) {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    /***
     * Get the voltage reported by the main hub's 12v rail monitor
     * @return the voltage reported by the main hub's 12v monitor
     */
    public double getMainVoltage() {
        return mainHub.getInputVoltage(VoltageUnit.VOLTS);
    }

    /***
     * Get the current reported by the main hub's current monitor
     * @return the current reported by the main hub's current monitor
     */
    public double getMainCurrent() {
        return mainHub.getCurrent(CurrentUnit.AMPS);
    }

    /***
     * Get the voltage reported by the secondary hub's 12v rail monitor
     * @return the voltage reported by the secondary hub's 12v monitor
     */
    public double getSecondaryVoltage() {
        if (hasSecondary) return secondaryHub.getInputVoltage(VoltageUnit.VOLTS);
        return 0;
    }

    /***
     * Get the current reported by the secondary hub's current monitor
     * @return the current reported by the secondary hub's current monitor
     */
    public double getSecondaryCurrent() {
        if (hasSecondary) return secondaryHub.getCurrent(CurrentUnit.AMPS);
        return 0;
    }

    public String toString() {
        String ret = "";

        ret += "Main Voltage: " + getMainVoltage() + " V\n";
        ret += "Main Current: " + getMainCurrent() + " A\n";
        if (hasSecondary) {
            ret += "\nSecondary Voltage: " + getSecondaryVoltage() + " V\n";
            ret += "Secondary Current: " + getSecondaryCurrent() + " A\n";
        }
        ret += "\nMain Hub - Motor Currents";
        for (int i = 0; i < 4; i++) {
            if (motors[i] != null) ret += "\nPort " + i + ": " + getMotorCurrent(motors[i]) + " A";
        }
        if (hasSecondary) {
            ret += "\n\nSecondary Hub - Motor Currents";
            for (int i = 4; i < 8; i++) {
                if (motors[i] != null) ret += "\nPort " + (i - 4) + ": " + getMotorCurrent(motors[i]) + " A";
            }
        }

        return ret;
    }
}
