package frc.robot.seafinder.utils;

import java.util.ArrayList;

public class Setpoint {

    double elevator;
    double arm;
    double wrist;
    String retractAction;

    public Setpoint(double elevator, double arm, double wrist, String retractAction) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.retractAction = retractAction;
    }

    public double getElevator() {
        return elevator;
    }

    public double getArm() {
        return arm;
    }

    public double getWrist() {
        return wrist;
    }

    public String getRetractAction() {
        return retractAction;
    }

}