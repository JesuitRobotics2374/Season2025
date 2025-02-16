package frc.robot.utils;

public class Setpoint {

    double elevator;
    double arm;
    double wrist;

    public Setpoint(double elevator, double arm, double wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
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

}