
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {

    private enum State {
        INTAKING,
        OUTTAKING,
        LOADED,
        EMPTY
    }

    public CoreCANrange sensor;
    public TalonFX control;

    private State currentState;

    private final double defaultIntakeRPM = 60;
    private final double defaultOuttakeRPM = 60;
    private final double distanceThreshold = 0.1;

    public ManipulatorSubsystem() {

        this.control = new TalonFX(26, "FastFD");
        this.sensor = new CoreCANrange(27, "FastFD");

        control.setNeutralMode(NeutralModeValue.Brake);
    }

    public SequentialCommandGroup intake() {
        return intake(defaultIntakeRPM);
    }

    public SequentialCommandGroup intake(double rpm) {
        SequentialCommandGroup group = new SequentialCommandGroup();
        Command c = new InstantCommand(() -> runIntake(rpm));
        group.addCommands(c);

        return group;
    }

    public SequentialCommandGroup outtake() {
        return outtake(defaultOuttakeRPM);
    }

    public SequentialCommandGroup outtake(double rpm) {
        SequentialCommandGroup group = new SequentialCommandGroup();
        Command c = new InstantCommand(() -> runOuttake(rpm));
        group.addCommands(c);

        return group;
    }

    private void runOuttake(double rpm) {
        control.set(rpm); // TODO: FIGURE OUT MATH FOR THIS, should be Normalrpm/Maxrpm
    }

    private void runIntake(double rpm) {
        control.set(rpm); // TODO: FIGURE OUT MATH FOR THIS, should be Normalrpm/Maxrpm
    }

    public boolean isGamepiecePresent() {
        return sensor.getDistance().getValueAsDouble() <= distanceThreshold;
    }

    public State mechanismStatus() { // INTAKING, OUTTAKING, LOADED, EMPTY
        return currentState;
    }

    public double getDefaultIntakeRpm() {
        return defaultIntakeRPM;
    }

    public double getDefaultOuttakeRpm() {
        return defaultOuttakeRPM;
    }

    // public SequentialCommandGroup intake(double setspeed) {
    // SequentialCommandGroup group = new SequentialCommandGroup();
    // Command c = new InstantCommand(() -> runIntake(setspeed));
    // group.addCommands(c);

    // return group;
    // }

    // public SequentialCommandGroup outtake(double setspeed) {
    // SequentialCommandGroup group = new SequentialCommandGroup();
    // Command c = new InstantCommand(() -> runOuttake(setspeed));
    // group.addCommands(c);

    // return group;
    // }

    // public SequentialCommandGroup intake() {
    // return this.intake(defaultSpeed);
    // }

    // public SequentialCommandGroup outtake() {
    // return this.intake(defaultSpeed);
    // }

    // private void runIntake(double speed) {
    // if (isHolding) {
    // return;
    // }

    // control.set(speed);

    // isIntaking = true;
    // }

    // private void runOuttake(double speed) {
    // if (!isHolding) {
    // return;
    // }

    // control.set(speed);

    // isOuttaking = false;
    // }

    // private void stop() {
    // control.set(0);
    // }

    // // public void setOverride(boolean x) {
    // // overriding = x;
    // // }

    int clock = 0;

    @Override
    public void periodic() {

        // if (algaeIntake && clock == 12) {
        // intake();
        // }
        // if (algaeIntake && clock == 25) {
        // stop();
        // clock = 0;
        // }
    }
}
