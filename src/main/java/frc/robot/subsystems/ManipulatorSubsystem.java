
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

    private void runIntake(double rpm) {
        if (currentState != State.EMPTY) {
            return;
        }

        currentState = State.INTAKING;

        control.set(rpm); // TODO: FIGURE OUT MATH FOR THIS, should be Normalrpm/Maxrpm
    }

    private void runOuttake(double rpm) {
        if (currentState != State.LOADED) {
            return;
        }

        currentState = State.OUTTAKING;

        control.set(rpm); // TODO: FIGURE OUT MATH FOR THIS, should be Normalrpm/Maxrpm
    }

    private void stop() {
        control.set(0);
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

    int clock = 0;
    private boolean intakeStartTimer = false;
    private double intakeStopThreshold = 25;
    private boolean outtakeStartTimer = true;
    private double outtakeStopThreshold = 25;

    @Override
    public void periodic() {
        if (intakeStartTimer) {
            clock++;

            if (clock > intakeStopThreshold) {
                stop();
                clock = 0;
                currentState = State.LOADED;
            }
        }

        if (outtakeStartTimer) {
            clock++;

            if (clock > outtakeStopThreshold) {
                stop();
                clock = 0;
                currentState = State.EMPTY;
            }
        }

        if (currentState == State.INTAKING) {
            if (sensor.getDistance().getValueAsDouble() <= distanceThreshold) {
                intakeStartTimer = true;
            }
        }
        else if (currentState == State.OUTTAKING) {
            if (sensor.getDistance().getValueAsDouble() > distanceThreshold) {
                outtakeStartTimer = true;
            }
        }
        else if (isGamepiecePresent()) {
            currentState = State.LOADED;
        }
        else {
            currentState = State.EMPTY;
        }
    }
}
