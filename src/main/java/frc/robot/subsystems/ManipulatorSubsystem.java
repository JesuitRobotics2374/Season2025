
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {

    public enum FeedState {
        EMPTY, LOADING, LOADED
    }

    private CoreCANrange sensor;
    private TalonFX control;

    private boolean movingReverse = false;
    private boolean feeding = false;

    private FeedState state = FeedState.EMPTY;

    private int controlledEjectionClock = 0;

    public ManipulatorSubsystem() {

        // this.eject = new SparkMax(33, MotorType.kBrushless);
        this.control = new TalonFX(38, "FastFD");
        this.sensor = new CoreCANrange(27, "FastFD");

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.signals.primaryEncoderPositionPeriodMs(5);
        // eject.configure(config, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);

        control.setNeutralMode(NeutralModeValue.Brake);
    }

    public void reverse() {
        control.set(0.2);
    }

    public void feed() {
        control.set(-1.0);
    }

    public void preFeed() {
        control.set(-0.2);
    }

    public void forceEjectFast() {
        control.set(-1);
    }

    public void spinAt(double speed) {
        control.set(speed);
    }

    public void stop() {
        control.stopMotor();
    }

    public boolean isFeeding() {
        return feeding;
    }

    public boolean isReversing() {
        return movingReverse;
    }

    public boolean isPieceDetected() {
        return sensor.getIsDetected().getValue();
    }

    public Command load() {
        if (state != FeedState.EMPTY) {
            return new InstantCommand();
        }
        return new FunctionalCommand(
            // On Init:
            () -> {},
            // Every frame:
            () -> this.preFeed(),
            // When we stop:
            interrupted -> this.stop(),
            // End the command when:
            () -> this.isPieceLoaded(),
            // Require this subsystem
            this
        );
    }

    public Command eject() {
        return new FunctionalCommand(
            // On Init:
            () -> {
                controlledEjectionClock = 0;
            },
            // Every frame:
            () -> {
                this.forceEjectFast();
                controlledEjectionClock++;
            },
            // When we stop:
            interrupted -> {
                this.stop();
                setState(FeedState.EMPTY);
            },
            // End the command when:
            () -> (controlledEjectionClock>=20),
            // Require this subsystem
            this
        );
    }

    private void updateState() {
        switch (state) {
            case EMPTY:
                if (isPieceDetected()) {
                    setState(FeedState.LOADING);
                }
                break;
            case LOADING:
                if (!isPieceDetected()) {
                    setState(FeedState.LOADED);
                }
                break;
            case LOADED:
                break;
        }
    }

    private boolean isPieceLoaded() {
        return state == FeedState.LOADED;
    }

    private void setState(FeedState newState) {
        state = newState;
    }

    public FeedState getState() {
        return state;
    }

    public void clearLoaded() {
        if (state != FeedState.LOADED) {
            System.out.println("WARN: STATE WAS NOT LOADED");
        }
        state = FeedState.EMPTY;
    }

    private int clock = 0;

    @Override
    public void periodic() {
        updateState();

/*         clock++;
        if (clock == 50) {
            clock = 0;
            System.out.println(state);
            System.out.println("detect: " + sensor.getIsDetected().getValue());
        } */
    }
}
