
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {

    public CoreCANrange sensor;
    public TalonFX control;
    // public SparkMax eject;

    private boolean isHolding = false;

    boolean algaeIntake = false;

    private boolean isIntaking = false;
    public boolean isOuttaking = false;

    public boolean overriding = false;
    public boolean allowMaxOuttake = false;

    public ManipulatorSubsystem() {

        // this.eject = new SparkMax(33, MotorType.kBrushless);
        this.control = new TalonFX(26, "FastFD");
        this.sensor = new CoreCANrange(27, "FastFD");

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.signals.primaryEncoderPositionPeriodMs(5);
        // eject.configure(config, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);

        control.setNeutralMode(NeutralModeValue.Brake);
    }

    public void intake() {
        isIntaking = true;
        isOuttaking = false;
        control.set(-0.75);
    }

    public void outtake() {
        isIntaking = false;
        isOuttaking = true;
        control.set(1.0);
    }

    public void outtake(double speed) {
        System.out.println("OUTTKAIGN DKFK");
        isIntaking = false;
        isOuttaking = true;
        control.set(speed);
    }

    public void stopOuttake() {
        System.out.println("TSOPING OUTTAKIGN");
        isIntaking = false;
        isOuttaking = false;
        control.set(0);
    }

    public void spinAt(double speed) {
        control.set(-speed);

        if (speed > 0.0) {
            isIntaking = false;
            isOuttaking = true;
        } else {
            isIntaking = true;
            isOuttaking = false;
        }
    }

    public void stop() {
        isIntaking = false;
        isOuttaking = false;
        control.stopMotor();
    }

    public boolean getIsIntaking() {
        return isIntaking;
    }

    int algaeClock = 0;

    public void holdAlgae() {
        algaeIntake = !algaeIntake;
    }

    public void setOverride(boolean x) {
        overriding = x;
    }

    int clock = 11;

    @Override
    public void periodic() {

        clock++;
        algaeClock++;

        boolean withinRange = sensor.getDistance().getValueAsDouble() <= 0.06
                && sensor.getIsDetected().getValueAsDouble() == 1.0;

        if (clock > 10 && withinRange) {
            clock = 5;
        }

        if (!overriding && isIntaking && !isOuttaking && withinRange && clock == 10 && !algaeIntake) {
            // stop();
            isIntaking = false;
        }

        if (algaeIntake && algaeClock == 20) {
            // spinAt(0.6);
        }

        if (algaeClock == 27) {
            algaeClock = 0;
            if (!overriding && algaeIntake) {
                // stop();
            }
        }

        // if (algaeIntake && clock == 12) {
        // intake();
        // }
        // if (algaeIntake && clock == 25) {
        // stop();
        // clock = 0;
        // }
    }
}
