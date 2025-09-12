
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

    // private boolean isHolding = false;

    // boolean algaeIntake = false;

    private boolean isIntaking = false;
    public boolean isOuttaking = false;

    public boolean overriding = false;
    public boolean allowMaxOuttake = false;

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

    public void intake() {
        control.set(1.0);
    }

    public void debugIntake() {
        control.set(0.2);
    }

    public void outtake() {
        control.set(-1.0);
    }

    public void spinAt(double speed) {
        control.set(speed);
    }

    public void stop() {
        control.stopMotor();
    }

    public boolean getIsIntaking() {
        return isIntaking;
    }

    public void setOverride(boolean x) {
        overriding = x;
    }


    @Override
    public void periodic() {

        
    }
}
