package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.ObjectInputFilter.Config;
import java.security.InvalidParameterException;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    public TalonFX armMotor;
    public SparkMax wristMotor;
    // public CANcoder shaftEncoder;

    // public CANcoderConfiguration coderConfig = new CANcoderConfiguration();
    // private CANrangeConfiguration rangeConfig = new CANrangeConfiguration();

    public ArmSubsystem() {
        this.armMotor = new TalonFX(18);
        this.wristMotor = new SparkMax(0, MotorType.kBrushless);

        this.armMotor.setNeutralMode(NeutralModeValue.Brake);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        this.wristMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

        slot0Configs.kG = 8; //Output of voltage to overcome gravity
        slot0Configs.kV = 1.6; //Output per unit target velocity, perhaps not needed
        slot0Configs.kA = 0.3; //Output per unit target acceleration, perhaps not needed
        slot0Configs.kP = 7; //Controls the response to position error—how much the motor reacts to the difference between the current position and the target position.
        slot0Configs.kI = 1; //Addresses steady-state error, which occurs when the motor doesn’t quite reach the target position due to forces like gravity or friction.
        slot0Configs.kD = 0.1; //Responds to the rate of change of the error, damping the motion as the motor approaches the target. This reduces overshooting and oscillations.

        motionMagicConfigs.MotionMagicCruiseVelocity = 4; // Target velocity in rps
        motionMagicConfigs.MotionMagicAcceleration = 3; // Target acceleration in rps/s
        motionMagicConfigs.MotionMagicJerk = 5; // Target jerk in rps/s/s

        armMotor.getConfigurator().apply(talonFXConfigs);
        armMotor.getConfigurator().apply(slot0Configs);
        armMotor.getConfigurator().apply(motionMagicConfigs);
    }

    public void armUp() {
        MotionMagicVoltage m_request = new MotionMagicVoltage(armMotor.getPosition().getValueAsDouble() + 0.7);
        System.out.println("Up" + (armMotor.getPosition().getValueAsDouble() + 2));

        armMotor.setControl(m_request);
    }

    public void armDown() {
        MotionMagicVoltage m_request = new MotionMagicVoltage(armMotor.getPosition().getValueAsDouble() - 0.7);
        System.out.println("Down" + (armMotor.getPosition().getValueAsDouble() - 2));


        armMotor.setControl(m_request);
    }

    public void zeroSystem() {
        MotionMagicVoltage m_request = new MotionMagicVoltage(0);

        armMotor.setPosition(0.0);
        armMotor.setControl(m_request);
    }

    public void stopArm() {
        armMotor.stopMotor();
        armMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void rotateIntake() {
        rotateTo(0.25, 0.1);
    }

    public void rotateOuttake() {
        rotateTo(0, -0.1);
    }

    private void rotateTo(double position, double speed) {
        while (wristMotor.getEncoder().getPosition() < position) {
            wristMotor.set(speed);
        }
        wristMotor.stopMotor();
    }

    @Override
    public void periodic() {
    }
}
  