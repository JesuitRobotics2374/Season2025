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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    // Members
    public TalonFX armMotor2;
    public TalonFX wristMotor;
    public CANcoder armEncoder;
    public CANcoder wristEncoder;
    public double armGoal;

    // Constructor
    public ArmSubsystem(){}

    // Methods
    public void zeroArm(){}
    public boolean armPassedGoal(){return true;}
    public void armUp(){}
    public void armDown(){}
    public void armGoTo(double pos){}
    public void armChangeBy(double pos){}
    public void setZero(){}
    public void stopArm(){}
    public void rotateWristIntake(){}
    public void rotateWristOuttake(){}
    public void wristCW(){}
    public void wristCCW(){}
    public void wristGoTo(double pos){}
    @Override
    public void periodic(){}

}


// public class ArmSubsystem extends SubsystemBase {

//     // public TalonFX armMotor1;
//     public TalonFX armMotor2;
//     public TalonFX wristMotor;

//     public CANcoder armEncoder;
//     public CANcoder wristEncoder;

//     public double armGoal = Integer.MAX_VALUE; // Data storage variable 

//     // private double wristTarget = Double.NaN;

//     // public CANcoderConfiguration coderConfig = new CANcoderConfiguration();
//     // private CANrangeConfiguration rangeConfig = new CANrangeConfiguration();

//     public ArmSubsystem() {
//         // this.armMotor1 = new TalonFX(11, "rio");
//         this.armMotor2 = new TalonFX(16, "FastFD");
//         this.wristMotor = new TalonFX(17, "FastFD");

//         this.armEncoder = new CANcoder(28, "FastFD");
//         this.wristEncoder = new CANcoder(29, "FastFD");

//         // this.armMotor1.setNeutralMode(NeutralModeValue.Brake);
//         this.armMotor2.setNeutralMode(NeutralModeValue.Brake);

//         TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
//         Slot0Configs slot0Configs = talonFXConfigs.Slot0;
//         MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

//         slot0Configs.kG = 8.5; // Output of voltage to overcome gravity
//         slot0Configs.kV = 2; // Output per unit target velocity, perhaps not needed
//         slot0Configs.kA = 0.3; // Output per unit target acceleration, perhaps not needed
//         slot0Configs.kP = 15; // Controls the response to position error—how much the motor reacts to the
//                              // difference between the current position and the target position.
//         slot0Configs.kI = 1.5; // Addresses steady-state error, which occurs when the motor doesn’t quite reach
//                              // the target position due to forces like gravity or friction.
//         slot0Configs.kD = 0.3; // Responds to the rate of change of the error, damping the motion as the motor
//                                // approaches the target. This reduces overshooting and oscillations.

//         talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

//         motionMagicConfigs.MotionMagicCruiseVelocity = 50; // Target velocity in rps
//         motionMagicConfigs.MotionMagicAcceleration = 32; // Target acceleration in rps/s
//         motionMagicConfigs.MotionMagicJerk = 400; // Target jerk in rps/s/s

//         armMotor2.getConfigurator().apply(talonFXConfigs);
//         armMotor2.getConfigurator().apply(slot0Configs);
//         armMotor2.getConfigurator().apply(motionMagicConfigs);

//         /////

//         TalonFXConfiguration talonFXConfigsWrist = new TalonFXConfiguration();
//         Slot0Configs slot0ConfigsWrist = talonFXConfigs.Slot0;
//         MotionMagicConfigs motionMagicConfigsWrist = talonFXConfigs.MotionMagic;

//         slot0ConfigsWrist.kG = 0; // Output of voltage to overcome gravity
//         slot0ConfigsWrist.kV = 2; // Output per unit target velocity
//         slot0ConfigsWrist.kA = 0.3; // Output per unit target acceleration
//         slot0ConfigsWrist.kP = 8; // Controls the response to position error—how much the motor reacts to the
//         // difference between the current position and the target position.
//         slot0ConfigsWrist.kI = 0.1; // Addresses steady-state error, which occurs when the motor doesn’t quite reach
//         // the target position due to forces like gravity or friction.
//         slot0ConfigsWrist.kD = 0.25; // Responds to the rate of change of the error, damping the motion as the motor
//         // approaches the target. This reduces overshooting and oscillations.

//         motionMagicConfigsWrist.MotionMagicCruiseVelocity = 70; // Target velocity in rps
//         motionMagicConfigsWrist.MotionMagicAcceleration = 50; // Target acceleration in rps/s
//         motionMagicConfigsWrist.MotionMagicJerk = 450; // Target jerk in rps/s/s

//         wristMotor.getConfigurator().apply(talonFXConfigsWrist);
//         wristMotor.getConfigurator().apply(slot0ConfigsWrist);
//         wristMotor.getConfigurator().apply(motionMagicConfigsWrist);

//         // armMotor1.setPosition(shaftEncoder.getPosition().getValueAsDouble() * 125);

//         // armMotor2.setControl(new Follower(armMotor1.getDeviceID(), true));

//         armMotor2.setPosition(armEncoder.getAbsolutePosition().getValueAsDouble() * Constants.ARM_RATIO);
//         wristMotor.setPosition(wristEncoder.getAbsolutePosition().getValueAsDouble() * Constants.WRIST_RATIO);
//         MotionMagicVoltage m_request = new MotionMagicVoltage(armMotor2.getPosition().getValueAsDouble() + 3);
//         MotionMagicVoltage m_requestWrist = new MotionMagicVoltage(wristMotor.getPosition().getValueAsDouble());
//         armMotor2.setControl(m_request);
//         wristMotor.setControl(m_requestWrist);

//     }

//     public void zeroArm() {
//         armMotor2.setPosition(armEncoder.getAbsolutePosition().getValueAsDouble() * Constants.ARM_RATIO);
//         MotionMagicVoltage m_request = new MotionMagicVoltage(armMotor2.getPosition().getValueAsDouble());
//         armMotor2.setControl(m_request);
//     }

//     public boolean armPassedGoal() {
//         return armMotor2.getPosition().getValueAsDouble() >= armGoal;
//     }

//     public void armUp() {
//         armGoal = armMotor2.getPosition().getValueAsDouble() + 2;
        
//         MotionMagicVoltage m_request = new MotionMagicVoltage(armMotor2.getPosition().getValueAsDouble() + 2);

//         armMotor2.setControl(m_request);
//     }

//     public void armDown() {
//         armGoal = armMotor2.getPosition().getValueAsDouble() - 2;

//         MotionMagicVoltage m_request = new MotionMagicVoltage(armMotor2.getPosition().getValueAsDouble() - 2);

//         armMotor2.setControl(m_request);
//     }

//     public void armGoTo(double pos) {
//         armGoal = pos;

//         MotionMagicVoltage m_request = new MotionMagicVoltage(pos);

//         armMotor2.setControl(m_request);
//     }

//     public void armChangeBy(double pos) {
//         armGoal = armMotor2.getPosition().getValueAsDouble() + pos;

//         MotionMagicVoltage m_request = new MotionMagicVoltage(armMotor2.getPosition().getValueAsDouble() + pos);
//         armMotor2.setControl(m_request);
//     }

//     public void setZero() {
//         armGoal = 0.0;

//         wristEncoder.setPosition(0.0);
//         armMotor2.setPosition(0.0);

//         MotionMagicVoltage m_request = new MotionMagicVoltage(0);
//         armMotor2.setControl(m_request);
//     }

//     public void stopArm() {
//         armMotor2.stopMotor();
//         armMotor2.setNeutralMode(NeutralModeValue.Brake);
//     }

//     public void rotateWristIntake() {
//         MotionMagicVoltage m_request = new MotionMagicVoltage(Constants.WRIST_MAX_POSITION * Constants.WRIST_RATIO);
//         wristMotor.setControl(m_request);
//     }

//     public void rotateWristOuttake() {
//         MotionMagicVoltage m_request = new MotionMagicVoltage(Constants.WRIST_MIN_POSITION * Constants.WRIST_RATIO);
//         wristMotor.setControl(m_request);
//     }

//     public void wristCW() {
//         // if (goal + Constants.WRIST_INCREMENT <= Constants.WRIST_MAX_POSITION) {
//         MotionMagicVoltage m_request = new MotionMagicVoltage(
//                 wristMotor.getPosition().getValueAsDouble() - Constants.WRIST_INCREMENT);
//         wristMotor.setControl(m_request);
//         // }
//     }

//     public void wristCCW() {
//         // if (goal - Constants.WRIST_INCREMENT >= Constants.WRIST_MIN_POSITION) {
//         MotionMagicVoltage m_request = new MotionMagicVoltage(
//                 wristMotor.getPosition().getValueAsDouble() + Constants.WRIST_INCREMENT);
//         wristMotor.setControl(m_request);
//         // }
//     }

//     public void wristGoTo(double pos) {
//         MotionMagicVoltage m_request = new MotionMagicVoltage(pos * Constants.WRIST_RATIO);
//         wristMotor.setControl(m_request);
//     }

//     // public void rotateWristTo(double ctrePosition, double speed) {
//     // wristTarget = ctrePosition;
//     // double current = wristEncoder.getPosition().getValueAsDouble();
//     // if (current < wristTarget) {
//     // wristMotor.set(Math.abs(speed));
//     // } else {
//     // wristMotor.set(-Math.abs(speed));
//     // }
//     // }

//     int clock = 0;

//     @Override
//     public void periodic() {

//         clock++;
//         if (clock == 20) {
//             System.out.println(armMotor2.getPosition().getValueAsDouble());
//             clock = 0;
//         }

//         // double wristVel =
//         // wristController.calculate(wristMotor.getEncoder().getPosition(),
//         // wristMotor.getEncoder().getVelocity());
//         // wristVel = Math.signum(wristVel) * Math.min(Math.abs(wristVel),
//         // Constants.WRIST_MAX_SPEED); // Limits speed
//         // wristMotor.set(wristVel);
//     }

// }
