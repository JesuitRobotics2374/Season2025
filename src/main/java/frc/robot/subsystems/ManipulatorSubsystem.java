
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {

  // public TimeOfFlight sensor;
  public TalonFX control;
  public SparkMax eject;

  boolean isIntake = true; // false for outtake, true for intake
  boolean outtaking = false; // to stop motor after outtaking

  public ManipulatorSubsystem() {

    this.eject = new SparkMax(33, MotorType.kBrushless);
    this.control = new TalonFX(21, "rio");
    // this.sensor = new TimeOfFlight(22);

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.signals.primaryEncoderPositionPeriodMs(5);
    eject.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    control.setNeutralMode(NeutralModeValue.Brake);
  }

  public void intake() {
    // if (sensor.getRange() > 100) {
    control.set(-1);
    // }
  }

  public void outtake() {
    control.set(-0.15);
    eject.setVoltage(10);
  }

  public void eject() {
    control.set(1);
  }

  public void stop() {
    control.stopMotor();
    eject.stopMotor();
  }

 

  @Override
  public void periodic() {



    // if (sensor.getRange() <= 100) {
    // control.stopMotor();
    // } else {
    // eject.stopMotor();
    // }
  }
}
