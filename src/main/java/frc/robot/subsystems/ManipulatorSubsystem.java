
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {

  //TimeOfFlight sensor;
  TalonFX control;
  SparkMax eject;

  boolean isIntake = true;  //false for outtake, true for intake
  boolean outtaking = false; //to stop motor after outtaking

  public ManipulatorSubsystem() {
    
    this.eject = new SparkMax(20, MotorType.kBrushless);
    this.control = new TalonFX(21);
    //this.sensor = new TimeOfFlight(22);

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    eject.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    control.setNeutralMode(NeutralModeValue.Brake);
  }

  public void intake() {
    //if (sensor.getRange() > 100) {
      control.set(-0.1);
    //}
  }
  
  public void outtake() {
    eject.set(-0.1);
  }

  public void outtakeAlgae() {
    control.set(0.1);
  }

  public void stop() {
    control.stopMotor();
    eject.stopMotor();
  }

  // @Override
  // public void periodic() {
  //   if (sensor.getRange() <= 100) {
  //     control.stopMotor();
  //   } else {
  //     eject.stopMotor();
  //   }
  // }
}
