package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class TestEject extends SubsystemBase {

  SparkMax eject;

  public TestEject() {
    this.eject = new SparkMax(33, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    
    config.idleMode(IdleMode.kBrake);
    config.signals.primaryEncoderPositionPeriodMs(5);
    eject.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void outtake() {
    eject.set(0.1);
  }

  public void stop() {
    
    eject.stopMotor();
  }
}