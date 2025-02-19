
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

  public TimeOfFlight sensor;
  public TalonFX control;
  //public SparkMax eject;

  private boolean isHolding = false;

  boolean algaeIntake = false;

  public ManipulatorSubsystem() {

    //this.eject = new SparkMax(33, MotorType.kBrushless);
    this.control = new TalonFX(21, "rio");
    this.sensor = new TimeOfFlight(22);

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.signals.primaryEncoderPositionPeriodMs(5);
    //eject.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    control.setNeutralMode(NeutralModeValue.Brake);
  }

  public void intake() {
      control.set(-0.5);
  }

  public void outtake() {
    control.set(-0.15);
    //eject.setVoltage(10);
  }

  public void eject() {
    control.set(0.5);
  }

  public void spinAt(double speed) {
    control.set(-speed);
  }

  public void stop() {
    control.stopMotor();
    //eject.stopMotor();
  }

  // public void holdAlgae() {
  //   algaeIntake = !algaeIntake;
  //   clock = 0;
  // }

 
  int clock = 0;

  @Override
  public void periodic() {

    clock++;

    if (sensor.getRange() <= 100) {
      isHolding = true;
    } else {
      isHolding = false;
    }

    if (isHolding && clock == 5) {
      stop();
    }

    if (clock == 10) {
      clock = 0;
    }
  

    // if (algaeIntake && clock == 12) {
    //   intake();
    // }
    // if (algaeIntake && clock == 25) {
    //   stop();
    //   clock = 0;
    // }
  }
}
