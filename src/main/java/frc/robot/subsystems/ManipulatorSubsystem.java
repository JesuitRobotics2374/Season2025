
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
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

  public CANrange sensor;
  public TalonFX control;

  public ManipulatorSubsystem() {

    this.control = new TalonFX(39, "rio");
    this.sensor = new CANrange(22, "rio");

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.signals.primaryEncoderPositionPeriodMs(5);

    control.setNeutralMode(NeutralModeValue.Brake);
  }

  public void intake() {
    spinAt(-0.5);
  }

  public void outtake() {
    spinAt(0.5);
  }

  public void spinAt(double speed) {
    control.set(-speed);
  }

  public void stop() {
    control.stopMotor();
  }

  // public void holdAlgae() {
  //   algaeIntake = !algaeIntake;
  //   clock = 0;
  // }

 
  int clock = 11;

  public void periodic() {

    System.out.println(sensor.getDistance().getValueAsDouble());

    clock++;

    boolean withinRange = sensor.getDistance().getValueAsDouble() <= 100;

    if (clock > 10 && withinRange) {
        clock = 0;
    }

    if (withinRange && clock == 10) {
        stop();
    }

}
}
