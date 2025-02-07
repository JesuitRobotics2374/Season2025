
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {

  TimeOfFlight sensor;
  TalonFX control;
  SparkMax eject;

  boolean isIntake = true;  //false for outtake, true for intake
  boolean outtaking = false; //to stop motor after outtaking

  public ManipulatorSubsystem() {
    this.sensor = new TimeOfFlight(0);
    this.control = new TalonFX(0);
    this.eject = new SparkMax(0, MotorType.kBrushless);
  }

  private void spinControl(double speed) {
    control.set(speed);
  }

  private void spinEject(double speed) {
    eject.set(speed);
  }

  public void intake() {
    if (!isIntake) {
      spinControl(0.1);
    }
  }
  
  public void outtake() {
    spinEject(-0.1);
  }

  public void stopControl() {
	  control.stopMotor();
  }

  public void stopEject() {
	  eject.stopMotor();
  }

  @Override
  public void periodic() {
    if (sensor.getRange() <= 100) {
      isIntake = true;
    }
    if(sensor.getRange() > 100) {
	    isIntake = false;
    }
  }
}
