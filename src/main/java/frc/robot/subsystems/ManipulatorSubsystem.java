
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {

  TimeOfFlight sensor;
  TalonFX outerWheels;
  SparkFlex innerWheels;

  boolean type = true;  //false for outtake, true for intake

  public ManipulatorSubsystem() {
  }

  private void spinOuter(double speed) {
    outerWheels.set(speed);
  }

  private void spinInner(double speed) {
    innerWheels.set(speed);
  }

  public void intakeOuter() {
    spinOuter(0.1);
  }
  
  public void outtakeOuter() {
    spinOuter(-0.1);
  }

  public void intakeInner() {
    spinInner(0.1);
  }
  
  public void outtakeInner() {
    spinInner(-0.1);
  }

  public void stopOuter() {
	  outerWheels.stopMotor();
  }

  public void stopInner() {
	  innerWheels.stopMotor();
  }

  @Override
  public void periodic() {
    if(sensor.getRange() < 50 && type) {
	    outerWheels.stopMotor();
    }
  }
}
