// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {

  public final WPI_TalonSRX manipulator;

  public OuttakeSubsystem() {
    
    manipulator = new WPI_TalonSRX(40);

    manipulator.setNeutralMode(NeutralMode.Brake);

  }

  public void setSpeed(double speed) {
    manipulator.set(speed);
  }

  public void stop() {
    manipulator.stopMotor();
  }

  // Template modes

  public void intake() {
    setSpeed(0.5);
  }

  public void outtake() {
    setSpeed(-0.5);
  }

  public void stopIntake() {
    stop();
  }

  @Override
  public void periodic() {
  }

}
