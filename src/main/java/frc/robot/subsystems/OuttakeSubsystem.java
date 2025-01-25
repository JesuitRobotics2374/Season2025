// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.core.CoreCANrange;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {

  public final WPI_TalonSRX motorController;
  public CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
  public CoreCANrange cCANrange = new CoreCANrange(19);

  public OuttakeSubsystem() {
    
    motorController = new WPI_TalonSRX(40);

    motorController.setNeutralMode(NeutralMode.Brake);

  }

  // Typeset

  public void getDistance() {
    StatusSignal<Distance> d = cCANrange.getDistance();
    System.out.println(d);
  }

  private void setSpeed(double speed) {
    motorController.set(speed);
  }

  private void stop() {
    motorController.stopMotor();
  }

  // Templates

  public void intake() {
    System.out.println("in");
    setSpeed(0.2);
  }

  public void outtake() {
    setSpeed(-0.2);
  }

  public void stopIntake() {
    stop();
  }

  @Override
  public void periodic() {
  }

}
