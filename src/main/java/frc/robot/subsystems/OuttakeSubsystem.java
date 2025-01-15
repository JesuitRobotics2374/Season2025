// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//update com.ctre to phoenix 6

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {

  private TalonFXConfiguration config;

  private final TalonFX motorController;

  public OuttakeSubsystem() {
    config = new TalonFXConfiguration();
    motorController = new TalonFX(19);
    configure();
    checkConfiguration();
  }

  public ErrorCode configAllSettings(TalonFXConfiguration allConfigs) {
    //allConfigs.continuousCurrentLimit = 1;
    //allConfigs.peakCurrentDuration = 1;
    //allConfigs.peakCurrentLimit = 1;
    //NeutralMode
    
    ErrorCode errorCode = motorController.configAllSettings(allConfigs);
    return errorCode;
  } 

  public void configure() {
    motorController.setNeutralMode(NeutralMode.Brake);
    configAllSettings(config);
  }

  public void checkConfiguration() {
    motorController.getAllConfigs(config);
    System.out.println("Intake Configs: ");
    System.out.println(config);
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
