// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.config.BaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {

  private SparkBaseConfig config;
  private final SparkMax motorController;
  private final MotorType kBrushless;
  
  public OuttakeSubsystem() {
    config = new SparkBaseConfig();
    motorController = new SparkMax(40, kBrushless);
    configure();
    checkConfiguration();
  }

  public ErrorCode configAllSettings(SparkBaseConfig allConfigs) {
    //NeutralMode

    ErrorCode errorCode = motorController.configAllSettings(allConfigs);
    return errorCode;
  } 
  
  public void configure(SparkBaseConfig config, SparkBase.) {
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
