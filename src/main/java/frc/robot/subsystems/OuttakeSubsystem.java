// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class OuttakeSubsystem extends SubsystemBase {

  private SparkMaxConfig config;
  private SparkMax motorController;
  private MotorType kBrushless;
  private SparkMaxConfigAccessor configAccessor;
  
  public OuttakeSubsystem() {
    config = new SparkMaxConfig();
    motorController = new SparkMax(40, kBrushless);
    configure(config);
    checkConfiguration();
  }

  public void configAllSettings(SparkMaxConfig config) {
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(1);
  } 
  
  public void configure(SparkMaxConfig config) {
    configAllSettings(config);
  }

  public void checkConfiguration() {
    configAccessor.getSmartCurrentLimit();
    configAccessor.getIdleMode();
    
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
