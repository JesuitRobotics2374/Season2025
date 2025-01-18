// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.*;
import java.util.*;

import org.json.simple.JSONObject;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {

  private TalonFXConfiguration config;
  private TalonFXConfigurator configurator;

  private final TalonFX motorController;

  public OuttakeSubsystem() {
    config = new TalonFXConfiguration();
    motorController = new TalonFX(19);
    configure();
    checkConfiguration();
    createConfigFile(config);
    //readConfigFile(config, new File("/home/lvuser/deploy/talonfx-19-configs.txt"));
  }

  public void configAllSettings(TalonFXConfiguration allConfigs) {
    configurator = motorController.getConfigurator();
    configurator.refresh(allConfigs);
  // ErrorCode errorCode = TalonFXConfiguration.configAllSettings(allConfigs);
  //  return errorCode;
  
  }

  public void configure() {
    motorController.setNeutralMode(NeutralModeValue.Brake);
    configAllSettings(config);
  }

  public void checkConfiguration() {
    
    System.out.println("Intake Configs: ");
    System.out.println(config);
  }

  //Make sure to test this!!!
  public void createConfigFile( TalonFXConfiguration config){
    try{
      File configFile = new File("/home/lvuser/talonfx-19-configs.txt");
      FileWriter writer = new FileWriter(configFile);
      writer.write(config.serialize());
      writer.close();
    }
    catch (IOException e){
      System.out.println("File not created");
    }
  }

  public void readConfigFile(TalonFXConfiguration config, File file){
    try{
      Scanner scanner = new Scanner(file);
      StringBuilder sb = new StringBuilder();
      while (scanner.hasNextLine()) {
        String data = scanner.nextLine();
        sb.append(data);
      }
      String value = sb.toString();
      config.deserialize(value);
      System.out.println(config.deserialize(value));
      scanner.close();
    }catch(IOException e){
      System.out.println("File was not able to be read");
    }
    
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
