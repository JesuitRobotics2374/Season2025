// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.*;
import java.lang.reflect.Field;
import java.util.*;

import org.json.simple.JSONObject;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase{

  private TalonFXConfiguration config;
  private Map<String, Object> configMap = new HashMap<>();
  private TalonFXConfigurator configurator;

  private final TalonFX motorController;

  public OuttakeSubsystem() {
    config = new TalonFXConfiguration();
    motorController = new TalonFX(19);

    configure();
    createConfigFile(config);
    //readConfigFile(config, new File("/home/lvuser/deploy/talonfx-19-configs-keysheet.txt"));
  }

  public void configAllSettings(TalonFXConfiguration allConfigs) {
    configurator = motorController.getConfigurator();
    configurator.refresh(allConfigs);
  }

  public void configure() {
    motorController.setNeutralMode(NeutralModeValue.Brake);
    configAllSettings(config);
  }

  public void createConfigFile(TalonFXConfiguration config){
    //File configFile = new File("/home/lvuser/talonfx-19-configs.json");
    Field[] fields = TalonFXConfiguration.class.getFields();
    
    for(Field field : fields){
      
        field.setAccessible(true);
        Field[] subFields = field.getType().getFields();

        try{
          Object obj = field.get(config);
          for(Field subField : subFields){
            subField.setAccessible(true);
            Object value = subField.get(obj);
            configMap.put(subField.getName(), value);
          }
          
        }catch(Exception e){
          
        }
    }
  }

  public void readConfigFile(TalonFXConfiguration config, File file){
    
  }

  public Object convertString(String value){
    try{
        return Integer.parseInt(value);
    }catch(NumberFormatException e){

    }

    try{
        return Double.parseDouble(value);
    }catch(NumberFormatException e){

    }

    try{
        return Boolean.parseBoolean(value);
    }catch(NumberFormatException e){

    }


    return value;

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
