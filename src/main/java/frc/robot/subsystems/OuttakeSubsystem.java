// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.*;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {

    private TalonFXConfiguration config;
    private Map<String, Field> configMap = new HashMap<>();

    //private final TalonFX motorController;

    public OuttakeSubsystem() {
        config = new TalonFXConfiguration();
        //motorController = new TalonFX(19);
        configure();
        //checkConfiguration();
    }

    public void configure() {
        Field[] fields = TalonFXConfiguration.class.getFields();
        for( Field field : fields){
            Field[] subFields = field.getType().getFields();

            try{
                Object obj = field.get(config);
                for(Field subField : subFields) {
                    configMap.put(subField.getName(), subField);
                    //Object value = subField.get(obj);
                    System.out.println(subField.getName());
                }
            } catch(Exception e){
                e.printStackTrace();
            }
        }
        
    }

    public void checkConfiguration() {
        // motorController.getAllConfigs(config);

        File file = new File("/home/lvuser/deploy/talonfx-19-configs-keysheet.txt");
        try{
            Scanner scanner = new Scanner(file);
            String fileValueString = scanner.toString();

            StringTokenizer st = new StringTokenizer(fileValueString);
            while (st.hasMoreTokens()) {
                String preKey = st.nextToken();
                String key = preKey.substring(0, preKey.length() - 2);
                String value = st.nextToken();
                Field field = configMap.get(key);
                field.set(config, convertString(value));
                System.out.println(field.get(config));
            }
            scanner.close();
            

        }catch(Exception e){
            e.printStackTrace();
        }
    }

    private void setSpeed(double speed) {
        //motorController.set(speed);
    }

    private void stop() {
        //motorController.stopMotor();
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

    public Object convertString(String value){
        try{
            return Integer.getInteger(value);
        }catch(Exception e){
            
        }
        try{
            return Double.valueOf(value);
        }catch(Exception e){
            
        }
        try{
            return Boolean.valueOf(value);
        }catch(Exception e){
            
        }
        return value;
    }

}
