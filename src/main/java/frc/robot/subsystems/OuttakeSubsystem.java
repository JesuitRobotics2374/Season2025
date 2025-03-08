// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.channels.Pipe.SourceChannel;
import java.util.*;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {

    private TalonFXConfiguration config;
    private TalonFXConfigurationHelper configHelper;

    private final TalonFX motorController;

    public OuttakeSubsystem() {
        config = new TalonFXConfiguration();
        motorController = new TalonFX(29);
        configHelper = new TalonFXConfigurationHelper(config, new File("/home/lvuser/deploy/talonfx-configs.txt"), motorController);
        try {
            configHelper.setConfiguration();
        } catch (Exception e) {

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
