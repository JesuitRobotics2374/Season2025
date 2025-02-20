// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.seafinder.interfaces;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.seafinder.PathfinderSubsystem;
import frc.robot.seafinder.PathfinderSubsystem.Alignment;

public class PanelSubsystem extends SubsystemBase {

    private final Joystick navControllerA = new Joystick(2);
    private final Joystick navControllerB = new Joystick(3);

    public final PathfinderSubsystem pathfinderSubsystem;

    public PanelSubsystem(PathfinderSubsystem pathfinderSubsystem) {
        this.pathfinderSubsystem = pathfinderSubsystem;
        configureBindings();

    }

    public void configureBindings() {
        new JoystickButton(navControllerA, 1)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(1, Alignment.LEFT)));
        new JoystickButton(navControllerA, 2)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(1, Alignment.RIGHT)));
        new JoystickButton(navControllerA, 3)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(2, Alignment.LEFT)));
        new JoystickButton(navControllerA, 4)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(2, Alignment.RIGHT)));
        new JoystickButton(navControllerA, 5)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(3, Alignment.LEFT)));
        new JoystickButton(navControllerA, 6)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(3, Alignment.RIGHT)));
        new JoystickButton(navControllerA, 7)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(4, Alignment.LEFT)));
        new JoystickButton(navControllerA, 8)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(4, Alignment.RIGHT)));
        new JoystickButton(navControllerA, 9)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(5, Alignment.LEFT)));
        new JoystickButton(navControllerA, 10)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(5, Alignment.RIGHT)));
        new JoystickButton(navControllerA, 11)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(6, Alignment.LEFT)));
        new JoystickButton(navControllerA, 12)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(6, Alignment.RIGHT)));
        new JoystickButton(navControllerA, 17)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(10, Alignment.CENTER)));
        new JoystickButton(navControllerA, 18)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueFind(11, Alignment.CENTER)));
        new JoystickButton(navControllerA, 13)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueAlign(Constants.SETPOINT_REEF_T4)));
        new JoystickButton(navControllerA, 14)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueAlign(Constants.SETPOINT_REEF_T3)));
        new JoystickButton(navControllerA, 15)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueAlign(Constants.SETPOINT_REEF_T2)));
        new JoystickButton(navControllerA, 16)
                .onTrue(new InstantCommand(() -> pathfinderSubsystem.queueAlign(Constants.SETPOINT_REEF_T1)));
    }

}
