// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.seafinder2.interfaces;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.seafinder2.PathfinderSubsystem;
import frc.robot.seafinder2.utils.Target.Height;
import frc.robot.seafinder2.utils.Target.Landmark;
import frc.robot.seafinder2.utils.Target.Location;
import frc.robot.seafinder2.utils.Target.Side;

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
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.REEF_FRONT, Side.LEFT))));
                new JoystickButton(navControllerA, 2)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.REEF_FRONT, Side.RIGHT))));
                new JoystickButton(navControllerA, 3)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.REEF_FRONT_RIGHT, Side.LEFT))));
                new JoystickButton(navControllerA, 4)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.REEF_FRONT_RIGHT, Side.RIGHT))));
                new JoystickButton(navControllerA, 5)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.REEF_BACK_RIGHT, Side.LEFT))));
                new JoystickButton(navControllerA, 6)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.REEF_BACK_RIGHT, Side.RIGHT))));
                new JoystickButton(navControllerA, 7)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.REEF_BACK, Side.LEFT))));
                new JoystickButton(navControllerA, 8)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.REEF_BACK, Side.RIGHT))));
                new JoystickButton(navControllerA, 9)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.REEF_BACK_LEFT, Side.LEFT))));
                new JoystickButton(navControllerA, 10)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.REEF_BACK_LEFT, Side.RIGHT))));
                new JoystickButton(navControllerA, 11)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.REEF_FRONT_LEFT, Side.LEFT))));
                new JoystickButton(navControllerA, 12)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.REEF_FRONT_LEFT, Side.RIGHT))));
                new JoystickButton(navControllerB, 2)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.STATION_RIGHT))));
                new JoystickButton(navControllerA, 18)
                                .onTrue(new InstantCommand(() -> pathfinderSubsystem
                                                .queueFind(new Location(Landmark.STATION_LEFT))));
                new JoystickButton(navControllerA, 13)
                                .onTrue(new InstantCommand(
                                                () -> pathfinderSubsystem.queueAlign(Height.TROUGH)));
                new JoystickButton(navControllerA, 14)
                                .onTrue(new InstantCommand(
                                                () -> pathfinderSubsystem.queueAlign(Height.BRANCH_L2)));
                new JoystickButton(navControllerA, 15)
                                .onTrue(new InstantCommand(
                                                () -> pathfinderSubsystem.queueAlign(Height.BRANCH_L3)));
                new JoystickButton(navControllerA, 16)
                                .onTrue(new InstantCommand(
                                                () -> pathfinderSubsystem.queueAlign(Height.BRANCH_L4)));
        }

}
