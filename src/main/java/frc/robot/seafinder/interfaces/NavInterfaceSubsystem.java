// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.seafinder.interfaces;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.seafinder.PathfinderSubsystem.Alignment;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class NavInterfaceSubsystem extends SubsystemBase {

    private final NetworkTable navTable;

    public NavInterfaceSubsystem() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        navTable = inst.getTable("NavGUI");
        sendReadData(0);
    }

    public void sendReadData(int data) {
        navTable.getEntry("Embedded").setInteger(data);
    }
    
}
