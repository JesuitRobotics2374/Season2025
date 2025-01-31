// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.digital;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.teleop.OrganizePathfind;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class NavInterfaceSubsystem extends SubsystemBase {

    private final NetworkTableEntry teleopActionEntry;
    private int lastTag = -1;

    private CommandSwerveDrivetrain drivetrain;

    public NavInterfaceSubsystem(CommandSwerveDrivetrain drivetrain) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable navTable = inst.getTable("NavGUI");
        teleopActionEntry = navTable.getEntry("TeleopAction");
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        int actionValue = (int) teleopActionEntry.getInteger(-1);
        if (actionValue != -1) {
            DecodedData decodedData = decodeBitmask(actionValue);
            if (lastTag != decodedData.tagID) {
                System.out.println(decodedData.tagID);
                lastTag = decodedData.tagID;
                (new OrganizePathfind(drivetrain, lastTag)).schedule();
            }
        }
    }

    // Class to store the decoded data
    public static class DecodedData {
        int tagID; // 5-bit integer
        boolean isReef; // Boolean
        int reefLoc; // 3-bit integer
        boolean isRed; // Boolean
        int instrSet; // 3-bit integer
        int instrUnit; // 3-bit integer

        // Constructor for easy initialization
        public DecodedData(int tagID, boolean isReef, int reefLoc, boolean isRed, int instrSet, int instrUnit) {
            this.tagID = tagID;
            this.isReef = isReef;
            this.reefLoc = reefLoc;
            this.isRed = isRed;
            this.instrSet = instrSet;
            this.instrUnit = instrUnit;
        }

        @Override
        public String toString() {
            return "DecodedData{" +
                    "tagID=" + tagID +
                    ", isReef=" + isReef +
                    ", reefLoc=" + reefLoc +
                    ", isRed=" + isRed +
                    ", instrSet=" + instrSet +
                    ", instrUnit=" + instrUnit +
                    '}';
        }
    }

    // Method to decode the bitmask
    public static DecodedData decodeBitmask(int bitmask) {
        // Decode each field using bitwise operations
        int tagID = (bitmask >> 8) & 0x1F; // First 5 bits (positions 8–12)
        boolean isReef = ((bitmask >> 7) & 0x1) == 1; // Next bit (position 7)
        int reefLoc = (bitmask >> 4) & 0x7; // Next 3 bits (positions 4–6)
        boolean isRed = ((bitmask >> 3) & 0x1) == 1; // Next bit (position 3)
        int instrSet = (bitmask >> 0) & 0x7; // Next 3 bits (positions 0–2)
        int instrUnit = (bitmask >> 0) & 0x7; // Last 3 bits (positions 0–2)

        return new DecodedData(tagID, isReef, reefLoc, isRed, instrSet, instrUnit);
    }
}
