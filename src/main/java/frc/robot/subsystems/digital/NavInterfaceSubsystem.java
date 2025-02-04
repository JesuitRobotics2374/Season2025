// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.digital;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.PathfindCommand;
import frc.robot.commands.PathfindCommand.Alignment;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class NavInterfaceSubsystem extends SubsystemBase {

    private final NetworkTableEntry teleopActionEntry;
    private final NetworkTableEntry autoActionEntry;
    private int lastTag = -1;

    private ArrayList<Command> autonomousRoutine = new ArrayList<>();

    private CommandSwerveDrivetrain drivetrain;

    public NavInterfaceSubsystem(CommandSwerveDrivetrain drivetrain) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable navTable = inst.getTable("NavGUI");
        teleopActionEntry = navTable.getEntry("TeleopAction");
        autoActionEntry = navTable.getEntry("AutoScheme");
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        int teleActionValue = (int) teleopActionEntry.getInteger(-1);
        if (teleActionValue != -1) {
            DecodedData decodedData = decodeBitmask(teleActionValue);
            if (lastTag != decodedData.tagID) {
                System.out.println(decodedData.tagID);
                lastTag = decodedData.tagID;
                Alignment alignment = Alignment.parseTopology(decodedData.isReef, decodedData.reefLoc);
                System.out.println("Sending pathfind command: " + lastTag + " " + alignment + " " + decodedData);
                (new PathfindCommand(drivetrain, lastTag, alignment)).schedule();
                return;
            }
        }
        long[] autoActionValue = autoActionEntry.getIntegerArray((long[]) null);
        if (autoActionValue != null && autoActionValue.length > 0 && autoActionValue[0] != -1) {
            parseAutonomousRoutine(autoActionValue);
        }
    }

    private void parseAutonomousRoutine(long[] autoActionValue) {
        for (int i = 0; i < autoActionValue.length; i++) {
            DecodedData decodedData = decodeBitmask(autoActionValue[i]);
            switch (decodedData.instrSet) {
                case 5: // Pathfind
                    Alignment alignment = Alignment.parseTopology(decodedData.isReef, decodedData.reefLoc);
                    autonomousRoutine.add(new PathfindCommand(drivetrain, decodedData.tagID, alignment));
                    break;
            }
        }
    }

    public ArrayList<Command> getAutonomousRoutine() {
        return autonomousRoutine;
    }

    public void wipeAutonomousRoutine() {
        autonomousRoutine.clear();
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
    public static DecodedData decodeBitmask(long bitmask) {
        // Decode each field using bitwise operations
        int tagID = (int) (bitmask >> 8) & 0x1F; // First 5 bits (positions 8–12)
        boolean isReef = ((bitmask >> 7) & 0x1) == 1; // Next bit (position 7)
        int reefLoc = (int) (bitmask >> 4) & 0x7; // Next 3 bits (positions 4–6)
        boolean isRed = ((bitmask >> 3) & 0x1) == 1; // Next bit (position 3)
        int instrSet = (int) (bitmask >> 0) & 0x7; // Next 3 bits (positions 0–2)
        int instrUnit = (int) (bitmask >> 0) & 0x7; // Last 3 bits (positions 0–2)

        return new DecodedData(tagID, isReef, reefLoc, isRed, instrSet, instrUnit);
    }
}
