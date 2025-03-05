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

    private int[][] decodeRaw(long[] raw) {
        // Split each number where 4 bits (A) are the point and 4 bits (B) are the alignment: AAAABBBB
        int[][] decoded = new int[raw.length][2];
        for (int i = 0; i < raw.length; i++) {
            decoded[i][0] = (int) (raw[i] >> 4);
            decoded[i][1] = (int) (raw[i] & 0b1111);
        }
        return decoded;
    }

    public NavInterfaceSubsystem() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        navTable = inst.getTable("NavGUI");
        sendReadData(0);
    }

    public void sendReadData(int data) {
        navTable.getEntry("Embedded").setInteger(data);
    }

    public int[][] loadPathData() {

        NetworkTableEntry pathEntry = navTable.getEntry("kAutoSequence");
        // long[] pathDataRaw = pathEntry.getIntegerArray(new long[0]);
        long[] pathDataRaw = {72};
        int[][] pathData = decodeRaw(pathDataRaw);
        return pathData;
    }
    
}
