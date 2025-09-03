package frc.robot.seafinder2.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Core;
import frc.robot.seafinder2.SF2Constants;
import frc.robot.seafinder2.commands.StaticBackCommand;

public class Target {

    public enum Landmark {
        REEF_FRONT,
        REEF_FRONT_RIGHT,
        REEF_BACK_RIGHT,
        REEF_BACK,
        REEF_BACK_LEFT,
        REEF_FRONT_LEFT,
        STATION_LEFT,
        STATION_RIGHT,
        PROCESSOR,
        BARGE_LEFT,
        BARGE_RIGHT,
    }

    public enum Side {
        LEFT,
        RIGHT,
        CENTER,
    }

    public enum Height {
        TROUGH,
        BRANCH_L2,
        BRANCH_L3,
        BRANCH_L4,
    }

    public static class Location {
        Landmark landmark;
        Side side;
        boolean isReef;

        public Location(Landmark landmark, Side side) {
            this.landmark = landmark;
            this.side = side;
            isReef = true;
        }

        public Location(Landmark landmark) {
            this.landmark = landmark;
            isReef = false;
        }

        public Landmark getLandmark() {
            return landmark;
        }

        public Side getSide() {
            return side;
        }

        public boolean isReef() {
            return isReef;
        }

        @Override
        public String toString() {
            return landmark.toString() + " " + side.toString();
        }
    }

    public static class TagRelativePose {
        int tagId;

        // NWU coordinate system
        double x;
        double y;
        double yaw;

        public TagRelativePose(int tagId, double x, double y, double yaw) {
            this.tagId = tagId;
            this.x = x;
            this.y = y;
            this.yaw = yaw;
        }

        public int getTagId() {
            return tagId;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getYaw() {
            return yaw;
        }

        public Pose2d getPose2d() {
            return new Pose2d(x, y, new Rotation2d(yaw));
        }

        public String toString() {
            return "Tag " + tagId + " at (" + x + ", " + y + ") with yaw " + yaw;
        }
    }

    public static class Setpoint {

        double elevator;
        double arm;
        double wrist;

        public Setpoint(double elevator, double arm, double wrist) {
            this.elevator = elevator;
            this.arm = arm;
            this.wrist = wrist;
        }

        public double getElevator() {
            return elevator;
        }

        public double getArm() {
            return arm;
        }

        public double getWrist() {
            return wrist;
        }

    }

    Core core;

    Location location;
    Height height;

    Setpoint setpoint;
    TagRelativePose tagRelativePose;
    Command retractCommand;

    public Target(Core core) {
        this.core = core;
    }

    public void setLocation(Location location) {
        this.location = location;
        if (isValid()) {
            compute();
        }
    }

    public void setHeight(Height height) {
        this.height = height;
        if (isValid()) {
            compute();
        }
    }

    public void compute() {
        init();
    }

    public boolean isValid() {
        if (location == null) {return false;}
        if (this.location.isReef) {
            return height != null;
        }
        return true;
    }

    public boolean isComputed() {
        return tagRelativePose != null;
    }

    public Location getLocation() {
        return location;
    }

    public Height getHeight() {
        return height;
    }

    public Setpoint getSetpoint() {
        return setpoint;
    }

    public TagRelativePose getTagRelativePose() {
        return tagRelativePose;
    }

    public int getTag() {
        return tagRelativePose.getTagId();
    }

    public Command getRetractCommand() {
        return retractCommand;
    }

    public boolean isReef() {
        return this.location != null && this.location.isReef;
    }

    public String toString() {
        return "Target at " + location + " " + height;
    }

    private void init() {
        final Optional<DriverStation.Alliance> driverStationAlliance = DriverStation.getAlliance();
        if (!driverStationAlliance.isPresent()) {
            throw new IllegalStateException("Driver station alliance not present");
        }
        final boolean isRed = driverStationAlliance.get() == Alliance.Red;

        int tagId = -1;
        double x = 0;
        double y = 0;
        double yaw = 0;

        switch (this.location.landmark) {
            case REEF_FRONT:
                tagId = isRed ? 7 : 18;
                break;
            case REEF_FRONT_RIGHT:
                tagId = isRed ? 8 : 17;
                break;
            case REEF_BACK_RIGHT:
                tagId = isRed ? 9 : 22;
                break;
            case REEF_BACK:
                tagId = isRed ? 10 : 21;
                break;
            case REEF_BACK_LEFT:
                tagId = isRed ? 11 : 20;
                break;
            case REEF_FRONT_LEFT:
                tagId = isRed ? 6 : 19;
                break;
            case STATION_LEFT:
                setpoint = SF2Constants.SETPOINT_HP_INTAKE;
                tagId = isRed ? 1 : 13;
                break;
            case STATION_RIGHT:
                setpoint = SF2Constants.SETPOINT_BARGE;
                tagId = isRed ? 2 : 12;
                break;
            case PROCESSOR:
                setpoint = SF2Constants.SETPOINT_PROCESSOR;
                tagId = isRed ? 3 : 16;
                break;
            case BARGE_LEFT:
                setpoint = SF2Constants.SETPOINT_BARGE;
                tagId = isRed ? 5 : 14;
                break;
            case BARGE_RIGHT:
                setpoint = SF2Constants.SETPOINT_BARGE;
                tagId = isRed ? 4 : 15;
                break;
        }

        double extraFrontBuffer = 0;
        boolean isTrough = false;

        System.out.println("ISREEF: " + this.location.isReef);
        if (this.location.isReef) {
            switch (this.height) {
                case TROUGH:
                    setpoint = SF2Constants.SETPOINT_REEF_T1;
                    retractCommand = (new StaticBackCommand(core.getDrivetrain(), -0.4, -1)).withTimeout(1.5);
                    isTrough = true;
                    break;
                case BRANCH_L2:
                    setpoint = SF2Constants.SETPOINT_REEF_T2;
                    //retractCommand = new RetractL2(core);
                    break;
                case BRANCH_L3:
                    setpoint = SF2Constants.SETPOINT_REEF_T3;
                    //retractCommand = new RetractL3(core);
                    break;
                case BRANCH_L4:
                    setpoint = SF2Constants.SETPOINT_REEF_T4;
                    //retractCommand = new RetractL4(core);
                    break;
            }
            switch (this.location.side) {
                
                case LEFT:
                    y = SF2Constants.SEAFINDER2_REEF_FRONT_PADDING;
                    x = SF2Constants.SEAFINDER2_REEF_LEFT_BRANCH_OFFSET;
                    break;
                case RIGHT:
                    y = SF2Constants.SEAFINDER2_REEF_FRONT_PADDING;
                    x = SF2Constants.SEAFINDER2_REEF_RIGHT_BRANCH_OFFSET;
                    break;
                case CENTER:
                    y = SF2Constants.SEAFINDER2_REEF_FRONT_PADDING;
                    x = (SF2Constants.SEAFINDER2_REEF_LEFT_BRANCH_OFFSET
                            + SF2Constants.SEAFINDER2_REEF_RIGHT_BRANCH_OFFSET) / 2;
                    break;
            }
            if (isTrough) {
                y = SF2Constants.SEAFINDER2_REEF_FRONT_PADDING + (isTrough ? -0.25 : 0);
                x = (SF2Constants.SEAFINDER2_REEF_LEFT_BRANCH_OFFSET
                            + SF2Constants.SEAFINDER2_REEF_RIGHT_BRANCH_OFFSET) / 2;
            }
        } else {
            setpoint = SF2Constants.SETPOINT_HP_INTAKE;
            retractCommand = (new StaticBackCommand(core.getDrivetrain(), -0.4, -1)).withTimeout(1.5);
        }

        this.tagRelativePose = new TagRelativePose(tagId, x, y, yaw);

    }

}
