package frc.robot.seafinder.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightObject;

public class ExactAlignRot extends Command {

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d targetPose;

    private double alignmentShift;

    private boolean doneMoving;
    private boolean doneRotating;

    private int tag_id;

    private int clock = 0;

    public ExactAlignRot(CommandSwerveDrivetrain drivetrain, int tag_id) {
        this.drivetrain = drivetrain;
        
        this.alignmentShift = 0.0;

        this.tag_id = tag_id;
    }

    public ExactAlignRot(CommandSwerveDrivetrain drivetrain, int tag_id, double alignmentShift) {
        this.drivetrain = drivetrain;

        this.alignmentShift = alignmentShift;

        this.tag_id = tag_id;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        doneMoving = false;
        doneRotating = false;
    }

    @Override
    public void execute() {        int canSeeCount = 0;
        double[] total = new double[6];

        for (LimelightObject ll : Constants.LIMELIGHTS_ON_BOARD) {
            if (NetworkTableInstance.getDefault().getTable(ll.name).getEntry("tid").getDouble(-1) == tag_id) {
                canSeeCount++;
                double[] raw = NetworkTableInstance.getDefault().getTable(ll.name).getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
                for (int i = 0; i < 6; i++) {
                    total[i] += raw[i];
                }
            }
        }

        if (canSeeCount == 0) {
            doneMoving = true;
            doneRotating = true;
            return;
        }

        for (int i = 0; i < 6; i++) {
            total[i] /= canSeeCount;
        }

        targetPose = new Pose3d(total[0], total[1], total[2], new Rotation3d(total[5] * Math.PI / 180, total[3] * Math.PI / 180, total[4] * Math.PI / 180)).toPose2d();

        double rotationToTarget = Math.abs(targetPose.getRotation().getRadians());
        if (rotationToTarget < Constants.GENERIC_ROTATION_THRESHOLD) {doneRotating = true;}

        double rotationalRate = 0;

        if (!doneRotating) {
            double rotationError = targetPose.getRotation().getRadians();
            double RESign = rotationError / Math.abs(rotationError);
            rotationalRate = rotationError * Constants.ALIGN_ROTATE_SPEED
            + (RESign * Constants.ALIGN_ROTATIONAL_FEED_FORWARD);
        }

        System.out.println("ROT: " + rotationToTarget);
        System.out.println("THRESH: " + Constants.GENERIC_ROTATION_THRESHOLD);

        // System.out.println(" 5: " + raw[5] + " 3: " + raw[3] + " 4: " + raw[4]);


        // System.out.println((new Translation2d(0, 0)).getDistance(targetPose.getTranslation()));
        // System.out.println("DIS" + distanceToTarget + " VELX: " + velocityX + " VELY: " + velocityY + " ROT: " + rotationalRate);

        // + Y is Forard on dr 
        if (!doneRotating) {
            drivetrain.setControl(driveRequest.withRotationalRate(-rotationalRate));
        }
    }

    @Override
    public boolean isFinished() {
        // System.out.println("--" + doneMoving + " " + doneRotating + "--");
        return doneRotating;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ExactAlign ended" + interrupted);
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}