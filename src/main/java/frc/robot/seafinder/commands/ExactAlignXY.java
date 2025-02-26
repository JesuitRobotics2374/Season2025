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

public class ExactAlignXY extends Command {

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d targetPose;

    private double alignmentShift;

    private boolean doneMoving;

    private int tag_id;

    private int clock = 0;

    public ExactAlignXY(CommandSwerveDrivetrain drivetrain, int tag_id) {
        this.drivetrain = drivetrain;

        this.alignmentShift = 0.0;

        this.tag_id = tag_id;
    }

    public ExactAlignXY(CommandSwerveDrivetrain drivetrain, int tag_id, double alignmentShift) {
        this.drivetrain = drivetrain;

        this.alignmentShift = alignmentShift;

        this.tag_id = tag_id;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("Exact XY init");
        doneMoving = false;
    }

    @Override
    public void execute() {
        int canSeeCount = 0;
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
            return;
        }

        for (int i = 0; i < 6; i++) {
            total[i] /= canSeeCount;
        }
        total[0] += alignmentShift;
        total[1] += Constants.ALIGN_Y_SHIFT;

        targetPose = new Pose3d(total[0], total[1], total[2], new Rotation3d(total[5] * Math.PI / 180, total[3] * Math.PI / 180, total[4] * Math.PI / 180)).toPose2d();

        double distanceToTarget = (new Translation2d(0, 0)).getDistance(targetPose.getTranslation());
        if (distanceToTarget < Constants.GENERIC_DISTANCE_THRESHOLD) {
            doneMoving = true;
        }

        double velocityX = 0;
        double velocityY = 0;
        double magnitude = Math.sqrt(Math.pow(total[0], 2) + Math.pow(total[1], 2)); // TODO: Retune after removing
                                     

        if (!doneMoving) {
            velocityX = total[0] / (magnitude + 1e-6) * Constants.ALIGN_MOVE_SPEED;
            velocityY = total[1] / (magnitude + 1e-6) * Constants.ALIGN_MOVE_SPEED;
        }

        System.out.println(" 5: " + total[5] + " 3: " + total[3] + " 4: " + total[4]);

        // System.out.println((new Translation2d(0,
        // 0)).getDistance(targetPose.getTranslation()));
        // System.out.println("DIS" + distanceToTarget + " VELX: " + velocityX + " VELY:
        // " + velocityY + " ROT: " + rotationalRate);

        // + Y is Forard on dr
        if (!(doneMoving)) {
            drivetrain.setControl(driveRequest.withVelocityX(velocityY).withVelocityY(-velocityX));
        }
    }

    @Override
    public boolean isFinished() {
        // System.out.println("--" + doneMoving + " " + doneRotating + "--");
        return doneMoving;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Exact XY ended" + interrupted);
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}