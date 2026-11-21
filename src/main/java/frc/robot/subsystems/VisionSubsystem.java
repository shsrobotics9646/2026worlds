package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final Timer timer = new Timer();
    private double lastTimeTargetSeen = 0.0;
    private final String limelightName;
    private String sdKey;

    private boolean seesTag30 = false;
    private double tag30DistanceMeters = -1.0;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        timer.start();
        this.sdKey = "VisionDebug/LastSeen";
        SmartDashboard.putNumber(this.sdKey, -1);
    }

    @Override
    public void periodic() {
        LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        // if (est != null && est.tagCount > 0) {
        //     System.out.println("X,Y,Heading=" + est.pose.getX() + "," + est.pose.getY() + "," + est.pose.getRotation().getDegrees());
        // }
        // if (LimelightHelpers.getTV(limelightName)) {
        //     lastTimeTargetSeen = timer.get();
        // }
        // SmartDashboard.putNumber(this.sdKey, lastTimeTargetSeen);

        seesTag30 = false;
        tag30DistanceMeters = -1.0;

        if (LimelightHelpers.getTV(limelightName)) {
            LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
            for (LimelightHelpers.RawFiducial fiducial : fiducials) {
                if (fiducial.id == 30) {
                    seesTag30 = true;
                    tag30DistanceMeters = fiducial.distToCamera;
                    break;
                }
            }
        }
        // System.out.println("VisionSubsystem seesTag30=" + seesTag30 + ", tag30DistanceMeters=" + tag30DistanceMeters);
    }

    public boolean seesTag30() {
        return seesTag30;
    }

    public double getTag30DistanceMeters() {
        return tag30DistanceMeters;
    }
}
