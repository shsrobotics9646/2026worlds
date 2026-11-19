package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.ALIGN_DESIRED_LATERAL_OFFSET_METERS;
import static frc.robot.Constants.VisionConstants.ALIGN_HEADING_TOLERANCE_DEGREES;
import static frc.robot.Constants.VisionConstants.ALIGN_LATERAL_TOLERANCE_METERS;
import static frc.robot.Constants.VisionConstants.APRILTAG_PIPELINE;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_NAME;
import static frc.robot.Constants.VisionConstants.MAX_VISION_TAG_AMBIGUITY;
import static frc.robot.Constants.VisionConstants.MAX_VISION_ANGULAR_RATE_DEGREES_PER_SECOND;
import static frc.robot.Constants.VisionConstants.MAX_VISION_AVG_TAG_DISTANCE_METERS;
import static frc.robot.Constants.VisionConstants.VALID_TAG_IDS;

import java.util.OptionalInt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, APRILTAG_PIPELINE);
        if (VALID_TAG_IDS.length > 0) {
            LimelightHelpers.SetFiducialIDFiltersOverride(LIMELIGHT_NAME, VALID_TAG_IDS);
        }
    }

    @Override
    public void periodic() {
        publishTelemetry();
        addVisionMeasurement();
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(LIMELIGHT_NAME);
    }

    public double getTx() {
        return LimelightHelpers.getTX(LIMELIGHT_NAME);
    }

    public double getLateralOffsetMeters() {
        Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(LIMELIGHT_NAME);
        return targetPose.getY();
    }

    public OptionalInt getBestTagId() {
        double fiducialId = LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
        if (fiducialId < 0) {
            return OptionalInt.empty();
        }

        return OptionalInt.of((int) fiducialId);
    }

    public boolean isAligned() {
        if (!hasTarget()) {
            return false;
        }

        return Math.abs(getLateralOffsetMeters() - ALIGN_DESIRED_LATERAL_OFFSET_METERS) <= ALIGN_LATERAL_TOLERANCE_METERS
            && Math.abs(getTx()) <= ALIGN_HEADING_TOLERANCE_DEGREES;
    }

    private void addVisionMeasurement() {
        double yawDegrees = drivetrain.getState().Pose.getRotation().getDegrees();
        double yawRateDegreesPerSecond = Units.radiansToDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(
            LIMELIGHT_NAME,
            yawDegrees,
            yawRateDegreesPerSecond,
            0,
            0,
            0,
            0
        );

        PoseEstimate poseEstimate = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(LIMELIGHT_NAME)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        if (!shouldUseMeasurement(poseEstimate, yawRateDegreesPerSecond)) {
            return;
        }

        double xyStdDev = poseEstimate.tagCount >= 2 ? 0.3 : 0.7;
        double thetaStdDev = poseEstimate.tagCount >= 2 ? 0.6 : 999999.0;

        drivetrain.addVisionMeasurement(
            poseEstimate.pose,
            poseEstimate.timestampSeconds,
            VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
        );
    }

    private boolean shouldUseMeasurement(PoseEstimate poseEstimate, double yawRateDegreesPerSecond) {
        if (!hasTarget() || poseEstimate.tagCount <= 0) {
            return false;
        }

        if (Math.abs(yawRateDegreesPerSecond) > MAX_VISION_ANGULAR_RATE_DEGREES_PER_SECOND) {
            return false;
        }

        if (poseEstimate.avgTagDist > MAX_VISION_AVG_TAG_DISTANCE_METERS) {
            return false;
        }

        RawFiducial[] fiducials = poseEstimate.rawFiducials;
        if (fiducials.length == 1 && fiducials[0].ambiguity > MAX_VISION_TAG_AMBIGUITY) {
            return false;
        }

        return !MathUtil.isNear(0.0, poseEstimate.pose.getX(), 0.001)
            || !MathUtil.isNear(0.0, poseEstimate.pose.getY(), 0.001);
    }

    private void publishTelemetry() {
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
        SmartDashboard.putNumber("Vision/Tx", getTx());
        SmartDashboard.putNumber("Vision/LateralOffsetMeters", getLateralOffsetMeters());
        SmartDashboard.putNumber("Vision/TargetCount", LimelightHelpers.getTargetCount(LIMELIGHT_NAME));
        SmartDashboard.putBoolean("Vision/Aligned", isAligned());
        SmartDashboard.putNumber("Vision/BestTagId", getBestTagId().orElse(-1));
    }
}
