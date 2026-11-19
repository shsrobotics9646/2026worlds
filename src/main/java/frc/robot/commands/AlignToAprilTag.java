package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.ALIGN_DESIRED_LATERAL_OFFSET_METERS;
import static frc.robot.Constants.VisionConstants.ALIGN_MAX_ROTATION_RADIANS_PER_SECOND;
import static frc.robot.Constants.VisionConstants.ALIGN_MAX_STRAFE_METERS_PER_SECOND;
import static frc.robot.Constants.VisionConstants.ALIGN_ROTATION_KP;
import static frc.robot.Constants.VisionConstants.ALIGN_STRAFE_KP;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToAprilTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final DoubleSupplier forwardInput;
    private final double maxSpeedMetersPerSecond;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AlignToAprilTag(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsystem visionSubsystem,
        DoubleSupplier forwardInput,
        double maxSpeedMetersPerSecond
    ) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.forwardInput = forwardInput;
        this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double velocityX = -forwardInput.getAsDouble() * maxSpeedMetersPerSecond;
        double velocityY = 0.0;
        double rotationalRate = 0.0;

        if (visionSubsystem.hasTarget()) {
            double strafeError = visionSubsystem.getLateralOffsetMeters() - ALIGN_DESIRED_LATERAL_OFFSET_METERS;

            velocityY = MathUtil.clamp(
                -strafeError * ALIGN_STRAFE_KP,
                -ALIGN_MAX_STRAFE_METERS_PER_SECOND,
                ALIGN_MAX_STRAFE_METERS_PER_SECOND
            );

            rotationalRate = MathUtil.clamp(
                -Math.toRadians(visionSubsystem.getTx()) * ALIGN_ROTATION_KP,
                -ALIGN_MAX_ROTATION_RADIANS_PER_SECOND,
                ALIGN_MAX_ROTATION_RADIANS_PER_SECOND
            );
        }

        drivetrain.setControl(
            driveRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationalRate)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
