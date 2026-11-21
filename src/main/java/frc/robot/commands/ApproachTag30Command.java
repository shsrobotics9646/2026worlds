package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class ApproachTag30Command extends Command {

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private static final double TARGET_DISTANCE_METERS = Units.feetToMeters(5.0);
    private final PIDController distancePid = new PIDController(0.5, 0.0, 0.0);

    public ApproachTag30Command(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    distancePid.reset();
    distancePid.setSetpoint(TARGET_DISTANCE_METERS);
    }

    @Override
    public void execute() {
        if (!vision.seesTag30()) {
        drivetrain.setControl(
            driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)
        );
        return;
        }
        double currentDistanceMeters = vision.getTag30DistanceMeters();
        double rawOutput = distancePid.calculate(currentDistanceMeters);
        double clampedSpeed = Math.max(-0.5, Math.min(0.5, rawOutput));
        drivetrain.setControl(
            driveRequest.withVelocityX(clampedSpeed).withVelocityY(0.0).withRotationalRate(0.0)
        );
    }

    @Override
    public void end(boolean interrupted) {
    drivetrain.setControl(
        driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)
    );
}

    @Override
    public boolean isFinished() {
        return false;
    }
}