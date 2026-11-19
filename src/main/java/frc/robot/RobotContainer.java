// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static frc.robot.Constants.FuelConstants.INDEXER_INTAKING_PERCENT;
import static frc.robot.Constants.FuelConstants.INDEXER_LAUNCHING_PERCENT;
import static frc.robot.Constants.FuelConstants.INTAKE_INTAKING_PERCENT;
import static frc.robot.Constants.FuelConstants.LAUNCHING_LAUNCHER_PERCENT;
import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.BlockShoot;
import frc.robot.commands.Eject;
import frc.robot.commands.IndexEject;
import frc.robot.commands.Intake;
import frc.robot.commands.Launch;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.SpinUpMAX;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KrakenFuelSubSystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = .6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final KrakenFuelSubSystem fuelSubsystem = new KrakenFuelSubSystem();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain);

    private final SendableChooser<Command> autoChooser;

    private Command driveAuto = Commands.deadline(Commands.run(()-> drive.withVelocityX(0.5)), Commands.waitSeconds(10));

    public RobotContainer() {
        DriverStation.isJoystickConnectionWarningSilenced();
        // Auto Builder

        // Registering


        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Drive Auto", driveAuto);
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (le_ft)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driveController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driveController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driveController.rightBumper().whileTrue(
            new AlignToAprilTag(
                drivetrain,
                visionSubsystem,
                driveController::getLeftY,
                MaxSpeed
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);
        // Operator Controls

        /* OPERATOR CONTROL TEMPORARILY OFF UNTIL CONFIRMED MOTOR WONT GO BOOM
        operatorController.leftTrigger().whileTrue(new IndexEject(fuelSubsystem));
        operatorController.leftBumper().whileTrue(new Launch(fuelSubsystem));
        operatorController.rightBumper().whileTrue(new Eject(fuelSubsystem));
        operatorController.rightTrigger().whileTrue(new Intake(fuelSubsystem));
        operatorController.a().whileTrue(new SpinUpMAX(fuelSubsystem));
        operatorController.y().whileTrue(new BlockShoot(fuelSubsystem));
        */

        // Resets The Robot Command After Done Runner
        fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stopAll()));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
