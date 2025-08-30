// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Effector.WheelState;
import frc.robot.subsystems.Superstructure.WantedState;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Subsystems
    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision vision = new Vision(drivetrain);
    public final Effector effector = new Effector();
    public final Elevator elevator = new Elevator();
    public final Intake intake = new Intake();
    public final Superstructure superstructure = new Superstructure(drivetrain, effector, elevator, intake, vision);

    public final CommandXboxController driverController = Constants.OperatorConstants.driverController.getController();
    public final CommandXboxController operatorController = Constants.OperatorConstants.operatorController.getController();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        drivetrain.provideSubsystemAccessToSuperstructure(superstructure);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drive and steer stuff
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-MathUtil.applyDeadband(driverController.getLeftY(), 0.1) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(driverController.getLeftX(), 0.1) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(driverController.getRightX(), 0.1) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.leftTrigger().onTrue(superstructure.setState(WantedState.CORAL_GROUND_PICKUP)).onFalse(superstructure.setState(WantedState.IDLE));
        // Right Trigger Will Automatically Track Coral

        driverController.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.setTarget(drivetrain.getLeftReefBranch())));
        driverController.leftBumper().whileTrue(drivetrain.alignCommand);
        driverController.rightBumper().onTrue(Commands.runOnce(() -> drivetrain.setTarget(drivetrain.getRightReefBranch())));
        driverController.rightBumper().whileTrue(drivetrain.alignCommand);

        // reset the field-centric heading on left bumper press
        driverController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // y will be whatever climb system we have

        operatorController.a().onTrue(effector.setWheelState(WheelState.ALGAE_INTAKE)).onFalse(effector.setWheelState(WheelState.IDLE));
        operatorController.b().onTrue(effector.setWheelState(WheelState.EJECT)).onFalse(effector.setWheelState(WheelState.IDLE));
        operatorController.x().onTrue(superstructure.setState(WantedState.IDLE));
        operatorController.y().onTrue(Commands.runOnce(() -> superstructure.toggleMode()));

        operatorController.povUp().onTrue(superstructure.setState(WantedState.L3));
        operatorController.povDown().onTrue(superstructure.setState(WantedState.L1));
        operatorController.povLeft().onTrue(superstructure.setState(WantedState.L2));
        operatorController.povRight().onTrue(superstructure.setState(WantedState.L4));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
