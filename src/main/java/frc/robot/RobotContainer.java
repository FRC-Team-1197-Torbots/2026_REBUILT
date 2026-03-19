// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.ZoneDetection;
import frc.robot.subsystems.Shooter.SHOOTER_SIDE;
import frc.robot.subsystems.Turret.TURRET_SIDE;

public class RobotContainer {
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
                                                                                            // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final CommandXboxController driverController = new CommandXboxController(0);

        
        // private final CommandXboxController overrideController = new
        // CommandXboxController(1);
        /***************************
         * CORE SUBSYSTEMS
         ******************************/
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final ZoneDetection m_zoneDetection = new ZoneDetection(drivetrain, drivetrain.getPigeon2());

        /***************************
         * INTAKE & INDEXING
         ******************************/
        private final Intake m_intake = new Intake();
        private final Hopper m_hopper = new Hopper(m_intake);

        /***************************
         * SHOOTER & TARGETING
         ******************************/
        private final Shooter leftShooter = new Shooter(Constants.ShooterConstants.ShooterCanId1,
                        Constants.ShooterConstants.ShooterCanId2, SHOOTER_SIDE.LEFT);
        private final Shooter rightShooter = new Shooter(Constants.ShooterConstants.ShooterCanId3,
                        Constants.ShooterConstants.ShooterCanId4, SHOOTER_SIDE.RIGHT);

        // Turrets for testing
        private final Turret leftTurret = new Turret(Constants.TurretConstants.TurretCanId2,
                        Constants.TurretConstants.encoderCanID1, Constants.TurretConstants.TurretOffset2,
                        drivetrain, m_zoneDetection, TURRET_SIDE.LEFT, m_intake);
        private final Turret rightTurret = new Turret(Constants.TurretConstants.TurretCanId1,
                        Constants.TurretConstants.encoderCanID2, Constants.TurretConstants.TurretOffset1,
                        drivetrain, m_zoneDetection, TURRET_SIDE.RIGHT, m_intake);

        private final Hood rightHood = new Hood(Constants.HoodConstants.HoodCanId1, Hood.HOOD_SIDE.RIGHT);
        private final Hood leftHood = new Hood(Constants.HoodConstants.HoodCanId2, Hood.HOOD_SIDE.LEFT);

        private final frc.robot.subsystems.AimingManager m_aimingManager = new
        frc.robot.subsystems.AimingManager(drivetrain, m_zoneDetection, leftTurret, rightTurret, leftShooter, rightShooter);

        private final SendableChooser<Command> autoChooser;

        Command shootGroup = new ParallelDeadlineGroup(
                                Commands.waitUntil(() -> leftShooter.isAtSpeed() && rightShooter.isAtSpeed()).withTimeout(1)
                                                .andThen(m_hopper.runShootCommand()),
                                leftShooter.runShooterCommand(),
                                rightShooter.runShooterCommand());

        public RobotContainer() {
                configureNamedCommands();
                configureBindings();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto selection", autoChooser);
        }

        private void configureNamedCommands() {
                NamedCommands.registerCommand("intake on", m_intake.runDeployImmediate(() -> drivetrain.getState().Speeds));
                NamedCommands.registerCommand("run intake", m_intake.runIntakeWheelAuto(() -> drivetrain.getState().Speeds));
                NamedCommands.registerCommand("shoot balls", shootGroup);
        }

        private void configureBindings() {
                drivetrain.registerTelemetry(logger::telemeterize);

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive
                                                                                                        // forward
                                                                                                        // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left
                                                                                                        // with negative
                                                                                                        // X (left)
                                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive
                                                                                                                    // counterclockwise
                                                                                                                    // with
                                // negative X (left)
                                ));

                // ******************** Default Commands *****************************/
                // Shooter idle commands (using the new closed-loop target RPM)
                leftShooter.setDefaultCommand(leftShooter.run(() -> leftShooter.runIdle()));
                rightShooter.setDefaultCommand(rightShooter.run(() -> rightShooter.runIdle()));

                // ********************WORKING FUNCTIONS *****************************/
                // Reset the field-centric heading on start button press (right middle button)
                driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                // Click to drop intake
                driverController.rightBumper()
                                .onTrue(m_intake.runDeployAndIntakeCommand(() -> drivetrain.getState().Speeds));

                // // Click to retract intake
                driverController.leftBumper().onTrue(m_intake.runRetractCommand());

                driverController.rightTrigger(0.5f).whileTrue(shootGroup);

                //////////////////////Testing functions//////////////////////////
                driverController.a().whileTrue(m_intake.runAgiCommand())
                        .onFalse(m_intake.runDeployAndIntakeCommand(() -> drivetrain.getState().Speeds));
                

                // ******************** OVERRIDES *****************************/
                // Manual encoder resets without touching PID voltages
                // overrideController.povUp().onTrue(m_intake.forceRetract());
                // overrideController.povDown().onTrue(m_intake.forceDeploy());

                // // Allow the co-driver to auto-home manually or in test mode
                // overrideController.start().onTrue(m_intake.autoHome());
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

}
