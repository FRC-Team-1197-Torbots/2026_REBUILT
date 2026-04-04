// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.Commands.*;

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

        
        // private final CommandXboxController overrideController = new CommandXboxController(1);
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

        Command shootGroup = new ShootCommand(leftShooter, rightShooter, m_hopper, m_zoneDetection);
        Command shootTimeout4 = new ShootCommand(leftShooter, rightShooter, m_hopper, m_zoneDetection).withTimeout(4);
        Command shootTimeout2 = new ShootCommand(leftShooter, rightShooter, m_hopper, m_zoneDetection).withTimeout(2);

        public RobotContainer() {
                configureNamedCommands();
                configureBindings();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto selection", autoChooser);
        }

        private void configureNamedCommands() {
                NamedCommands.registerCommand("intake on", m_intake.runDeployImmediate(() -> drivetrain.getState().Speeds));
                
                // Build the timeouts clearly in the Container to keep the subsystem clean
                NamedCommands.registerCommand("run intake 1", m_intake.runDeployAndIntakeCommand(() -> drivetrain.getState().Speeds).withTimeout(4.7));
                NamedCommands.registerCommand("run intake 2", m_intake.runDeployAndIntakeCommand(() -> drivetrain.getState().Speeds).withTimeout(6.0));
                NamedCommands.registerCommand("run intake", m_intake.runDeployAndIntakeCommand(() -> drivetrain.getState().Speeds));
                
                NamedCommands.registerCommand("shoot balls", shootGroup);
                NamedCommands.registerCommand("shoot balls 4", shootTimeout4);
        }

        private void configureBindings() {
                drivetrain.registerTelemetry(logger::telemeterize);

                // Note that WPILib convention: X is forward, Y is left.
                // Negative controller Y = drive forward (X axis).
                // Negative controller X = drive left (Y axis).
                // Negative right X = rotate counterclockwise.
                drivetrain.setDefaultCommand(
                        drivetrain.applyRequest(() -> drive
                                .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                                .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
                        )
                );

                // ******************** Default Commands *****************************/
                // Shooter idle commands (using the new closed-loop target RPM)
                leftShooter.setDefaultCommand(leftShooter.run(() -> leftShooter.runIdle()));
                rightShooter.setDefaultCommand(rightShooter.run(() -> rightShooter.runIdle()));

                ParallelCommandGroup resetcommand = new ParallelCommandGroup(drivetrain.runOnce(drivetrain::seedFieldCentric),
                        Commands.runOnce(()->rightTurret.zeroTurret()), Commands.runOnce(()->leftTurret.zeroTurret()));

                // ********************WORKING FUNCTIONS *****************************/
                // Reset the field-centric heading on start button press (right middle button)
                driverController.start().onTrue((resetcommand));

                // Click to drop intake
                driverController.rightBumper()
                                .onTrue(m_intake.runDeployAndIntakeCommand(() -> drivetrain.getState().Speeds));


                // Click to retract intake safely
                driverController.leftBumper().onTrue(safeRetractCommand());

                driverController.rightTrigger(0.5f).whileTrue(shootGroup);

                // driverController.y().onTrue(Commands.runOnce(()->m_aimingManager.toggleShootOnTheMove()));

                //////////////////////Co Pilot functions//////////////////////////
                overrideController.a().whileTrue(m_intake.runAgiCommand())
                        .onFalse(m_intake.runDeployAndIntakeCommand(() -> drivetrain.getState().Speeds));

                overrideController.rightTrigger(0.5).whileTrue(m_hopper.reverseHopper()).onFalse(m_hopper.stopCommand());
        }

        private Command safeRetractCommand() {
                return Commands.runOnce(() -> {
                        m_intake.stopIntake();
                        m_intake.m_position = Intake.INTAKE_POSITION.RETRACTING;
                }).andThen(Commands.waitUntil(() -> leftTurret.isStraight() && rightTurret.isStraight()))
                  .andThen(m_intake.runRetractCommand());
        }

        public Command getAutonomousCommand() {
                m_zoneDetection.enableZoneDetection(false); 
                return autoChooser.getSelected();
        }

        public void teleInit() {
                m_zoneDetection.teleinit();
                m_zoneDetection.enableZoneDetection(true);
        }

}
