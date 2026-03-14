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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.ZoneDetection;
import frc.robot.subsystems.Turret.TURRENT_SIDE;
import frc.robot.Commands.ShootWhileMoving;

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

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final CommandXboxController driverController = new CommandXboxController(0);
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        // Use the drivetrain's Pigeon2 for ZoneDetection
        private final ZoneDetection m_zoneDetection = new ZoneDetection(drivetrain, drivetrain.getPigeon2());

        /***************************
         * TORBOTS SPECIFIC VARIABLES
         ******************************/
        private final Intake m_intake = new Intake();
        private final Hopper m_hopper = new Hopper(driverController);
        private final Shooter leftShooter = new Shooter(Constants.ShooterConstants.ShooterCanId1,
                        Constants.ShooterConstants.ShooterCanId2, m_hopper);
        private final Shooter rightShooter = new Shooter(Constants.ShooterConstants.ShooterCanId3,
                        Constants.ShooterConstants.ShooterCanId4, m_hopper);

        // Turrets for testing
        private final Turret leftTurret = new Turret(Constants.TurretConstants.TurretCanId2,
                        Constants.TurretConstants.TurretOffset1, drivetrain, m_zoneDetection, TURRENT_SIDE.LEFT);
        private final Turret rightTurret = new Turret(Constants.TurretConstants.TurretCanId1,
                        Constants.TurretConstants.TurretOffset1, drivetrain, m_zoneDetection, TURRENT_SIDE.RIGHT);

        private final Hood rightHood = new Hood(Constants.HoodConstants.HoodCanId1);
        private final Hood leftHood = new Hood(Constants.HoodConstants.HoodCanId2);

        // private final Climber m_climber = new Climber();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                configureNamedCommands();
                configureBindings();

                // setting our inital pose
                drivetrain.resetPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(-90)));

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto selection", autoChooser);

                // Turret manual testing inputs
                SmartDashboard.putNumber("Turret/ManualPowerLeft", 0.0);
                SmartDashboard.putNumber("Turret/ManualPowerRight", 0.0);
                SmartDashboard.putNumber("Turret/ManualAngleLeft", 0.0);
                SmartDashboard.putNumber("Turret/ManualAngleRight", 0.0);
                SmartDashboard.putBoolean("Turret/UseManualPosition", false);

                // Hood manual testing inputs
                SmartDashboard.putNumber("Hood/ManualPowerLeft", 0.0);
                SmartDashboard.putNumber("Hood/ManualPowerRight", 0.0);
                SmartDashboard.putNumber("Hood/ManualAngleLeft", 0.0);
                SmartDashboard.putNumber("Hood/ManualAngleRight", 0.0);
                SmartDashboard.putBoolean("Hood/UseManualPosition", false);

                // Shooter manual testing inputs
                SmartDashboard.putNumber("Shooter/ManualPowerLeft", 0.0);
                SmartDashboard.putNumber("Shooter/ManualPowerRight", 0.0);
                SmartDashboard.putNumber("Shooter/ManualRPMMLeft", 0.0);
                SmartDashboard.putNumber("Shooter/ManualRPMMRight", 0.0);
                SmartDashboard.putBoolean("Shooter/UseManualRPM", false);
        }

        private void configureNamedCommands() {
                // Shoot While Moving - for use at PathPlanner waypoint events
                NamedCommands.registerCommand("ShootWhileMoving",
                                new ShootWhileMoving(
                                                rightTurret,
                                                rightShooter,
                                                rightHood,
                                                m_hopper,
                                                () -> drivetrain.getState().Pose,
                                                () -> drivetrain.getState().Speeds,
                                                this::getShootTargetTranslation,
                                                false) // use RightSpin
                                .withTimeout(15.0)); // Max duration for auto shooting segment

                // Intake Commands
                // NamedCommands.registerCommand("Intake On", m_intake.runIntakeCommand(() ->
                // drivetrain.getState().Speeds));
                // NamedCommands.registerCommand("Intake Off", m_intake.stopCommand());

                // Shooter Commands
                // NamedCommands.registerCommand("Rev Shooter", m_shooter.runShooterCommand());

                // Composite Auto Shoot (Brake -> Shoot -> Feed -> Retract Intake)
                /*
                 * Command autoShoot = Commands.parallel(
                 * drivetrain.applyRequest(() -> brake),
                 * m_shooter.runShooterCommand(),
                 * Commands.sequence(
                 * Commands.waitUntil(m_shooter::isAtSpeed),
                 * m_hopper.runShootFeedCommand(),
                 * m_intake.runRetractCommand()
                 * )
                 * );
                 */
                // NamedCommands.registerCommand("Auto Shoot", autoShoot);

                // Climber Commands
                // NamedCommands.registerCommand("Climb L1", new Climb(m_climber,
                // Climb.LEVELS.L1));
        }

        /** Returns the target translation for shooting (hub or passing corner) based on zone and alliance. */
        private Translation2d getShootTargetTranslation() {
                var alliance = DriverStation.getAlliance();
                if (!alliance.isPresent()) {
                        return Constants.FieldConstants.BlueTargetPose.getTranslation();
                }
                var zone = m_zoneDetection.getZone();
                Pose2d robotPose = drivetrain.getState().Pose;

                if (alliance.get() == DriverStation.Alliance.Blue) {
                        if (zone == ZoneDetection.ZONE.BLUE) {
                                return Constants.FieldConstants.BlueTargetPose.getTranslation();
                        } else if (zone == ZoneDetection.ZONE.NEUTRAL) {
                                return robotPose.getY() < Constants.FieldConstants.FieldWidth / 2.0
                                                ? Constants.FieldConstants.BluePassingCornerRight.getTranslation()
                                                : Constants.FieldConstants.BluePassingCornerLeft.getTranslation();
                        }
                } else {
                        if (zone == ZoneDetection.ZONE.RED) {
                                return Constants.FieldConstants.RedTargetPose.getTranslation();
                        } else if (zone == ZoneDetection.ZONE.NEUTRAL) {
                                return robotPose.getY() < Constants.FieldConstants.FieldWidth / 2.0
                                                ? Constants.FieldConstants.RedPassingCornerRight.getTranslation()
                                                : Constants.FieldConstants.RedPassingCornerLeft.getTranslation();
                        }
                }
                return Constants.FieldConstants.BlueTargetPose.getTranslation();
        }

        private void configureBindings() {
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

                // Shooter idle commands
                /*
                 * leftShooter.setDefaultCommand(
                 * Commands.run(() ->
                 * leftShooter.LeftSpin(Constants.ShooterConstants.IdleSpeed), leftShooter));
                 * rightShooter.setDefaultCommand(
                 * Commands.run(() ->
                 * rightShooter.RightSpin(Constants.ShooterConstants.IdleSpeed), rightShooter));
                 */

                // Reset the field-centric heading on start button press (right middle button)
                driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                drivetrain.registerTelemetry(logger::telemeterize);

                // ********************WORKING FUNCTIONS *****************************/

                // ********************FUNCTIONS For Testing*****************************/

                // TODO: Remove this manual binding in the future.
                driverController.b().whileTrue(m_hopper.runHopperCommand());
                driverController.a().onTrue(Commands.runOnce(() -> m_intake.runIntake(drivetrain.getState().Speeds)))
                                .onFalse(Commands.runOnce(() -> m_intake.stopIntake()));

                ParallelCommandGroup shootGroup = new ParallelCommandGroup(
                                Commands.run(() -> leftShooter.LeftSpin(Constants.ShooterConstants.IdleSpeed),
                                                leftShooter),
                                Commands.run(() -> rightShooter.RightSpin(Constants.ShooterConstants.IdleSpeed),
                                                rightShooter),
                                m_hopper.runShootCommand());

                ParallelCommandGroup stopShooters = new ParallelCommandGroup(
                                Commands.run(() -> leftShooter.LeftSpin(0), leftShooter),
                                Commands.run(() -> rightShooter.RightSpin(0)));

                driverController.rightTrigger(0.5f).whileTrue(shootGroup).onFalse(stopShooters);

                // Click to drop intake
                driverController.rightBumper().onTrue(m_intake.runDeployCommand());

                // Click to retract intake
                driverController.leftBumper().onTrue(m_intake.runRetractCommand());

                // TODO: Test command to deploy and run intake wheels simultaneously
                // driverController.leftTrigger().whileTrue(m_intake.runDeployAndIntakeCommand(()
                // -> drivetrain.getState().Speeds));

                // --- Turret Testing (D-Pad) ---
                // Left Turret: D-Pad Left/Right
                // driverController.povLeft().onTrue(rightHood.ManualHoodUp()).onFalse(rightHood.ManualHoodStop());
                // driverController.povRight().onTrue(rightHood.ManualHoodDown()).onFalse(rightHood.ManualHoodStop());

                // Right Turret: D-Pad Up/Down
                // driverController.povUp().onTrue(leftHood.ManualHoodUp()).onFalse(leftHood.ManualHoodStop());
                // driverController.povDown().onTrue(leftHood.ManualHoodDown()).onFalse(leftHood.ManualHoodStop());
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void testPeriodic() {
                boolean usePosition = SmartDashboard.getBoolean("Turret/UseManualPosition", false);
                if (usePosition) {
                        double leftAngle = SmartDashboard.getNumber("Turret/ManualAngleLeft", 0.0);
                        double rightAngle = SmartDashboard.getNumber("Turret/ManualAngleRight", 0.0);
                        leftTurret.setTargetAngle(leftAngle);
                        rightTurret.setTargetAngle(rightAngle);
                } else {
                        double leftPower = SmartDashboard.getNumber("Turret/ManualPowerLeft", 0.0);
                        double rightPower = SmartDashboard.getNumber("Turret/ManualPowerRight", 0.0);
                        leftTurret.setPower((float) leftPower);
                        rightTurret.setPower((float) rightPower);
                }

                boolean useHoodPosition = SmartDashboard.getBoolean("Hood/UseManualPosition", false);
                if (useHoodPosition) {
                        double leftHoodAngle = SmartDashboard.getNumber("Hood/ManualAngleLeft", 0.0);
                        double rightHoodAngle = SmartDashboard.getNumber("Hood/ManualAngleRight", 0.0);
                        leftHood.setTargetAngle(leftHoodAngle);
                        rightHood.setTargetAngle(rightHoodAngle);
                } else {
                        double leftHoodPower = SmartDashboard.getNumber("Hood/ManualPowerLeft", 0.0);
                        double rightHoodPower = SmartDashboard.getNumber("Hood/ManualPowerRight", 0.0);
                        leftHood.setPower(leftHoodPower);
                        rightHood.setPower(rightHoodPower);
                }

                // Shooter Testing
                boolean useShooterRPM = SmartDashboard.getBoolean("Shooter/UseManualRPM", false);
                if (useShooterRPM) {
                        double leftRPM = SmartDashboard.getNumber("Shooter/ManualRPMMLeft", 0.0);
                        double rightRPM = SmartDashboard.getNumber("Shooter/ManualRPMMRight", 0.0);
                        leftShooter.LeftSpin(leftRPM);
                        rightShooter.RightSpin(rightRPM);
                } else {
                        double leftShooterPower = SmartDashboard.getNumber("Shooter/ManualPowerLeft", 0.0);
                        double rightShooterPower = SmartDashboard.getNumber("Shooter/ManualPowerRight", 0.0);
                        leftShooter.setPowerLeft(leftShooterPower);
                        rightShooter.setPowerRight(rightShooterPower);
                }
        }

}
