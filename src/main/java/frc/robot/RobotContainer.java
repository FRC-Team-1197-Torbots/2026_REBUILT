// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.Commands.Climb;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.ZoneDetection;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /***************************
     * TORBOTS SPECIFIC VARIABLES
     ******************************/
    private final Intake m_intake = new Intake();
    private final Hopper m_hopper = new Hopper(joystick, m_intake);
    private final Shooter m_shooter = new Shooter();
    private final Climber m_climber = new Climber();
    private final Hood m_hood = new Hood();
    // Use the drivetrain's Pigeon2 for ZoneDetection
    // private final ZoneDetection m_zoneDetection = new ZoneDetection(drivetrain,
    // drivetrain.getPigeon2());

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureNamedCommands();
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto selection", autoChooser);
    }

    private void configureNamedCommands() {
        // Intake Commands
        NamedCommands.registerCommand("Intake On", m_intake.runIntakeCommand(() -> drivetrain.getState().Speeds));
        NamedCommands.registerCommand("Intake Off", m_intake.stopCommand());

        // Shooter Commands
        NamedCommands.registerCommand("Rev Shooter", m_shooter.runShooterCommand());
        
        // Composite Auto Shoot (Brake -> Shoot -> Feed -> Retract Intake)
        Command autoShoot = Commands.parallel(
            drivetrain.applyRequest(() -> brake),
            m_shooter.runShooterCommand(),
            Commands.sequence(
                Commands.waitUntil(m_shooter::isAtSpeed),
                m_hopper.runShootFeedCommand(),
                m_intake.runRetractCommand()
            )
        );
        NamedCommands.registerCommand("Auto Shoot", autoShoot);

        // Climber Commands
        NamedCommands.registerCommand("Climb L1", new Climb(m_climber, Climb.LEVELS.L1));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        m_shooter.setDefaultCommand(m_shooter.runIdleCommand());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        drivetrain.registerTelemetry(logger::telemeterize);

        //********************WORKING FUNCTIONS *****************************/

        joystick.a().whileTrue(
                Commands.parallel(
                        m_intake.runIntakeCommand(drivetrain.getState().Speeds),
                        m_hopper.runIndexCommand()));


        // Brake (X-Stance): hold Right Bumper
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        //********************FUNCTIONS For Testing*****************************/

        // TODO: Remove this manual binding in the future.
        joystick.b().whileTrue(m_hopper.runHopperCommand());

        joystick.x().whileTrue(m_climber.getAlignToClimbCommand());

        // Agitate: hold Y
        joystick.y().whileTrue(m_intake.runAgitateCommand());

        // Run Shooter, wait for speed, then run Hopper (Machine Gun).
        // The parallel group keeps the Shooter running.
        // The sequence waits for speed, then runs the Hopper feed command.
        // Climber Controls
        joystick.pov(0).onTrue(new Climb(m_climber, Climb.LEVELS.L3));
        joystick.pov(90).onTrue(new Climb(m_climber, Climb.LEVELS.L2));
        joystick.pov(270).onTrue(new Climb(m_climber, Climb.LEVELS.L1));
        joystick.pov(180).onTrue(new Climb(m_climber, Climb.LEVELS.L0));

        joystick.rightTrigger().whileTrue(
                Commands.parallel(
                        drivetrain.applyRequest(() -> brake),
                        m_shooter.runShooterCommand(),
                        Commands.sequence(
                                Commands.waitUntil(m_shooter::isAtSpeed),
                                m_hopper.runShootFeedCommand(),
                                m_intake.runRetractCommand())));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void testPeriodic() {
        if (joystick.getHID().getLeftBumper()) {
            m_shooter.Spin();
        } else {
            m_shooter.Stop();
        }

        m_hood.GoToAngle();
        m_zoneDetection.publishRawDistance();
    }
}
