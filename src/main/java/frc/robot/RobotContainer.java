// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Team4450.Lib.Util;
import Team4450.Lib.XboxController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem  robotDrive = new DriveSubsystem();
  private final Shooter         shooter = new Shooter();

  // The driver's controller
  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Util.consoleLog();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightY(), OIConstants.kDriveDeadband),
                false),
            robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    Util.consoleLog();

    // Holding Left bumper brakes and sets X pattern to stop movement.
    new Trigger(() -> driverController.getLeftBumper())
        .whileTrue(new RunCommand(() -> robotDrive.setX(), robotDrive));

    // holding top right bumper enables the alternate rotation mode in
    // which the driver points stick to desired heading.
    new Trigger(() -> driverController.getRightBumper())
        .whileTrue(new StartEndCommand(robotDrive::enableAlternateRotation,
                                       robotDrive::disableAlternateRotation));

    // the "A" button (or cross on PS4 controller) toggles tracking mode.
    new Trigger(() -> driverController.getAButton())
        .toggleOnTrue(new StartEndCommand(robotDrive::enableTracking, robotDrive::disableTracking));

    // POV buttons do same as alternate driving mode but without any lateral
    // movement and increments of 45deg.
    new Trigger(()-> driverController.getPOV() != -1)
        .whileTrue(new RunCommand(
            () -> robotDrive.driveTracking(
                0,0,
                povToHeading(driverController.getPOV()),
                false
            ), robotDrive));

    // run shooter (manupulator controller)
    new Trigger(() -> manipulatorController.getBButton())
        .whileTrue(new StartEndCommand(shooter::start, shooter::stop, shooter));

    // reset field orientation
    new Trigger(() -> driverController.getStartButton())
        .onTrue(new InstantCommand(robotDrive::zeroGyro));

    // toggle field-oriented
    new Trigger(() -> driverController.getBackButton())
        .onTrue(new InstantCommand(robotDrive::toggleFieldRelative));

    // toggle slow-mode
    new Trigger(() -> driverController.getLeftTrigger())
        .whileTrue(new StartEndCommand(robotDrive::enableSlowMode, robotDrive::disableSlowMode));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        robotDrive::setModuleStates,
        robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
  }
  
  /**
  * Convert a POV reading (in degrees) to heading (-pi/2 to 0 to pi/2)
  *
  * @param pov The POV value
  * @return The heading in radians
  */
  private double povToHeading(double pov) {
    double radians = Math.toRadians(pov);

    if (radians < -Math.PI) {
        double overshoot = radians + Math.PI;
        radians = -overshoot;
    }

    radians *= -1;

    return radians;
  }
}
