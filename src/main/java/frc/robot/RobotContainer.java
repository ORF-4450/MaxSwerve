// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Team4450.Lib.NavX;
import Team4450.Lib.Util;
import Team4450.Lib.XboxController;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FaceAprilTag;
import frc.robot.commands.PointToYaw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem  robotDrive;
  private final Shooter         shooter;
  private final PhotonVision    camera;

  // The driver's controller
  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);

  // Navigation board, RoboLib wrapper.
  public static NavX			navx;

  // auto chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Util.consoleLog();

    // Create NavX object here since must done before CameraFeed is created (don't remember why).
    // Navx calibrates at power on and must complete before robot moves. Takes ~1 second for 2nd
    // generation Navx ~15 seconds for classic Navx. We assume there will be enough time between
    // power on and our first movement because normally things don't happen that fast

    // Warning: The navx instance is shared with the swerve drive code. Resetting or otherwise
    // manipulating the navx (as opposed to just reading data) may crash the swerve drive code.

    navx = NavX.getInstance(NavX.PortType.SPI);

    // Add navx as a Sendable. Updates the dashboard heading indicator automatically.
    
    SmartDashboard.putData("Gyro2", navx);

    robotDrive = new DriveSubsystem();
    shooter = new Shooter();
    camera = new PhotonVision();

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
                0,
                false),
            robotDrive));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    // new Trigger(() -> driverController.getRightBumper())
    //     .whileTrue(new StartEndCommand(robotDrive::enableAlternateRotation,
    //                                    robotDrive::disableAlternateRotation));
    new Trigger(() -> driverController.getRightBumper())
        .whileTrue(new PointToYaw(
            ()->PointToYaw.yawFromAxes(
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightY(), OIConstants.kDriveDeadband)
            ), robotDrive, false
        ));

    // the "A" button (or cross on PS4 controller) toggles tracking mode.
    new Trigger(() -> driverController.getAButton())
        .toggleOnTrue(new FaceAprilTag(camera, robotDrive));

    // POV buttons do same as alternate driving mode but without any lateral
    // movement and increments of 45deg.
    new Trigger(()-> driverController.getPOV() != -1)
        .onTrue(new PointToYaw(()->PointToYaw.yawFromPOV(driverController.getPOV()), robotDrive, true));

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
        return autoChooser.getSelected();
    }
}
