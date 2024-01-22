// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Team4450.Lib.LibraryVersion;
import Team4450.Lib.Util;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private StringLogEntry myStringLog = new StringLogEntry(DataLogManager.getLog(), "CustomLogOutput");
  private void logLog(String x) {
    myStringLog.append(x);
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    
    // Set up our custom logger.

    try {
      Util.CustomLogger.setup("frc.robot.");
    } catch  (Exception e) { endCompetition(); }

    Util.consoleLog("%s", Constants.kProgramName);
    logLog("askjhdfgskdf");

    if (RobotBase.isSimulation()) 
      Util.consoleLog("Simulated Robot");
    else
      Util.consoleLog("Real Robot");
    
    // Send program version to the dashboard.
    SmartDashboard.putString("Program", Constants.kProgramName);

    // Log RobotLib and WPILib versions we are using. Note Robolib WPILib version can be different
    // than robot WPILib version. Should be the same for best results.
    Util.consoleLog("Robot WPILib=%s  Java=%s", WPILibVersion.Version, System.getProperty("java.version"));
    Util.consoleLog("RobotLib=%s", LibraryVersion.version);

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() { Util.consoleLog("------------------------------------"); }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Util.consoleLog("------------------------------------"); 

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Util.consoleLog("------------------------------------"); 

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /**
   * This function is called once at the start of test mode.
   */
  @Override
  public void testInit() 
  {
    Util.consoleLog();

    // Cancels all running commands at the start of test mode.

    CommandScheduler.getInstance().cancelAll();

    // Next two lines launch teleop mode, but since we are in test
    // mode, LiveWindow will be enabled to display test data to the
    // outlineviewer to shuffleboard. Our "test" mode is the regular
    // telop with LW enabled.

    LiveWindow.enableAllTelemetry();
    
    teleopInit();

    CommandScheduler.getInstance().enable();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
