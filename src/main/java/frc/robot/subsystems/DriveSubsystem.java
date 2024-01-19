// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;

import Team4450.Lib.Util;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset, "FL");

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset, "FR");

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset, "RL");

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset, "RR");

  // The gyro sensor
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final AHRS    m_navx = new AHRS();

  private SimDouble     simAngle; // navx sim.

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private Pose2d        lastPose;
  private double        distanceTraveled, yawAngle, lastYawAngle;
  private boolean       fieldRelative = true, currentBrakeMode = false;
  private boolean       alternateRotation = false, istracking = false;

  // Field2d object creates the field display on the simulation and gives us an API
  // to control what is displayed (the simulated robot).

  private final Field2d     field2d = new Field2d();
  private Field2d           trackingField = new Field2d();  // Used by Advantage scope to show tracking target.

  private PIDController     trackingPid = new PIDController(0.8, 0.1, 0);
  private Pose2d            trackingPose = new Pose2d(1,5.6, new Rotation2d(0));

// Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private double speedLimiter = 1;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getGyroAngleDegrees()), //m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    Util.consoleLog("max vel=%.2f m/s", DriveConstants.kMaxSpeedMetersPerSecond);

    // This thread will wait a bit and then reset the navx while this constructor
    // continues to run. We do this because we have to wait a bit to reset the
    // navx after creating it.

    new Thread(() -> {
      try {
        Thread.sleep(2000);
        zeroHeading();
      } catch (Exception e) { }
    }).start();

    // Sets the module center translations from center of robot.
   m_frontLeft.setTranslation2d(new Translation2d(DriveConstants.kTrackWidth / 2.0, DriveConstants.kTrackWidth / 2.0));
   m_frontRight.setTranslation2d(new Translation2d(DriveConstants.kTrackWidth / 2.0, -DriveConstants.kTrackWidth / 2.0));
   m_rearLeft.setTranslation2d(new Translation2d(-DriveConstants.kTrackWidth / 2.0, DriveConstants.kTrackWidth / 2.0));
   m_rearRight.setTranslation2d(new Translation2d(-DriveConstants.kTrackWidth / 2.0, -DriveConstants.kTrackWidth / 2.0));

    if (RobotBase.isSimulation()) 
    {
      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
    }

    trackingPid.setIZone(Math.PI); // reset if error accumulator is too high
    trackingPid.enableContinuousInput(-Math.PI, Math.PI); // because roattion is periodic/continuous
    SmartDashboard.putData("Tracking_field", trackingField); // to visualize position robot is tracking to
    SmartDashboard.putData("tracking_pid", trackingPid); // for tuning in Shuffleboard
    trackingField.setRobotPose(trackingPose); // add tracking pose to field
    
    SmartDashboard.putData("Field2d", field2d);

    resetOdometry(DriveConstants.DEFAULT_STARTING_POSE);
  }

  @Override
  public void periodic() {
    // Update the odometry
    Pose2d currentPose = m_odometry.update(
        Rotation2d.fromDegrees(getGyroAngleDegrees()),   //m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    SmartDashboard.putNumber("angle", getGyroAngleDegrees());

    SmartDashboard.putString("robot pose", currentPose.toString());

    Transform2d poseOffset = currentPose.minus(lastPose);
    
    double currentDistance = poseOffset.getX() + poseOffset.getY();
    //double currentDistance = Math.sqrt(Math.pow(poseOffset.getX(), 2) + Math.pow(poseOffset.getY(), 2));

    distanceTraveled += currentDistance;

    SmartDashboard.putNumber("Distance Traveled(m)", distanceTraveled);
    
    // Track gyro yaw to support simulation of resettable yaw.

    yawAngle += m_navx.getAngle() - lastYawAngle;

    lastYawAngle = m_navx.getAngle();

    SmartDashboard.putNumber("Yaw Angle", getYaw());

    lastPose = currentPose;

    field2d.setRobotPose(currentPose);
    
    // Now update the pose of each wheel (module).
    updateModulePose(m_frontLeft);
    updateModulePose(m_frontRight);
    updateModulePose(m_rearLeft);
    updateModulePose(m_rearRight);

    setField2dModulePoses();
  }

  /**
   * Called on every scheduler loop when in simulation.
   */
  @Override
  public void simulationPeriodic() 
  {
    // We are not using this call now because the REV simulation does not work
    // correctly. Will leave the code in place in case this issue gets fixed.
    //if (robot.isEnabled()) REVPhysicsSim.getInstance().run();

     // want to simulate navX gyro changing as robot turns
    // information available is radians per second and this happens every 20ms
    // radians/2pi = 360 degrees so 1 degree per second is radians / 2pi
    // increment is made every 20 ms so radian adder would be (rads/sec) *(20/1000)
    // degree adder would be radian adder * 360/2pi
    // so degree increment multiplier is 360/100pi = 1.1459

    double temp = m_chassisSpeeds.omegaRadiansPerSecond * 1.1459155;

    temp += simAngle.get();

    simAngle.set(temp);

    Unmanaged.feedEnable(20);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getGyroAngleDegrees()), //m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);

      lastPose = pose;

      m_navx.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean rateLimit) 
  {  
    double xSpeedCommanded;
    double ySpeedCommanded;

    // Have to invert for sim...not sure why.
    if (RobotBase.isSimulation()) rot *= -1;

    if (rateLimit) 
    {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);

      m_currentRotation = m_rotLimiter.calculate(rot);
    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * speedLimiter * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * speedLimiter * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * speedLimiter * DriveConstants.kMaxAngularSpeed;

    // SmartDashboard.putNumber("0 xspeed", xSpeedDelivered);
    // SmartDashboard.putNumber("0 yspeed", ySpeedDelivered);
    // SmartDashboard.putNumber("0 rot", rotDelivered);

    m_chassisSpeeds =
        fieldRelative
//           ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle()))
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getGyroAngleDegrees()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    // SmartDashboard.putNumber("0 vxspeed", m_chassisSpeeds.vxMetersPerSecond);
    // SmartDashboard.putNumber("0 vyspeed", m_chassisSpeeds.vyMetersPerSecond);

    SwerveModuleState swerveModuleStates[] = DriveConstants.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    //SmartDashboard.putNumber("0 fl spd", swerveModuleStates[0].speedMetersPerSecond);
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
/**
   * Method to drive the robot using joystick info.
   * An overload of the drive method that adds a parameter for the Y component of the rotation joystick
   * and chooses what drive behavior to use (added by cole)
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotX          X component of rotation joystick
   * @param rotY          Y component of rotation joystick
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rotX, double rotY, boolean rateLimit) 
  {
    // for pointing rot joystick at angle to match robot to 
    if (alternateRotation && fieldRelative) 
    {
      // rotX and rotY are backwards because from the drivers perspective they are at 90deg to the field.
      double theta = Math.atan2(rotX, rotY);
      double theta_magnitude = Math.sqrt(Math.pow(rotX, 2) + Math.pow(rotY, 2));
      
      if (theta_magnitude > 0.2) // don't do anything if the joystick hasn't moved enough
        driveTracking(xSpeed, ySpeed, theta, rateLimit);
      else 
        drive(xSpeed, ySpeed, 0, rateLimit);
            
      // if the robot has enabled tracking to a specific pose 
      // driver has no control over heading of robot in this "mode"
    } else if (istracking) {
      Pose2d robotPose = getPose();
      
      double theta = Math.atan2(
          robotPose.getY() - trackingPose.getY(),
          robotPose.getX() - trackingPose.getX()
        ) + Math.PI;
      
      Util.consoleLog("Robot: (%f, %f)\nTrack To: (%f, %f)\nTheta: %f\n\n\n",
                      robotPose.getX(), robotPose.getY(),
                      trackingPose.getX(), trackingPose.getY(),
                      theta);

      driveTracking(xSpeed, ySpeed, theta, rateLimit);

      // just drive like normal, ignoring the rotY component 
    } else 
      drive(xSpeed, ySpeed, rotX, rateLimit);
  }

  /**
   * A custom method to drive the robot while tracking (facing) a specific heading.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param theta         Desired heading of the robot (IN RADIANS!)
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void driveTracking(double xSpeed, double ySpeed, double theta, boolean rateLimit) 
  {
    double currentHeading = Math.toRadians(getHeading()) % (Math.PI * 2.0); // ignore multiples of 2pi
    double rotSpeed = trackingPid.calculate(currentHeading, theta);

    // for tuning purposes:
    SmartDashboard.putNumber("tracking setpoint", theta);
    SmartDashboard.putNumber("tracking value", currentHeading);
    SmartDashboard.putNumber("applied rot speed", rotSpeed);

    // drive using the calculated rot_speed like normal
    drive(xSpeed, ySpeed, rotSpeed, rateLimit);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    //m_gyro.reset();
    m_navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_navx.getAngle()).getDegrees();  //m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    //return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return -m_navx.getRate(); // * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
  /**
   * Get Gyro angle in degrees.
   * @return Angle in degrees. 0 to +- 180.
   */
  public double getGyroAngleDegrees() 
  {
    double angle = Math.IEEEremainder((-m_navx.getAngle()), 360);
    SmartDashboard.putNumber("0 gyroangle", angle);
    return angle;
  }
  
  /**
   * Get gyro yaw from the angle of the robot at last gyro reset.
   * @return Rotation2D containing Gyro yaw in radians. + is left of zero (ccw) - is right (cw).
   */
  public Rotation2d getGyroRotation2d() 
  {
    if (m_navx.isMagnetometerCalibrated()) 
    {
     // We will only get valid fused headings if the magnetometer is calibrated
     return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    //return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    return Rotation2d.fromDegrees(-m_navx.getYaw());
  }

  /**
   * Update the pose of a swerve module on the field2d object. Module
   * pose is connected to the robot pose so they move together on the
   * field simulation display.
   * @param module Swerve module to update.
   */
  private void updateModulePose(MAXSwerveModule module)
  {
    Translation2d modulePosition = module.getTranslation2d()
        //.rotateBy(getHeadingRotation2d())
        .rotateBy(getPose().getRotation())
        .plus(getPose().getTranslation());
    
    module.setModulePose(
        new Pose2d(modulePosition, module.getHeading2d().plus(Rotation2d.fromDegrees(getGyroAngleDegrees()))));
  }

  /**
   * Rotates the module icons on the field display to indicate where
   * the wheel is pointing.
   */
  private void setField2dModulePoses()
  {
    Pose2d      modulePoses[] = new Pose2d[4];
    
    modulePoses[0] = m_frontLeft.getPose();
    modulePoses[1] = m_frontRight.getPose();
    modulePoses[2] = m_rearLeft.getPose();
    modulePoses[3] = m_rearRight.getPose();

    field2d.getObject("Swerve Modules").setPoses(modulePoses);
  }

  /**
   * Toggle the drive mode between field or robot relative.
   */
  public void toggleFieldRelative()
  {
      Util.consoleLog();
    
      fieldRelative = !fieldRelative;

      updateDS();
  }

  /**
   * Return the drive mode status.
   * @return True if field oriented, false if robot relative.
   */
  public boolean getFieldRelative()
  {
      return fieldRelative;
  }

  private void updateDS()
  {
      SmartDashboard.putBoolean("Field Relative", fieldRelative);
      SmartDashboard.putBoolean("Brakes on", currentBrakeMode);
      SmartDashboard.putBoolean("Alternate Drive", alternateRotation);
      SmartDashboard.putBoolean("Tracking", istracking);
  }

  /**
   * Gets the distance traveled by the robot drive wheels since the
   * last call to resetDistanceTraveled. This simulates a regular
   * encoder on the drive wheel. We can't use the actual wheel encoder
   * because resetting that encoder would crash the swerve drive code.
   * Note: This distance is only accurate for forward/backward and
   * strafe moves. 
   * @return
   */
  public double getDistanceTraveled()
  {
    return distanceTraveled; // * -1;
  }

  /**
   * Reset the distance traveled by the robot.
   */
  public void resetDistanceTraveled()
  {
    Util.consoleLog();
    
    distanceTraveled = 0;
  }

  /**
   * Returns the current yaw angle of the robot measured from the last
   * call to resetYaw(). Angle sign is WPILib convention, inverse of NavX.
   * @return The yaw angle. - is right (cw) + is left (ccw).
   */
  public double getYaw()
  {
    return -yawAngle;
  }

  /**
   * Returns the current yaw angle of the robot measured from the last
   * call to resetYaw().
   * @return The yaw angle in radians.
   */
  public double getYawR()
  {
    return Math.toRadians(-yawAngle);
  }

  /**
   * Set yaw angle to zero.
   */
  public void resetYaw()
  {
    Util.consoleLog();

    yawAngle = 0;
  }

  /**
   * Set drive motor idle mode for each swerve module. Defaults to coast.
   * @param on True to set idle mode to brake, false sets to coast.
   */
  public void setBrakeMode(boolean on) 
  {
      Util.consoleLog("%b", on);

      currentBrakeMode = on;
    
      m_frontLeft.setBrakeMode(on); 
      m_frontRight.setBrakeMode(on); 
      m_rearLeft.setBrakeMode(on); 
      m_rearRight.setBrakeMode(on); 

      updateDS();
  }

  /**
   * Toggles state of brake mode (brake/coast) for drive motors.
   */
  public void toggleBrakeMode()
  {
    setBrakeMode(!currentBrakeMode);
  }

  public void toggleFieldOriented()
  {
    this.fieldRelative = !this.fieldRelative;
    Util.consoleLog("field relative: %b", fieldRelative);
  }

  public void enableSlowMode()
  {
    speedLimiter = 1;
    Util.consoleLog("speed limiter: %f", speedLimiter);
  }

  public void disableSlowMode()
  {
    speedLimiter = 0.3;
    Util.consoleLog("speed limiter: %f", speedLimiter);
  }


  /**
   * Enables the alternate field-centric rotation method
   */
  public void enableAlternateRotation() {
    Util.consoleLog();

    this.alternateRotation = true;
    this.trackingPid.reset();
    this.trackingPid.setTolerance(1);

    updateDS();
  }

  /**
   * Disables the alternate field-centric rotation method
   */
  public void disableAlternateRotation() {
    Util.consoleLog();

    alternateRotation = false;

    updateDS();
  }

  /**
   * Set a pose to track rotation to
   */
  public void setTrackingPose(double x, double y) {
    Util.consoleLog("x=%.2f y=%.2f", x, y);

    trackingPose = new Pose2d(x, y, new Rotation2d(0));
    trackingField.setRobotPose(trackingPose);
  }

  /**
   * Set pose to track to as current robot pose
   */
  public void setTrackingPose() {
    setTrackingPose(getPose().getX(), getPose().getY());
  }

  /**
   * Enables tracking of a pre-specified Pose2d
   */
  public void enableTracking() {
    Util.consoleLog();

    istracking = true;

    updateDS();
  }

  /**
   * Disables tracking of a pre-specified Pose2d
   */
  public void disableTracking() {
    Util.consoleLog();

    istracking = false;

    updateDS();
  }
}
