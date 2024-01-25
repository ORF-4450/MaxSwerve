// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import Team4450.Lib.Util;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utility.SwerveUtils;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset, "FL");

  private final MAXSwerveModule frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset, "FR");

  private final MAXSwerveModule rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset, "RL");

  private final MAXSwerveModule rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset, "RR");

  // The gyro sensor
  //private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private final AHRS    navx = RobotContainer.navx.getAHRS();

  private SimDouble     simAngle; // navx sim.

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private Pose2d        lastPose;
  private double        distanceTraveled, yawAngle, lastYawAngle;
  private boolean       fieldRelative = true, currentBrakeMode = false;
  private boolean       alternateRotation = false, istracking = false;
  private double        trackingRotation = 0;

  // Field2d object creates the field display on the simulation and gives us an API
  // to control what is displayed (the simulated robot).

  private final Field2d     field2d = new Field2d();

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private double speedLimiter = 1;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  // SwerveDriveOdometry odometry = new SwerveDriveOdometry(
  //     DriveConstants.kDriveKinematics,
  //     Rotation2d.fromDegrees(getGyroYaw()), //gyro.getAngle()),
  //     new SwerveModulePosition[] {
  //         frontLeft.getPosition(),
  //         frontRight.getPosition(),
  //         rearLeft.getPosition(),
  //         rearRight.getPosition()
  //     });

  // TODO: Fix the vectors used to set std deviations for measurements. Using default
  // for now. Not sure how to determine the values.
  private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getGyroYaw()), //gyro.getAngle()),
       new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        },
      new Pose2d());
      // VecBuilder.fill(0.1, 0.1, 0.1),
      // VecBuilder.fill(0.05),
      // VecBuilder.fill(0.1, 0.1, 0.1));

  /** Creates a new DriveSubsystem. */
  public DriveBase() {
    Util.consoleLog("max vel=%.2f m/s", DriveConstants.kMaxSpeedMetersPerSecond);

    // This thread will wait a bit and then reset the navx while this constructor
    // continues to run. We do this because we have to wait a bit to reset the
    // navx after creating it.

    new Thread(() -> {
      try {
        Thread.sleep(2000);
        zeroGyro();
      } catch (Exception e) { }
    }).start();

    // Sets the module center translations from center of robot.
   frontLeft.setTranslation2d(new Translation2d(DriveConstants.kTrackWidth / 2.0, DriveConstants.kTrackWidth / 2.0));
   frontRight.setTranslation2d(new Translation2d(DriveConstants.kTrackWidth / 2.0, -DriveConstants.kTrackWidth / 2.0));
   rearLeft.setTranslation2d(new Translation2d(-DriveConstants.kTrackWidth / 2.0, DriveConstants.kTrackWidth / 2.0));
   rearRight.setTranslation2d(new Translation2d(-DriveConstants.kTrackWidth / 2.0, -DriveConstants.kTrackWidth / 2.0));

    if (RobotBase.isSimulation())
    {
      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
    }

    SmartDashboard.putData("Field2d", field2d);

    if (ModuleConstants.kDrivingMotorIdleMode == IdleMode.kBrake) currentBrakeMode = true;

    resetOdometry(DriveConstants.DEFAULT_STARTING_POSE);

    configureAutoBuilder();
  }

  @Override
  public void periodic() {
    // Update the odometry
    Pose2d currentPose = odometry.update(
        Rotation2d.fromDegrees(getGyroYaw()),   //gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });

    SmartDashboard.putNumber("Gyro angle", getGyroYaw());
    //SmartDashboard.putNumber("Gyro turn rate", getTurnRate());

    SmartDashboard.putString("Robot pose", currentPose.toString());

    Transform2d poseOffset = currentPose.minus(lastPose);

    double currentDistance = poseOffset.getX() + poseOffset.getY();
    //double currentDistance = Math.sqrt(Math.pow(poseOffset.getX(), 2) + Math.pow(poseOffset.getY(), 2));

    distanceTraveled += currentDistance;

    SmartDashboard.putNumber("Distance Traveled(m)", distanceTraveled);

    // Track gyro yaw to support simulation of resettable yaw.

    yawAngle += navx.getAngle() - lastYawAngle;

    lastYawAngle = navx.getAngle();

    SmartDashboard.putNumber("Yaw Angle", getYaw());

    lastPose = currentPose;

    field2d.setRobotPose(currentPose);

    // Now update the pose of each wheel (module).
    updateModulePose(frontLeft);
    updateModulePose(frontRight);
    updateModulePose(rearLeft);
    updateModulePose(rearRight);

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
    // increment is made every 20 ms so radian adder would be (rads/sec) * (20/1000)
    // degree adder would be radian adder * 360/2pi
    // so degree increment multiplier is 360/100pi = 1.1459

    double temp = chassisSpeeds.omegaRadiansPerSecond * 1.1459155;

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
    return odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(getGyroYaw()), //gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);

      lastPose = pose;

      navx.reset();
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

    // override joystick value if tracking AND trackingRotation is real
    if (istracking && !Double.isNaN(trackingRotation)) rot = trackingRotation;

    // Have to invert for sim...not sure why.
    if (RobotBase.isSimulation()) rot *= -1;

    if (rateLimit)
    {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;

      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);

      if (angleDif < 0.45*Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        }
        else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }

      prevTime = currentTime;

      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);

      currentRotation = rotLimiter.calculate(rot);
    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * speedLimiter * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * speedLimiter * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * speedLimiter * DriveConstants.kMaxAngularSpeed;

    chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getGyroYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    driveChassisSpeeds(chassisSpeeds);
  }

  // for pathplanner
  public ChassisSpeeds getChassisSpeeds() {
    return this.chassisSpeeds;
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState swerveModuleStates[] = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }


  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 0 to 359.
   */
  public double getHeading() {
    return RobotContainer.navx.getHeadingInt();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    //return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return -navx.getRate(); // * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Get Gyro angle in degrees.
   * @return Angle in degrees. 0 to +- 180.
   */
  public double getGyroYaw()
  {
    double angle = Math.IEEEremainder((-navx.getAngle()), 360);

    return angle;
  }

  /**
   * Get gyro yaw from the angle of the robot at last gyro reset.
   * @return Rotation2D containing Gyro yaw in radians. + is left of zero (ccw) - is right (cw).
   */
  public Rotation2d getGyroYaw2dx()
  {
    if (navx.isMagnetometerCalibrated())
    {
     // We will only get valid fused headings if the magnetometer is calibrated
     return Rotation2d.fromDegrees(navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    //return Rotation2d.fromDegrees(360.0 - navx.getYaw());
    return Rotation2d.fromDegrees(-navx.getYaw());
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
        new Pose2d(modulePosition, module.getAngle2d().plus(Rotation2d.fromDegrees(getGyroYaw()))));
  }

  /**
   * Rotates the module icons on the field display to indicate where
   * the wheel is pointing.
   */
  private void setField2dModulePoses()
  {
    Pose2d      modulePoses[] = new Pose2d[4];

    modulePoses[0] = frontLeft.getPose();
    modulePoses[1] = frontRight.getPose();
    modulePoses[2] = rearLeft.getPose();
    modulePoses[3] = rearRight.getPose();

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
      SmartDashboard.putNumber("Speed Factor", speedLimiter);
  }

  /**
   * Gets the simulated distance traveled by the robot drive wheels since the
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
   * Reset the simulated distance traveled by the robot.
   */
  public void resetDistanceTraveled()
  {
    Util.consoleLog();

    distanceTraveled = 0;
  }

  /**
   * Returns the current simulated yaw angle of the robot measured from the last
   * call to resetYaw(). Angle sign is WPILib convention, inverse of NavX.
   * @return The yaw angle. - is right (cw) + is left (ccw).
   */
  public double getYaw()
  {
    return -yawAngle;
  }

  /**
   * Returns the current simulated yaw angle of the robot measured from the last
   * call to resetYaw().
   * @return The yaw angle in radians.
   */
  public double getYawR()
  {
    return Math.toRadians(-yawAngle);
  }

  /**
   * Set simulated yaw angle to zero.
   */
  public void resetYaw()
  {
    Util.consoleLog();

    yawAngle = 0;
  }

  /**
   * Sets the gyroscope yaw angle to zero. This can be used to set the direction
   * the robot is currently facing to the 'forwards' direction.
   */
  public void zeroGyro()
  {
    Util.consoleLog();

    navx.reset();
  }

  /**
   * Set drive motor idle mode for each swerve module. Defaults to coast.
   * @param on True to set idle mode to brake, false sets to coast.
   */
  public void setBrakeMode(boolean on)
  {
      Util.consoleLog("%b", on);

      currentBrakeMode = on;

      frontLeft.setBrakeMode(on);
      frontRight.setBrakeMode(on);
      rearLeft.setBrakeMode(on);
      rearRight.setBrakeMode(on);

      updateDS();
  }

  /**
   * Toggles state of brake mode (brake/coast) for drive motors.
   */
  public void toggleBrakeMode()
  {
    Util.consoleLog("%b", !currentBrakeMode);

    setBrakeMode(!currentBrakeMode);
  }

  /**
   * Enables the alternate field-centric rotation method
   */
  public void enableAlternateRotation() {
    Util.consoleLog();

    this.alternateRotation = true;
    
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

  public void enableSlowMode()
  {
    speedLimiter = DriveConstants.kSlowModeFactor;

    Util.consoleLog("%.2f", speedLimiter);

    updateDS();
  }

  public void disableSlowMode()
  {
    Util.consoleLog();

    speedLimiter = 1;

    updateDS();
  }

  public void setTrackingRotation(double o) {
    Util.consoleLog("%.2f", o);

    trackingRotation = o;
  }

  private void configureAutoBuilder() {
    Util.consoleLog();

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(AutoConstants.kHolonomicPathFollowerP, 0.0, 0.0), // Translation PID constants
              new PIDConstants(AutoConstants.kHolonomicPathFollowerP, 0.0, 0.0), // Rotation PID constants
              DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
              DriveConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          // RICH use global alliance info.
          var alliance = DriverStation.getAlliance();
          
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }

          return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }
}
