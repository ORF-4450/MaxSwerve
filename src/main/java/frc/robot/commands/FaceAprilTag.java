package frc.robot.commands;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.PhotonVision;

public class FaceAprilTag extends PIDCommand {
    DriveBase  robotDrive;
    PhotonVision    camera;

    public FaceAprilTag(PhotonVision camera, DriveBase robotDrive) {
        super(
            new PIDController(0.001, 0, 0), // the PID Controller
            ()->camera.getYaw(), // measurement
            0, // setpoint
            (o) -> robotDrive.setTrackingRotation(camera.hasTargets() ? o : Double.NaN)
        );
=======
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagNames;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;


public class FaceAprilTag extends Command {
    DriveSubsystem robotDrive;
    PhotonVision photonVision;
    PIDController pidController = new PIDController(0.01, 0, 0);
    AprilTagNames tagNames = new AprilTagNames(Alliance.Red);
>>>>>>> d22f4c209d7ae745136e67698e015eb4bc6e21f8

    public FaceAprilTag(PhotonVision cameraSubsystem, DriveSubsystem robotDrive) {
        Util.consoleLog();

        // tolerance is in degrees
        pidController.setTolerance(0.3);
        this.robotDrive = robotDrive;
        this.photonVision = cameraSubsystem;
        SmartDashboard.putData("AprilTag Rotate PID", pidController);
    }

    @Override
    public void initialize() {
        Util.consoleLog();

        robotDrive.enableTracking();
    }
    
    @Override
    public void execute() {
        // get an arralist of all the tags IDs that the camera currently sees
        ArrayList<Integer> tags = photonVision.getTrackedIDs();
        PhotonTrackedTarget target;

        // first prioritize the center speaker tag if it is in view
        if (tags.contains(tagNames.SPEAKER_MAIN)) target = photonVision.getTarget(tagNames.SPEAKER_MAIN);

        // next try finding the amp tag
        else if (tags.contains(tagNames.AMP))target = photonVision.getTarget(tagNames.AMP);

        // finally, default to the first tag that isn't the center speaker or amp
        // could be offset speaker, trap source, etc, other alliance, etc.
        else if (tags.size() > 0) target = photonVision.getTarget(tags.get(0));

        // otherwise tell drivebase to set NaN as rotation to let driver override commanded
        // rotation to reorient the robot manually
        else {
            robotDrive.setTrackingRotation(Double.NaN);
            SmartDashboard.putBoolean("Has AprilTag", false);
            return;
        }

        // attempt to use PID controller to make target yaw approach 0 degrees
        double output = pidController.calculate(target.getYaw(), 0);

        // override joystick rotation input and use the PID output to turn
        // the robot instead
        robotDrive.setTrackingRotation(output);

        SmartDashboard.putNumber("AprilTag ID", target.getFiducialId());
        SmartDashboard.putBoolean("Has AprilTag", true);
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        this.robotDrive.disableTracking();
        SmartDashboard.putBoolean("Has AprilTag", false);
        SmartDashboard.putNumber("AprilTag ID", 0);
    }
}
