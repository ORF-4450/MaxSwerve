package frc.robot.commands;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagNames;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;


public class FaceAprilTag extends Command {
    DriveSubsystem robotDrive;
    PhotonVision camera;
    PIDController pidController = new PIDController(0.01, 0, 0);
    AprilTagNames tagNames = new AprilTagNames(Alliance.Blue);

    public FaceAprilTag(PhotonVision camera, DriveSubsystem robotDrive) {
        Util.consoleLog();
        pidController.setTolerance(0.3);
        this.robotDrive = robotDrive;
        this.camera = camera;
        SmartDashboard.putData("AprilTag PID", pidController);
    }

    @Override
    public void initialize() {
        Util.consoleLog();

        robotDrive.enableTracking();
    }
    
    @Override
    public void execute() {
        ArrayList<Integer> tags = camera.getTrackedIDs();
        PhotonTrackedTarget target;

        if (tags.contains(tagNames.SPEAKER_MAIN)) {
            target = camera.getTarget(tagNames.SPEAKER_MAIN);
        } else if (tags.contains(tagNames.AMP)) {
            target = camera.getTarget(tagNames.AMP);
        } else if (tags.size() > 0) {
            target = camera.getTarget(tags.get(0));
        } else {
            robotDrive.setTrackingRotation(Double.NaN);
            SmartDashboard.putBoolean("Has AprilTag", false);
            return;
        }

        double output = pidController.calculate(target.getYaw(), 0);
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
