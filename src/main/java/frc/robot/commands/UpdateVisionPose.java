package frc.robot.commands;
import java.util.Optional;

import Team4450.Lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;


public class UpdateVisionPose extends Command {
    PhotonVision cameraSubsystem;
    DriveSubsystem robotDrive;
    
    public UpdateVisionPose(PhotonVision cameraSubsystem, DriveSubsystem robotDrive) {
        this.cameraSubsystem = cameraSubsystem;
        this.robotDrive = robotDrive;
    }

    @Override
    public void initialize() {
        Util.consoleLog();
    }

    @Override
    public void execute() {
        Optional<Pose2d> pose = cameraSubsystem.getEstimatedPose();
        if (pose.isPresent())
            robotDrive.updateOdometryVision(pose.get(), Util.timeStamp());
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}
