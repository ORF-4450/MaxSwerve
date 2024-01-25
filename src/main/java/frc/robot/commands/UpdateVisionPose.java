package frc.robot.commands;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;

import Team4450.Lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.PhotonVision;


public class UpdateVisionPose extends Command {
    PhotonVision cameraSubsystem;
    DriveBase robotDrive;
    Set<Subsystem> requirements;
    
    /**
     * updates the odometry pose estimator to include sighted AprilTag positions from
     * PhotonVision pose estimator
     * @param cameraSubsystem the PhotonVision subsystem in use
     * @param robotDrive the drive base
     */
    public UpdateVisionPose(PhotonVision cameraSubsystem, DriveBase robotDrive) {
        this.cameraSubsystem = cameraSubsystem;
        this.robotDrive = robotDrive;

        // require camera subsystem for use in defaultcommand
        requirements = Set.of(cameraSubsystem);
    }

    @Override
    public void initialize() {
        Util.consoleLog();
    }

    @Override
    public void execute() {
        Optional<EstimatedRobotPose> estimatedPoseOptional = cameraSubsystem.getEstimatedPose();

        // update pose estimator pose with current epoch timestamp and the pose from the camera
        // if the camera has a good pose output
        // logic to decide if a pose is valid should be put in PhotonVision.java file, not here
        if (estimatedPoseOptional.isPresent()) {
            EstimatedRobotPose estimatedPoseContainer = estimatedPoseOptional.get();
             // pose2d to pose3d (ignore the Z axis which is height off ground)
            Pose2d pose2d = new Pose2d(
                estimatedPoseContainer.estimatedPose.getX(),
                estimatedPoseContainer.estimatedPose.getY(),
                new Rotation2d(estimatedPoseContainer.estimatedPose.getRotation().getAngle())
            );
            robotDrive.updateOdometryVision(pose2d, estimatedPoseContainer.timestampSeconds);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}
