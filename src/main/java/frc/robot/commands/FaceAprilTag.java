package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;

public class FaceAprilTag extends PIDCommand {
    DriveSubsystem robotDrive;

    public FaceAprilTag(PhotonVision camera, DriveSubsystem robotDrive) {
        super(
            new PIDController(0.15, 0.01, 0.01), // the PID Controller
            camera::getYaw, // measurement
            0, // setpoint
            (o) -> robotDrive.setTrackingRotation(o)
        );
        getController().setTolerance(2);
        robotDrive.enableTracking();
        this.robotDrive = robotDrive;
    }
    
    @Override
    public void end(boolean interrupted) {
        this.robotDrive.disableTracking();
    }
    
}
