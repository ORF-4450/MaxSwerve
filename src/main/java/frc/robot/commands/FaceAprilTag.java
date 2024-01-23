package frc.robot.commands;

import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVision;

public class FaceAprilTag extends PIDCommand {
    DriveSubsystem robotDrive;
    PhotonVision camera;

    public FaceAprilTag(PhotonVision camera, DriveSubsystem robotDrive) {
        super(
            new PIDController(0.15, 0.01, 0.01), // the PID Controller
            ()->camera.getYaw(), // measurement
            0, // setpoint
            (o) -> robotDrive.setTrackingRotation(o)
        );
        getController().setTolerance(0.5);
        this.robotDrive = robotDrive;
        this.camera = camera;
        SmartDashboard.putData("april_tag_pid", this.getController());
    }

    @Override
    public void initialize() {
        robotDrive.enableTracking();
        super.initialize();
    }
    
    @Override
    public void execute() {
        Util.consoleLog("value: %f", -camera.getYaw());
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        this.robotDrive.disableTracking();
    }
    
}
