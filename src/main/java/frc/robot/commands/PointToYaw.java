package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;

public class PointToYaw extends Command {
    private DoubleSupplier yawSupplier;

    private boolean wait;
    private DriveSubsystem robotDrive;
    private PIDController pidController = new PIDController(1, 0, 0);
    private Set<Subsystem> requirements;

    /**
     * Point to yaw
     * @param yawSupplier a supplier of desired yaw values (IN RADIANS!)
     * @param robotDrive the drive subsystem
     * @param wait whether or not to wait until it is completed to drive again
     */
    public PointToYaw(DoubleSupplier yawSupplier, DriveSubsystem robotDrive, boolean wait) {
        this.yawSupplier = yawSupplier;
        this.robotDrive = robotDrive;
        this.wait = wait;
        this.requirements = Set.of();
        if (wait) this.requirements = Set.of(robotDrive);
    }

    @Override
    public void execute() {
        pidController.setSetpoint(yawSupplier.getAsDouble());
        double rotation = pidController.calculate(robotDrive.getYawR());
        if (wait) {
            robotDrive.drive(0,0,rotation,false);
        }
        robotDrive.setTrackingRotation(rotation);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        if (wait)
            return InterruptionBehavior.kCancelIncoming;
        else
            return InterruptionBehavior.kCancelSelf;
    }

    @Override
    public void initialize() {
        // pidController.setTolerance(.1);
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        robotDrive.enableTracking();
    }

    @Override
    public void end(boolean interrupted) {
        robotDrive.disableTracking();
        robotDrive.setTrackingRotation(0);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    /**
    * generate a yaw from a POV value
    *
    * @param pov The POV value
    * @return yaw
    */
    public static double yawFromPOV(double pov) {
        double radians = Math.toRadians(pov);

        if (radians < -Math.PI) {
            double overshoot = radians + Math.PI;
            radians = -overshoot;
        }
        radians *= -1;
        return radians;
    }
    
    /**
    * generate a yaw from axis values
    *
    * @param xAxis
    * @param yAxis
    * @return yaw
    */
    public static double yawFromAxes(double xAxis, double yAxis) {
        double theta = Math.atan2(xAxis, yAxis);      
        return theta;
    }
    
    

    // public static double yawFromAxes(y)
}
