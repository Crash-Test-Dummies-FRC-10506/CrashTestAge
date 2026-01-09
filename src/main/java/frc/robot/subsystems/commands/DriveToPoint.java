package frc.robot.subsystems.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPoint extends Command {

    private final DriveSubsystem m_drive;
    private final Pose2d target;
    private final double speed;

    private final PIDController xController = new PIDController(0.045, 0, 0);
    private final PIDController yController = new PIDController(0.045, 0, 0);
    private final PIDController headingController = new PIDController(0.045, 0, 0);
    private final double xyerrorTolerance = Units.inchesToMeters(2);
    private final double headingerrorTolerance = Math.toDegrees(90); // I could be wrong

    public DriveToPoint(DriveSubsystem drive, Pose2d point, double movespeed) {
        m_drive = drive;
        target = point;
        speed = movespeed;

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(xyerrorTolerance);
        yController.setTolerance(xyerrorTolerance);
        headingController.setTolerance(headingerrorTolerance);

        SmartDashboard.putData("X PID", xController);
        SmartDashboard.putData("Y PID", yController);
        SmartDashboard.putData("Heading PID", headingController);
        SmartDashboard.putNumber("X and Y Error Tolerance (Inches to Meters)", xyerrorTolerance);
        SmartDashboard.putNumber("Heading Error Tolerance (Degrees)", headingerrorTolerance);
        SmartDashboard.putNumber("Auto Driving Speed", speed);
    }

    @Override
    public void execute() {
        Pose2d currPose = m_drive.getPose();
        double xSpeed = xController.calculate(currPose.getX(), target.getX());
        double ySpeed = yController.calculate(currPose.getY(), target.getY());
       // Pose2d diff = new Pose2d(target.getX() - currPose.getX(), target.getY() - currPose.getY(), target.getRotation().minus(currPose.getRotation()));

        double angle = target.getRotation().getDegrees();
        if(angle > 180) angle = 360 - angle;
        double anglePow = headingController.calculate(currPose.getRotation().getDegrees(), angle);
    
        m_drive.driveCartesianFieldRelative(xSpeed * speed, ySpeed * speed, anglePow * speed);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() &&
               yController.atSetpoint() &&
               headingController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.driveCartesianFieldRelative(0, 0, 0);
    }
}