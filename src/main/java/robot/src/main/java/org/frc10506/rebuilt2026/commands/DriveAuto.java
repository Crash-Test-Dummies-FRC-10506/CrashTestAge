package robot.src.main.java.org.frc10506.viridian.commands;

import edu.wpi.first.wpilibj2.command.Command;
import framework.src.main.java.org.frc10506.framework.control.Axis;
import robot.src.main.java.org.frc10506.viridian.subsystems.MechanumDrive;

public class DriveAuto extends Command {
    private final MechanumDrive drivetrain;

    private double x;
    private double y;

    private double rot;

    public DriveAuto(MechanumDrive drivetrain, double x, double y, double rot) {
        this.drivetrain = drivetrain;

        this.x = x;
        this.y = y;
        this.rot = rot;
    }


    @Override
    public void execute() {
        this.drivetrain.driveMechanum(x, y, rot);
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.driveMechanum(0, 0, 0);
    }

    // Returns true when the command should end.
    //@Override
    //public boolean isFinished() {
    //    return false;
    //}
}