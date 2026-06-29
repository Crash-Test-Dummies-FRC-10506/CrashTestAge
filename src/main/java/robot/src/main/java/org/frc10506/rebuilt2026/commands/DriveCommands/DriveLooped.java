package robot.src.main.java.org.frc10506.rebuilt2026.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import framework.src.main.java.org.frc10506.framework.control.Axis;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Drivetrain;

public class DriveLooped extends Command {
    private final Drivetrain drivetrain;

    private final Axis x;
    private final Axis y;

    private final Axis rotationAxis;

    public DriveLooped(Drivetrain drivetrain, Axis x, Axis y, Axis rotationAxis) {
        this.drivetrain = drivetrain;

        this.x = x;
        this.y = y;
        this.rotationAxis = rotationAxis;
    }


    @Override
    public void execute() {
        this.drivetrain.driveMechanum(this.x.get(), this.y.get(), this.rotationAxis.get());
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