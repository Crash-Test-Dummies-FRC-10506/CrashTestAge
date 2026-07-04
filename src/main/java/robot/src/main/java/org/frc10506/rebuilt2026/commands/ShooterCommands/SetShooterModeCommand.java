package robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Shooter;

public class SetShooterModeCommand extends Command {
    private final Shooter shooter;
    private final double mode;

    public SetShooterModeCommand(Shooter shooter, double mode) {
        this.shooter = shooter;
        this.mode = mode;
    }

    @Override
    public void execute() {
        this.shooter.setMode(mode);
    }

    //@Override
   // public boolean isFinished() {
    //    return false;
    //}

    @Override
    public void end(boolean interrupted) {
        this.shooter.setMode(0);
    }

}
