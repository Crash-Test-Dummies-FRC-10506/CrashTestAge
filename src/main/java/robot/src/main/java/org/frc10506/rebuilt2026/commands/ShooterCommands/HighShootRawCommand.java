package robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Shooter;

public class HighShootRawCommand extends Command {
    private final Shooter shooter;

    public HighShootRawCommand(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        this.shooter.setShooterRaw(0.72);
    }

    //@Override
   // public boolean isFinished() {
    //    return false;
    //}

    @Override
    public void end(boolean interrupted) {
        this.shooter.setShooterRaw(0);
    }

}
