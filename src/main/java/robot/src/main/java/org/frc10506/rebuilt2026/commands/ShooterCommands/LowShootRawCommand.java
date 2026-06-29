package robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Shooter;

public class LowShootRawCommand extends Command {
    private final Shooter shooter;

    public LowShootRawCommand(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        this.shooter.setShooterRaw(0.6);
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
