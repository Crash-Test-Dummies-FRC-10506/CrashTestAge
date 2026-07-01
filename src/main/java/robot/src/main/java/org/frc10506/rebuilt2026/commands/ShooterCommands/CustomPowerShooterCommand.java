package robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Shooter;

public class CustomPowerShooterCommand extends Command {
    private final Shooter shooter;
    private final double power;

    public CustomPowerShooterCommand(Shooter shooter, double power) {
        this.shooter = shooter;
        this.power = power;
    }

    @Override
    public void execute() {
        this.shooter.setShooterRaw(power);
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
