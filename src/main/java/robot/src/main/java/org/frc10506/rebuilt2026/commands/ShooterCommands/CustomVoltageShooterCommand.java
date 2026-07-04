package robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Shooter;

public class CustomVoltageShooterCommand extends Command {
    private final Shooter shooter;
    private final double voltage;

    public CustomVoltageShooterCommand(Shooter shooter, double voltage) {
        this.shooter = shooter;
        this.voltage = voltage;
    }

    @Override
    public void execute() {
        this.shooter.setShooterVoltage(voltage);
    }

    //@Override
   // public boolean isFinished() {
    //    return false;
    //}

    @Override
    public void end(boolean interrupted) {
        this.shooter.setShooterVoltage(0);
    }

}
