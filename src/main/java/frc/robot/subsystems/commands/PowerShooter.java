package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PowerShooter extends Command {

    private final Shooter m_Shooter;
    private final double m_pow;

    public PowerShooter(Shooter shooter, double pow) {
        m_Shooter = shooter;
        m_pow = pow;
    }

    @Override
    public void execute() {
        m_Shooter.shoot(m_pow);
    }
}