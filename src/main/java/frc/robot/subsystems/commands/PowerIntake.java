package frc.robot.subsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class PowerIntake extends Command {

    private final Intake m_intake;
    private final double m_pow;

    public PowerIntake(Intake intake, double pow) {
        m_intake = intake;
        m_pow = pow;
    }

    @Override
    public void execute() {
        m_intake.setPower(m_pow);
    }
}