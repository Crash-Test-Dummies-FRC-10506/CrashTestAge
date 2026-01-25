package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    SparkMax m_motor = new SparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushed);

    public Intake() {}

    public void setPower(double pow) {
        m_motor.set(pow);
    }
}