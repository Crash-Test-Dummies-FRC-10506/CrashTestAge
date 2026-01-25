package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    SparkMax m_motor = new SparkMax(ShooterConstants.kShooterMotorID, MotorType.kBrushed);

    public Shooter() {}

    public void setPower(double pow) {
        m_motor.set(pow);
    }
}