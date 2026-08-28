package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Logger;

public class Shooter extends SubsystemBase {
    private TalonFX shooterMotor;
    private MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);

    public Shooter() {
        shooterMotor = new TalonFX(ShooterConstants.kShooterMotorID);
        shooterMotor.setPosition(0);
        shooterMotor.getConfigurator().apply(ShooterConstants.GetShooterConstants());
    }

    public void shootMotionMagic(double velocity) {
        motionMagic.Slot = 0;
        shooterMotor.setControl(motionMagic.withVelocity(velocity));
    }

    public void shoot(double power) {
        shooterMotor.set(power);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", shooterMotor.get());
    }
}
