package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Logger;

public class Shooter extends SubsystemBase {

    SparkMax m_Shootermotor = new SparkMax(ShooterConstants.kShooterMotorID, MotorType.kBrushed);
    private SparkMaxConfig m_config = new SparkMaxConfig();

    private TrapezoidProfile m_profile = new TrapezoidProfile(new Constraints(ShooterConstants.kMaxVelocity, ShooterConstants.kMaxAccelrate));
    private TrapezoidProfile.State m_goal = new State();
    private TrapezoidProfile.State m_setpoint = new State();
    
    private PIDController pidConstants = new PIDController(ShooterConstants.kPID_Proportional, 0.0, 0.0);

    private double m_motorPower = 0;

    public Shooter() {
        m_config.closedLoop.pid(ShooterConstants.kPID_Proportional, 0.0, 0.0);
        m_config.inverted(true); //
        m_config.idleMode(IdleMode.kBrake);
        m_config.smartCurrentLimit(90);

        m_Shootermotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.putData("Shooter PID", pidConstants);
        ShooterConstants.kFeedForward = SmartDashboard.getNumber("Shooter Feedforward", 0);
    }

    public void shoot() {
        // set the motorPower
        // and release the ball into the flywheels
        // then reset power to 0 once shooting trigger is released
    }

    public void setReference(double val, ControlType type, double ff) {
        m_Shootermotor.getClosedLoopController().setReference(val, type, ClosedLoopSlot.kSlot0, ff);
    }

    @Override
    public void periodic() {
        setReference(m_setpoint.position, ControlType.kPosition, ShooterConstants.kFeedForward);
        SmartDashboard.putNumber("NEO Power", m_Shootermotor.get());
        
        m_config.closedLoop.pid(pidConstants.getP(), pidConstants.getI(), pidConstants.getD());
        m_Shootermotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        Logger.log("Intake", m_Shootermotor);
    }

    public void setPower(int pwr) {
        m_Shootermotor.set(pwr);
    }
}