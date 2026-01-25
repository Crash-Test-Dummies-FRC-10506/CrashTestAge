package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    SparkMax m_motor = new SparkMax(ShooterConstants.kShooterMotorID, MotorType.kBrushed);
    private SparkMaxConfig m_config = new SparkMaxConfig();

    private TrapezoidProfile m_profile = new TrapezoidProfile(new Constraints(???, ???));
    private TrapezoidProfile.State m_goal = new State();
    private TrapezoidProfile.State m_setpoint = new State();
    
    private PIDController pidConstants = new PIDController(kPID_Proportional, 0.0, 0.0);

    private double m_motorPower = 0;

    public Shooter() {
        m_config.closedLoop.pid(kPID_Proportional, 0.0, 0.0);
        m_config.inverted(true); //
        m_config.idleMode(IdleMode.kBrake);
        m_config.smartCurrentLimit(???);

        m_elevMotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.putData("Shooter PID", pidConstants);
    }

    public void shoot() {
        // set the motorPower
        // and release the ball into the flywheels
        // then reset power to 0 once shooting trigger is released
    }

    public void setReference(double val, ControlType type) {
        m_motor.getClosedLoopController().setReference(val, type, ClosedLoopSlot.kSlot0, 0.0);
    }

    @Override
    public void periodic() {
        m_setpower = m_profile.calculate(0.02 ???, m_setpower, m_goal);

        setReference(m_setpoint.position, ControlType.kPosition);
        
        SmartDashboard.putNumber("Target Shooter Power", m_setpower.position);
        SmartDashboard.putNumber("Actual Shooter Power", m_motor.getEncoder().getPosition());
        SmartDashboard.putNumber("NEO Power", m_motor.get());
        
        m_config.closedLoop.pid(pidConstants.getP(), pidConstants.getI(), pidConstants.getD());
        m_elevMotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command setGoal(int pwr) {
        return runOnce(() -> {
            m_motorPower = pwr;
            setState(pos);
        });
    }}

    public Command setPower(double pwr) {
        return runOnce(() -> {
            m_goal = new TrapezoidProfile.State(pwr, 0.0);
        });
    }

    public void reset() {
        m_motorPowe = 0.0;
        
        m_goal     = new TrapezoidProfile.State(0.0, 0.0);
        m_setpower = new TrapezoidProfile.State(0.0, 0.0);
    }
}