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
    
    private PIDController pidConstants = new PIDController(???, 0.0, 0.0);

    private double ff = 0.77; // Feedforward

    private double motorPower = 0;

    public Shooter() {
        m_config.closedLoop.pid(???, 0, 0);
        m_config.inverted(true);
        m_config.idleMode(IdleMode.kBrake);
        m_config.smartCurrentLimit(???);

        m_elevMotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.putData("Shooter PID", pidConstants);
        
        SmartDashboard.putNumber("Shooter Feedforward", ff);
    }

    public void shoot() {
        // Set the motorPower
        // and release the ball into the flywheels
    }

    public void setReference(double val, ControlType type, double ff) {
        m_elevMotor.getClosedLoopController().setReference(val, type, ClosedLoopSlot.kSlot0, ff);
    }

    @Override
    public void periodic() {
        ff = SmartDashboard.getNumber("Elevator Feedforward", 0.7);

        m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal);

        setReference(m_setpoint.position, ControlType.kPosition, ff);
        
        SmartDashboard.putNumber("Target Elevator Position", m_setpoint.position);
        SmartDashboard.putNumber("Actual Elevator Position", m_elevMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("NEO Power", m_elevMotor.get());

        AutoConstants.elevatorHeights = SmartDashboard.getNumberArray("Elevator Heights", new double[] {0, 0, 0, 0});
        
        m_config.closedLoop.pid(pidConstants.getP(), pidConstants.getI(), pidConstants.getD());
        m_elevMotor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void setState(int pos) {
        m_goal = new TrapezoidProfile.State(AutoConstants.elevatorHeights[pos], 0);
    }

    public Command setGoal(int pos) {
        return runOnce(() -> {
            elevHeight = pos;
            setState(pos);
        });
    }

    public Command down() {
        return runOnce(() -> {
            elevHeight--;
            if(elevHeight < 0) elevHeight = 0;
            setState(elevHeight);
        });
    }

    public Command up() {
        return runOnce(() -> {
            elevHeight++;
            if(elevHeight > 3) elevHeight = 3;
            setState(elevHeight);
        });
    }

    public Command setPos(double pos) {
        return runOnce(() -> {
            m_goal = new TrapezoidProfile.State(pos, 0);
        });
    }

    public void reset() {
        elevHeight = 0;
        
        m_goal = new TrapezoidProfile.State(0, 0);
        m_setpoint = new TrapezoidProfile.State(0, 0);
    }
}