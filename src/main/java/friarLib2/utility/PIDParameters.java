package friarLib2.utility;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a set of PID constants
 */
public class PIDParameters implements Sendable
{
    private int SlotIndex;

    private double P;
    private double I;
    private double D;
    private double F;
    private double IZone;

    private final Set<BaseTalon> linkedMotors  = new HashSet<>();
    
    public PIDParameters (int slotIdx, String sendableName, double p, double i, double d, double f, double iZone)
    {
        this.P = p;
        this.I = i;
        this.D = d;
        this.F = f;
        this.IZone = iZone;
        SmartDashboard.putData(sendableName, this);
    }

    public PIDParameters (int slotIdx, String sendableName, double p, double i, double d)
    {
        this(slotIdx, sendableName, p, i, d, 0, 0);
    }

    /**
     * Helper method to initialize a Talon's PID parameters
     *
     * @param motor motor to configure
     */
    public void configureMotorPID (BaseTalon motor) {
        linkedMotors.add(motor);
        updateMotorPID();
    }

    public void updateMotorPID ()
    {
        for (BaseTalon linkedMotor : linkedMotors) {
            linkedMotor.config_kP(0, P);
            linkedMotor.config_kI(0, I);
            linkedMotor.config_kD(0, D);
            linkedMotor.config_kF(0, F);
            linkedMotor.config_IntegralZone(0, IZone);
        }
    }

	@Override
	public void initSendable(SendableBuilder builder)
    {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("iZone", this::getIZone, this::setIZone);
    }

    public double getP() { return P; }
    public double getI() { return I; }
    public double getD() { return D; }
    public double getF() { return F; }
    public double getIZone() { return IZone; }

    public void setP (double p)
    {
        this.P = p;
        updateMotorPID();
    }

    public void setI (double i)
    {
        this.I = i;
        updateMotorPID();
    }

    public void setD (double d)
    {
        this.D = d;
        updateMotorPID();
    }

    public void setF (double f)
    {
        this.F = f;
        updateMotorPID();
    }

    public void setIZone (double iZone)
    {
        this.IZone = iZone;
        updateMotorPID();
    }
}