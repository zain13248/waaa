package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CANdleSystem;


public class IndexSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor1;
    private final DigitalInput intakeLimitSwitch;
    private Animation m_toAnimate;

    public IndexSubsystem() {
        intakeMotor1 = new CANSparkMax(3, MotorType.kBrushless);
        intakeLimitSwitch = new DigitalInput(4);
    }

    public boolean getLimit() {
        return intakeLimitSwitch.get();
    }

    public void runIndex(double speed) {
        intakeMotor1.set(speed);
    }

    public void stopIndex() {
        intakeMotor1.set(0);
    }
    public boolean indexCurrent(){
        return intakeMotor1.getOutputCurrent() > 13;

    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("INDEX Limit Switch", intakeLimitSwitch.get());
        SmartDashboard.putBoolean("INDEX Limit Current", intakeMotor1.getOutputCurrent() > 13);

        SmartDashboard.putNumber("INDEX Current" , intakeMotor1.getOutputCurrent());
                // SmartDashboard.putNumber("INDEX Current" , intakeMotor1.ge());

    }
}
