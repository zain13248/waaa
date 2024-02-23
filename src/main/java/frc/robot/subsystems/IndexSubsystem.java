package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor1;
    private final DigitalInput intakeLimitSwitch;


    public IndexSubsystem() {
        intakeMotor1 = new CANSparkMax(12, MotorType.kBrushless);
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

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Limit Switch", intakeLimitSwitch.get());
    }
}
