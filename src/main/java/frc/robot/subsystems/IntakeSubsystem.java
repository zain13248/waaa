package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkFlex intakeMotor1;

    public IntakeSubsystem() {
        intakeMotor1 = new CANSparkFlex(9, MotorType.kBrushless);
    }

    public void runIntake(double speed) {
        intakeMotor1.set(speed);
    }

    public void stopIntake() {
        intakeMotor1.set(0);
    }

}
