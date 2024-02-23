package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OutTakeSubsystem extends SubsystemBase {
    private final CANSparkFlex intakeRightMotor;
    private final CANSparkFlex intakeLeftMotor;
    private final SparkPIDController pid1;
    private final SparkPIDController pid2;


    public OutTakeSubsystem() {
        intakeRightMotor = new CANSparkFlex(27, MotorType.kBrushless);
        intakeLeftMotor = new CANSparkFlex(24, MotorType.kBrushless);
        intakeRightMotor.restoreFactoryDefaults();
        intakeLeftMotor.restoreFactoryDefaults();

        intakeRightMotor.getEncoder().setVelocityConversionFactor(1);
        intakeLeftMotor.getEncoder().setVelocityConversionFactor(1);


        pid1 = intakeRightMotor.getPIDController();
        pid2 = intakeLeftMotor.getPIDController();
        
        // pid1.setSmartMotionMaxAccel(0, 0);

        pid1.setP(0.0005500000237487257);
        pid2.setP(0.0005000000237487257);

        pid1.setI(0);
        pid2.setI(0);

        pid1.setD(0);
        pid2.setD(0);

        pid1.setFF(0.0001500000071246177);
        pid2.setFF(0.0001500000071246177);

    }

    public void runOutTake(double N_speed, double P_speed) {
        pid1.setReference(N_speed, ControlType.kVelocity);
        pid2.setReference(-P_speed, ControlType.kVelocity);

    }

    public void stopOutTake() {
        pid1.setReference(0, ControlType.kVelocity);
        pid2.setReference(0 , ControlType.kVelocity);
        intakeLeftMotor.set(0);
        intakeRightMotor.set(0);

    }

    public double getRightMotorVelocity() {
        return intakeRightMotor.getEncoder().getVelocity();
        }
        
        public double getLeftMotorVelocity() {
        return intakeLeftMotor.getEncoder().getVelocity();
        }
    public void periodic() {
        double currentRightSpeed = this.getRightMotorVelocity();
        double currentLeftSpeed = this.getLeftMotorVelocity();

        SmartDashboard.putNumber("left Speed outake", currentLeftSpeed);
        SmartDashboard.putNumber("right Speed outake", currentRightSpeed);

        SmartDashboard.putBoolean("Outake Spool", Math.abs(currentRightSpeed - Constants.IntakeConstants.OUTTAKE_IN) <= Constants.Shooter.tolerance && Math.abs(currentLeftSpeed + Constants.IntakeConstants.OUTTAKE_OUT) <= Constants.Shooter.tolerance);
    }
}
