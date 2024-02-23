package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkFlex ArmRightMotor;
    private final CANSparkFlex ArmLeftMotor;
    private final SparkPIDController pid1;
    private final SparkPIDController pid2;
    private static final double POSITION_TOLERANCE = 1.0;

    public ArmSubsystem() {
        ArmRightMotor = new CANSparkFlex(31, MotorType.kBrushless);
        ArmLeftMotor = new CANSparkFlex(32, MotorType.kBrushless);
        pid1 = ArmRightMotor.getPIDController();
        pid2 = ArmLeftMotor.getPIDController();
        ArmRightMotor.restoreFactoryDefaults();
        ArmLeftMotor.restoreFactoryDefaults();

        ArmRightMotor.setInverted(true);
        ArmLeftMotor.setIdleMode(IdleMode.kBrake);
        ArmRightMotor.setIdleMode(IdleMode.kBrake);

        ArmRightMotor.getEncoder().setPositionConversionFactor(1);
        ArmLeftMotor.getEncoder().setPositionConversionFactor(1);

        // pid1.setSmartMotionMaxAccel(0, 0);

        pid1.setP(Constants.Arm.kP);
        pid2.setP(Constants.Arm.kP);

        pid1.setI(Constants.Arm.kI);
        pid2.setI(Constants.Arm.kI);

        pid1.setD(Constants.Arm.kD);
        pid2.setD(Constants.Arm.kD);

        pid1.setFF(Constants.Arm.kF);
        pid2.setFF(Constants.Arm.kF);

    }

    public Command armCommand(DoubleSupplier speed, DoubleSupplier speed2) {
        return run(() -> {
            this.runPercent(
                    speed.getAsDouble() > speed2.getAsDouble() ? speed.getAsDouble() / 10 : -speed2.getAsDouble() / 10);
        });
    }
    public void setEncoder(double position){
        ArmLeftMotor.getEncoder().setPosition(position);
        ArmRightMotor.getEncoder().setPosition(position);

    }
    public void runPercent(double N_speed) {
        ArmLeftMotor.set(N_speed);
        ArmRightMotor.set(N_speed);
    }

    public void runArm(double N_speed) {
        pid1.setReference(N_speed, ControlType.kVelocity);
        pid2.setReference(N_speed, ControlType.kVelocity);

    }

    public void stopArm() {
        ArmLeftMotor.set(0);
        ArmRightMotor.set(0);

    }

    public void resetArm() {
        ArmRightMotor.getEncoder().setPosition(0);
        ArmLeftMotor.getEncoder().setPosition(0);

    }

    public void setArmPosition(double setpoint) {
        pid1.setReference(setpoint, ControlType.kPosition);
        pid2.setReference(setpoint, ControlType.kPosition);

    }

    public boolean isAtSetpoint(double setpoint) {
        double currentPositionRight = ArmRightMotor.getEncoder().getPosition();
        double currentPositionLeft = ArmLeftMotor.getEncoder().getPosition();

        boolean rightAtSetpoint = Math.abs(setpoint - currentPositionRight) < POSITION_TOLERANCE;
        boolean leftAtSetpoint = Math.abs(setpoint - currentPositionLeft) < POSITION_TOLERANCE;

        return rightAtSetpoint && leftAtSetpoint;
    }

    public double getRightArmPosition() {
        return ArmRightMotor.getEncoder().getPosition();
    }

    public double getLeftArmPosition() {
        return ArmLeftMotor.getEncoder().getPosition();
    }

    public void periodic() {

        SmartDashboard.putNumber("Right Arm Position", getRightArmPosition());
        SmartDashboard.putNumber("Left Arm Position", getLeftArmPosition());

    }
}
