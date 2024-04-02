package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ejml.sparse.csc.factory.FillReductionFactory_FSCC;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.armAbsoluteControl;



public class ArmSubsystem extends SubsystemBase {

    private final CANcoder armAbsolutCANcoder;
        private final TalonFX ArmRightMotor;

    private final TalonFX ArmLeftMotor;
    // private final SparkPIDController pid1;
    // private final SparkPIDController pid2;
    // private static final double POSITION_TOLERANCE = 1.0;
    private final PIDController angleController;

    private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);

  private final PositionVoltage m_voltagePosition = new PositionVoltage(0, 0, false, 0, 0, true, false, false);

    public ArmSubsystem() {

        angleController = new PIDController(Constants.Arm.CAN.kP, Constants.Arm.CAN.kI, Constants.Arm.CAN.kD);
        angleController.setTolerance(0.02);
         ArmRightMotor = new TalonFX(31); 

        armAbsolutCANcoder = new CANcoder(Constants.Arm.CANcoder);
        ArmLeftMotor =new TalonFX(32); 


    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();
    /* User can optionally change the configs or leave it alone to perform a factory default */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfiguration.Feedback.FeedbackRemoteSensorID = armAbsolutCANcoder.getDeviceID();
    leftConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    leftConfiguration.Slot0.kP = Constants.Arm.CAN.kP;
    leftConfiguration.Slot0.kI = Constants.Arm.CAN.kI;
    leftConfiguration.Slot0.kD = Constants.Arm.CAN.kD;
    leftConfiguration.Voltage.PeakForwardVoltage = 9;
    leftConfiguration.Voltage.PeakReverseVoltage = -9;
    leftConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    leftConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    leftConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.CANRest;
    leftConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.CANAmp;

    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfiguration.Feedback.FeedbackRemoteSensorID = armAbsolutCANcoder.getDeviceID();
    rightConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    rightConfiguration.Slot0.kP = Constants.Arm.CAN.kP;
    rightConfiguration.Slot0.kI = Constants.Arm.CAN.kI;
    rightConfiguration.Slot0.kD = Constants.Arm.CAN.kD;
    rightConfiguration.Voltage.PeakForwardVoltage = 10;
    rightConfiguration.Voltage.PeakReverseVoltage = -10;
    rightConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    rightConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    rightConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Arm.CANRest;
    rightConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Arm.CANAmp;


    // absoluteEncoderConfiguration.MagnetSensor.

    ArmLeftMotor.getConfigurator().apply(leftConfiguration);
    ArmRightMotor.getConfigurator().apply(rightConfiguration);
        

        // ArmRightMotor.setControl(new Follower(ArmLeftMotor.getDeviceID(), false));

   

        

        // pid1.setSmartMotionMaxAccel(0, 0);

       
    }

    public Command armCommand(DoubleSupplier speed, BooleanSupplier rest, BooleanSupplier mid, BooleanSupplier max) {
        
            return run(() -> {
                this.runPercent(
                        speed.getAsDouble());
            });
        
    }

    // public Command setArmPositionCommand(double setpoint){
    // return runOnce(() -> {
    // runArm(setpoint);
    // });
    // }
    // public void setEncoder(double position) {
    //     ArmLeftMotor.getEncoder().setPosition(position);
    //     ArmRightMotor.getEncoder().setPosition(position);

    // }

    public void runPercent(double N_speed) {
        
        N_speed = Math.max( Math.min(N_speed , 0.75) , -0.75);
        leftOut.Output = N_speed;
        rightOut.Output = N_speed;

        ArmLeftMotor.setControl(leftOut);
        ArmRightMotor.setControl(rightOut);
    }

    // public void runArm(double N_speed) {
    // pid1.setReference(N_speed, ControlType.kVelocity);
    // pid2.setReference(N_speed, ControlType.kVelocity);

    // }

    public void stopArm() {
        ArmLeftMotor.set(0);
        ArmRightMotor.set(0);

    }

    // publicvoid resetArm() {
    //     ArmRightMotor.getEncoder().setPosition(0);
    //     ArmLeftMotor.getEncoder().setPosition(0);

    // } 
   
    public void setArmPosition(double setpoint) {
        if (setpoint > Constants.Arm.CANRest){
            setpoint = Constants.Arm.CANRest;
        }
        else if(setpoint < Constants.Arm.CANAmp) {
            setpoint = Constants.Arm.CANAmp;
        }
        // setpoint =Math.min( Math.max(setpoint , Constants.Arm.CANAmp ) , Constants.Arm.CANRest);
    //     pid1.setReference(setpoint, ControlType.kPosition);
    //     pid2.setReference(setpoint, ControlType.kPosition);

        ArmLeftMotor.setControl(m_voltagePosition.withPosition(setpoint));
        ArmRightMotor.setControl(m_voltagePosition.withPosition(setpoint));
    }

    public boolean isAtSetpoint(double setpoint, double tolerance) {
      return  Math.abs( ArmLeftMotor.getPosition().getValueAsDouble() - setpoint) < tolerance  ;  }

    public double getSetpoint(SwerveSubsystem drive){
        double setpoint = Constants.Arm.CAN90-0.05  +(0.0927*Math.log(drive.getLimelightA(1))) ;
        if (setpoint > Constants.Arm.CANRest){
            setpoint = Constants.Arm.CANRest;
        }
        if (setpoint < Constants.Arm.CANAmp){
            setpoint = Constants.Arm.CANAmp;
        }

        return  setpoint;
    }
    public boolean isAtSetpoint(SwerveSubsystem drive, double tolerance) {
        double setpoint = getSetpoint(drive);
        // double setpoint = Constants.Arm.CAN90 - ((55-Units.radiansToDegrees(Math.atan(0.5/drive.getLimelightA(1) )) )  * Constants.Arm.CANconv ) ;
      return  Math.abs( ArmLeftMotor.getPosition().getValueAsDouble() - setpoint) < tolerance  ;  }
    //     double currentPositionRight = ArmRightMotor.getEncoder().getPosition();
    //     double currentPositionLeft = ArmLeftMotor.getEncoder().getPosition();

    //     boolean rightAtSetpoint = Math.abs(setpoint - currentPositionRight) < POSITION_TOLERANCE;
    //     boolean leftAtSetpoint = Math.abs(setpoint - currentPositionLeft) < POSITION_TOLERANCE;

    //     return rightAtSetpoint && leftAtSetpoint;
    // }

    // public double getRightArmPosition() {
    //     return ArmRightMotor.getEncoder().getPosition();
    // }

    // public void resetEncoders(double setpoint) {
    //     ArmRightMotor.getEncoder().setPosition(setpoint);
    //     ArmLeftMotor.getEncoder().setPosition(setpoint);

    // }

    // public double getLeftArmPosition() {
    //     return ArmLeftMotor.getEncoder().getPosition();
    // }

    public double getAbsolutePosition() {
        return armAbsolutCANcoder.getPosition().getValueAsDouble();
    }

    public double getAbsolutePID(double setpoint) {

        return angleController.calculate(this.getAbsolutePosition(), setpoint);
    }
    public boolean getAbsoluteAtSetpoint() {
        return angleController.atSetpoint();
    }

    public void periodic() {
        SmartDashboard.putNumber("ARM Right Position", ArmRightMotor.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("ARM Absolute Position", armAbsolutCANcoder.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("ARM  Position", armAbsolutCANcoder.getPosition().getValueAsDouble()-Constants.Arm.CAN90);

        // SmartDashboard.putNumber("ARM Right Position", getRightArmPosition());
        // SmartDashboard.putNumber("ARM Left Position", getLeftArmPosition());

    }
}
