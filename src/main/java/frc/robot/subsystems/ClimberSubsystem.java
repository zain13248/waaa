// package frc.robot.subsystems;

// import java.util.function.DoubleSupplier;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class ClimberSubsystem extends SubsystemBase {
//     private final TalonFX ClimberMotor;

//     private final DutyCycleOut ClimberOut = new DutyCycleOut(0);


//     public ClimberSubsystem() {
//         ClimberMotor = new TalonFX(41); 

//     var ClimberConfiguration = new TalonFXConfiguration();

//    // ClimberConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//     ClimberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//     ClimberConfiguration.Slot0.kP = 0.00165500000237487257;
//     ClimberConfiguration.Slot0.kI = 0;
//     ClimberConfiguration.Slot0.kD = 0;
//     ClimberConfiguration.Slot0.kV = 0.0001500000071246177;
//     ClimberConfiguration.Voltage.PeakForwardVoltage = 10;
//     ClimberConfiguration.Voltage.PeakReverseVoltage = -10;


//     ClimberMotor.getConfigurator().apply(ClimberConfiguration);
//     }




//     public Command climbCommand(DoubleSupplier speed){
//         return run(() -> {
//             this.runClimber(
//                     speed.getAsDouble());
//         });
//     }

//     public void runClimber(double speed) {
//         // speed = Math.max( Math.min(speed , 0.1) , -0.1);
//         ClimberOut.Output = speed;
//         ClimberMotor.setControl(ClimberOut);
//     }


//     public Command runOutTakeCommand(double speed){
//         return runOnce(() -> {
//             runClimber(speed);
//             }
//         );
//     }

//     public void stopOutTake() {
//         ClimberMotor.set(0);

//     }

//     public double getRightMotorVelocity() {
//         return ClimberMotor.getVelocity().getValueAsDouble();
//         }
        

//     public void periodic() {
//         double currentRightSpeed = this.getRightMotorVelocity();
//         SmartDashboard.putNumber("Climber ", currentRightSpeed);
//     }
// }
