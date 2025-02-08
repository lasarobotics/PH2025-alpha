package frc.robot.subsystems;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private Spark climberMotor;

    public Hardware(Spark climberMotor) {
    this.climberMotor = climberMotor;
    }
  }

  private Spark m_climberMotor;

  private Double m_climberSpeed;

  /**
   * Create an instance of Climber Subsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * 
   * @param climbHardware Hardware devices required by climb
   */
  public ClimberSubsystem(Hardware climberHardware, Double climberSpeed) {
    this.m_climberMotor = climberHardware.climberMotor;
    this.m_climberSpeed = climberSpeed;
  }

  /**
   * Initialize hardware devices for amp subsystem
   * 
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware climbHardware = new Hardware(
      new Spark(Constants.Climber.CLIMB_MOTOR_ID, MotorKind.NEO)
    );
    return climbHardware;
  }

  /**
   * Stop climb
   */
  private void stop() {
    m_climberMotor.stopMotor();
  }

  /**
   * Runs the climb motor to raise arm
   */
  private void raiseClimber() {
    m_climberMotor.set(m_climberSpeed.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Runs the climb motor to lower arm
   */
  private void lowerClimber() {
    m_climberMotor.set(-m_climberSpeed.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Command to raise the climber hook
   * @return Command to run climber motor
   */
  public Command raiseClimbCommand() {
    return startEnd(() -> raiseClimber(), () -> stop());
  }

  /**
   * Command to lower the climber hook
   * @return Command to run climber motor in reverse
   */
  public Command lowerClimbCommand() {
    return startEnd(() -> lowerClimber(), () -> stop());
  }
  
  /**
   * Stop command
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  @Override
  public void close() {
    m_climberMotor.close();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_climberMotor.periodic();
  }
}