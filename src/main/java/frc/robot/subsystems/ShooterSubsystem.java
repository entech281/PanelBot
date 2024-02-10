// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.utilities.MovingAverage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private double m_speed = 0.0;
  private boolean shooter = false;
  private boolean intake = false;
  private CANSparkMax masterMotor;
  private CANSparkMax followMotor1;
  private CANSparkMax followMotor2;

  private MovingAverage motorCurrent = new MovingAverage(100);
  private Timer timer = new Timer();
  double FF_GAIN = 0.00017;
  double MAX_SPEED = 4000;
  double P_GAIN = 0.0004;
  double I_GAIN = 0;
  /** Creates a new ExampleSubsystem. */
  SparkPIDController masterPid;
  SparkPIDController followPid;
  public ShooterSubsystem() {
    m_speed = 0.0;
    shooter = false;
    intake  = false;

    masterMotor  = new CANSparkMax(Constants.CAN.MOTOR1, MotorType.kBrushless);
    followMotor1 = new CANSparkMax(Constants.CAN.MOTOR2, MotorType.kBrushless);
    followMotor2 = new CANSparkMax(Constants.CAN.MOTOR3, MotorType.kBrushless);

    masterMotor.getEncoder().setVelocityConversionFactor(1.0);
    followMotor1.getEncoder().setVelocityConversionFactor(1.0);
    followMotor2.getEncoder().setVelocityConversionFactor(1.0);

    masterMotor.setIdleMode(IdleMode.kCoast);
    followMotor1.setIdleMode(IdleMode.kCoast);
    followMotor2.setIdleMode(IdleMode.kCoast);

    masterMotor.restoreFactoryDefaults();
    followMotor1.restoreFactoryDefaults();
    followMotor2.restoreFactoryDefaults();

    masterMotor.setInverted(true);
    followMotor1.setInverted(true);
    followMotor2.setInverted(false);

     masterPid = masterMotor.getPIDController();
     masterPid.setD(0.0);
     masterPid.setP(P_GAIN);
     masterPid.setI(I_GAIN);
     masterPid.setFF(FF_GAIN);
     masterPid.setOutputRange(-1, 1);

     followPid = followMotor1.getPIDController();
     followPid.setD(0.0);
     followPid.setP(P_GAIN);
     followPid.setI(I_GAIN);
     followPid.setFF(FF_GAIN);
     followPid.setOutputRange(-1, 1);

  }

  public void setSpeed(double speed) {
    m_speed = speed;
  }

  public void shooterOn() {
    shooter = true;
  }

  public void shooterOff() {
    shooter = false;
  }

  public void intakeOn() {
    intake = true;
  }

public void intakeOff() {
    intake = false;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public Command ShooterCommand() {
    return runOnce(
        () -> {
          shooterOn();
    });
  }

  public Command shooterOffCommand() {
    return runOnce(
      () -> {
        shooterOff();
    });
  }
  public Command intakeCommand() {
    return runOnce(
      () -> {
        intakeOn();
      }
    );
  }

  public Command intakeOffCommand() {
    return runOnce(
      () -> {
        intakeOff();
      }
    );
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Shooter Target", MAX_SPEED);
    SmartDashboard.putNumber("Shooter Top", masterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Bottom", masterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Transfer", masterMotor.getEncoder().getVelocity());
    //SmartDashboard.putBoolean("Is at SetPoint", );
    if (shooter) {
      SmartDashboard.putNumber("Shooter Target", MAX_SPEED);
      masterPid.setReference(MAX_SPEED, ControlType.kVelocity);
      followPid.setReference(MAX_SPEED, ControlType.kVelocity);

      //followMotor1.set(10.0);
    } else {
      SmartDashboard.putNumber("Shooter Target", 0);
      masterMotor.set(0.0);
      followMotor1.set(0.0);
    }
    if (intake) {
      followMotor2.set(1);
    }else{
      followMotor2.set(0.0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
