// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.utilities.MovingAverage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private double m_speed = 0.0;
  private boolean shoot = false;
  private CANSparkMax masterMotor;
  private CANSparkMax followMotor1;
  private CANSparkMax followMotor2;

  private final boolean useAutoStop = false;
  private MovingAverage motorCurrent = new MovingAverage(100);
  private Timer timer = new Timer();
  private boolean auto_stop_active = false;
  private final double CURRENT_SURGE_RATIO = 2.0;
  private final double AUTO_STOP_TIMEOUT = 0.5;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    m_speed = 0.0;
    shoot = false;

    masterMotor  = new CANSparkMax(Constants.CAN.MOTOR1, MotorType.kBrushless);
    followMotor1 = new CANSparkMax(Constants.CAN.MOTOR2, MotorType.kBrushless);
    followMotor2 = new CANSparkMax(Constants.CAN.MOTOR3, MotorType.kBrushless);

    masterMotor.setIdleMode(IdleMode.kCoast);
    followMotor1.setIdleMode(IdleMode.kCoast);
    followMotor2.setIdleMode(IdleMode.kCoast);

    masterMotor.setInverted(false);
    followMotor1.setInverted(true);
    followMotor2.setInverted(true);
  }

  public void setSpeed(double speed) {
    m_speed = speed;
  }

  public void shooterOn() {
    shoot = true;
  }

  public void shooterOff() {
    shoot = false;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command ShooterOnCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          shooterOn();
        });
  }

  public Command ShooterOffCommand() {
    // Inline construction of command goes here.
    return runOnce(
        () -> {
          shooterOff();
        });
  }

  @Override
  public void periodic() {
    if (shoot) {
      masterMotor.set(m_speed);
      followMotor1.set(m_speed);
      followMotor2.set(m_speed);
    } else {
      masterMotor.set(0.0);
      followMotor1.set(0.0);
      followMotor2.set(0.0);
    }
    if (useAutoStop) {
      double curr = masterMotor.getOutputCurrent();
      motorCurrent.add(curr);
      if (shoot && (!auto_stop_active) && (curr > CURRENT_SURGE_RATIO*motorCurrent.average())) {
        timer.stop();
        timer.reset();
        timer.start();
        auto_stop_active = true;
      }
      if (shoot && auto_stop_active && (timer.get() > AUTO_STOP_TIMEOUT)) {
        shooterOff();
        auto_stop_active = false;
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
