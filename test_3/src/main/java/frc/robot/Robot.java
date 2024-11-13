/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Robot extends TimedRobot {
  private CANSparkMax armMotor;
  private static final int armMotor_ID = 5;
  private WPI_VictorSPX rightSlave = new WPI_VictorSPX(1); // right back
  private WPI_VictorSPX leftMaster = new WPI_VictorSPX(2); // left front
  private WPI_VictorSPX leftSlave = new WPI_VictorSPX(3); // left back
  private WPI_VictorSPX rightMaster = new WPI_VictorSPX(4); // right front
  private Joystick Joy1 = new Joystick(0);
  private Joystick Joy2 = new Joystick(1);
  private RelativeEncoder armEncoder;
  Encoder left_encoder = new Encoder(0, 1);
  Encoder right_encoder = new Encoder(2, 3);
  
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left Value", left_encoder.getDistance());
    SmartDashboard.putNumber("Left Rate", left_encoder.getRate());
    SmartDashboard.putNumber("Right Value", right_encoder.getDistance());
    SmartDashboard.putNumber("Right Rate", right_encoder.get());

    SmartDashboard.putNumber("Joystick speed", -Joy1.getRawAxis(1));
    SmartDashboard.putNumber("Joystick turn", Joy1.getRawAxis(3));
    SmartDashboard.putNumber("arm joy", -Joy2.getRawAxis(1));
  }

  @Override
  public void robotInit() {
    armMotor = new CANSparkMax(armMotor_ID, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armEncoder = armMotor.getEncoder();
    //left reverse, right normal
    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
  }

  @Override
  public void teleopInit() {
    enableMotors(true);
  }

  @Override
  public void disabledInit() {
    enableMotors(false);
  }

  private void reset_fork(){
    if (armEncoder.getPosition() > 20){
      armMotor.set(-0.5);
    }else {
      armMotor.set(0);
    }
  }

  @Override
  public void teleopPeriodic() {
    double power = -Joy1.getRawAxis(1)* 0.4; // remember: negative sign
    double turn = Joy1.getRawAxis(3)* 0.25;
    double armPower = -Joy2.getRawAxis(1); // remember negative sign

    double left = power + turn;
    double right = power - turn;
    if (armEncoder.getPosition() > -100){
      armMotor.set(armPower);
    } else {
      armMotor.set(0);
    }
    if (Joy2.getRawButton(1)){
      reset_fork();
    }
    leftMaster.set(left*1.1);
    rightMaster.set(right);
    SmartDashboard.putNumber("Encoder arm position", armEncoder.getPosition());
    SmartDashboard.putNumber("Arm motor velocity",armEncoder.getVelocity());
  }

  private void enableMotors(boolean on) {
    NeutralMode mode;
    if (on) {
      mode = NeutralMode.Brake;
    } else {
      mode = NeutralMode.Coast;
    }
    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    leftSlave.setNeutralMode(mode);
    rightSlave.setNeutralMode(mode);

  }
}
