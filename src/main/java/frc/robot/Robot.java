// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.FileNotFoundException;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Tracking;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;



public class Robot extends TimedRobot {
  // Define objects and variables
  private Joystick LeftStick;
  private Joystick RightStick;

  private AnalogEncoder AnalogData;

  private double ControllerOneX;
  private double ControllerOneY;
  private double ControllerOneTwist;
  private double[] MotorCurrents;
  private String[] AutoNames;
  private String PrevAuto;

  public SwerveDrive SwerveDrive;
  public Tracking Tracking;
  public Autonomous Autonomous;

  public NetworkTableInstance Inst;
  public NetworkTable DriverStation;
  public NetworkTableEntry GyroAng;
  public SendableChooser<String> AutoChooser;
  private ShuffleboardTab LiveWindow;
  private SimpleWidget FFGain;
  private SimpleWidget PGain;
  private SimpleWidget IGain;
  private SimpleWidget DGain;

  private CANSparkMax ArmAngle;
  private SparkMaxPIDController ArmAnglePIDController;
  private CANSparkMax ArmExtend;
  private SparkMaxPIDController ArmExtendPIDController;

  private Compressor Pump;
  private DoubleSolenoid Grabber;
  
  private XboxController ControllerOne = new XboxController(0);
  private XboxController ControllerTwo = new XboxController(1);

  private VictorSPX armMotor = new VictorSPX(9); // 0 is the RIO PWM port this is connected to

  // Claw open and close.
  public DoubleSolenoid doubleSolenoidOne = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  // Level 1 Solenoid
  public DoubleSolenoid doubleSolenoidTwo = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  // Level 2 Solenoid
  public DoubleSolenoid doubleSolenoidThree = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);




  @Override
  public void robotInit() {

    AnalogData = new AnalogEncoder(0);

    CameraServer.startAutomaticCapture();

    Inst = NetworkTableInstance.getDefault();

    AutoChooser = new SendableChooser<String>();
    AutoNames = Filesystem.getDeployDirectory().toPath().resolve("output").toFile().list();
    for (Integer Index = 0; Index <= AutoNames.length - 1; Index++) {
      AutoChooser.addOption(AutoNames[Index], AutoNames[Index]);
    }
    AutoChooser.setDefaultOption("default", "ATest1");

    SmartDashboard.putData("AutoChooser", AutoChooser);

    LiveWindow = Shuffleboard.getTab("LiveWindow");

    FFGain = LiveWindow.add("FFGain", 0.000175);
    PGain = LiveWindow.add("PGain", 0.000005);
    IGain = LiveWindow.add("IGain", 0.0000004);
    DGain = LiveWindow.add("DGain", 0.0);

    // Assign joysticks to the "LeftStick" and "RightStick" objects
    LeftStick = new Joystick(0);
    RightStick = new Joystick(1);

    ControllerOne = new XboxController(0);
    ControllerTwo = new XboxController(1);
    

    ArmAngle = new CANSparkMax(10, MotorType.kBrushless);
    ArmAnglePIDController = ArmAngle.getPIDController();
    ArmExtend = new CANSparkMax(11, MotorType.kBrushless);
    ArmExtendPIDController = ArmExtend.getPIDController();

    ArmAnglePIDController.setFeedbackDevice(ArmAngle.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    ArmAnglePIDController.setOutputRange(-1, 1);
    ArmAnglePIDController.setFF(0);
    ArmAnglePIDController.setP(.05);
    ArmAnglePIDController.setI(0);
    ArmAnglePIDController.setD(0);
    ArmExtendPIDController.setFeedbackDevice(ArmExtend.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42));
    ArmExtendPIDController.setOutputRange(-1, 1);
    ArmAnglePIDController.setFF(0);
    ArmAnglePIDController.setP(.06);
    ArmAnglePIDController.setI(0);
    ArmAnglePIDController.setD(0);

    Pump = new Compressor(0, PneumaticsModuleType.CTREPCM);
    // Grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    // Grabber.set(Value.kForward);

    // Instantiate an object for each class
    SwerveDrive = new SwerveDrive();
    Tracking = new Tracking(SwerveDrive);
    Autonomous = new Autonomous(SwerveDrive, Tracking);

    SwerveDrive.GyroRotation2d = SwerveDrive.Gyro.getRotation2d();

    // Call SwerveDrive methods, their descriptions are in the SwerveDrive.java file
    // SwerveDrive.initMotorControllers(1, 2, 7, 8, 5, 6, 3, 4);
    SwerveDrive.initMotorControllers(1, 2, 7, 8, 5, 6, 3, 4);
    SwerveDrive.setPID(0.000175, 0.0000007, 0.0000004, 0.0, 8.1, 0.01, 0.01);
    // SwerveDrive.setPID(0.000175, 0.000001, 0.0000004, 0.0, 8.0, 0.01, 0.01);
    SwerveDrive.initKinematicsAndOdometry();
    PrevAuto = AutoChooser.getSelected();
    Autonomous.AutoFile = AutoChooser.getSelected();
    if (AutoChooser.getSelected() != null) {
      System.out.println(Autonomous.AutoFile);
    try {
        Autonomous.initTrajectory();
      } catch (FileNotFoundException e) {
        System.out.println("AUTO NOT FOUND");
      }
    }
    Pump.enableDigital();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (PrevAuto != AutoChooser.getSelected()) {
      Autonomous.AutoFile = AutoChooser.getSelected();
      System.out.println(Autonomous.AutoFile);
      try {
        Autonomous.initTrajectory();
      } catch (FileNotFoundException e) {
        System.out.println("AUTO NOT FOUND");
      }
      PrevAuto = AutoChooser.getSelected();
    }
    // SwerveDrive.setPID(FFGain.getEntry().getDouble(0), PGain.getEntry().getDouble(0), IGain.getEntry().getDouble(0), DGain.getEntry().getDouble(0), 8.0, 0.01, 0.01);
  }
 
  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    SwerveDrive.GyroRotation2d = SwerveDrive.Gyro.getRotation2d().unaryMinus();

    SwerveDrive.FrontLeft.DriveEncoder.getVelocity();

    // Assign stick inputs to variables, to prevent discrepancies
    ControllerOneX = ControllerOne.getLeftX();
    ControllerOneY = ControllerOne.getLeftY();
    ControllerOneTwist = ControllerOne.getRightX();

    // Create deadzones on the joysticks, to prevent stick drift
    if (Math.abs(ControllerOneX) < 0.10) {
      ControllerOneX = 0.0;
    }
    if (Math.abs(ControllerOneY) < 0.10) {
      ControllerOneY = 0.0;
    }
    if (Math.abs(ControllerOneTwist) < 0.10) {
      ControllerOneTwist = 0.0;
    }

    if (LeftStick.getRawButton(2)) {
      Tracking.centerOnPole();
    }
    else {
      // Call swerveDrive() method, to do all the math and outputs for swerve drive
      SwerveDrive.swerveDrive(Math.pow(ControllerOneX, 3) * 3, (Math.pow(ControllerOneY, 3) * -3), (Math.pow(ControllerOneTwist, 3) * 3.0), (1 - ((ControllerOne.getLeftTriggerAxis() + 1) / 2)), (1 - ((ControllerOne.getRightTriggerAxis() + 1) / 2)));
      SwerveDrive.setVariablesAndOptimize();
      SwerveDrive.setSwerveOutputs();
    }

    SmartDashboard.putNumber("Gyro", SwerveDrive.GyroRotation2d.getDegrees());

    // Controller One
    SmartDashboard.putNumber("Left Y Input", ControllerOne.getLeftY());
    SmartDashboard.putNumber("Right X Input", ControllerOne.getRightX());

    SmartDashboard.putNumber("FrontLeft Absolute Output", SwerveDrive.FrontLeft.Steer.getSelectedSensorPosition());
    SmartDashboard.putNumber("FrontRight Absolute Output", SwerveDrive.FrontRight.Steer.getSelectedSensorPosition());
    SmartDashboard.putNumber("BackLeft Absolute Output", SwerveDrive.BackLeft.Steer.getSelectedSensorPosition());
    SmartDashboard.putNumber("BackRight Absolute Output", SwerveDrive.BackRight.Steer.getSelectedSensorPosition());


    SmartDashboard.putNumber("Rio Encoder Value", AnalogData.getAbsolutePosition());
    System.out.println(SwerveDrive.FrontRight.DrivePIDController.getP());

    MotorCurrents = new double[] {SwerveDrive.FrontLeft.Drive.getOutputCurrent(), SwerveDrive.FrontRight.Drive.getOutputCurrent(), SwerveDrive.BackLeft.Drive.getOutputCurrent(), SwerveDrive.BackRight.Drive.getOutputCurrent()};
    SmartDashboard.putNumberArray("RobotDrive Motors", MotorCurrents);
  
    // Claw open / close if statements.
    if (ControllerTwo.getLeftBumperPressed()) {
      doubleSolenoidOne.set(kForward);
    }
    if (ControllerTwo.getRightBumperPressed()) {
      doubleSolenoidOne.set(kReverse);
    }
  
    // Level 1 & 2 Buttons
    if (ControllerTwo.getYButtonPressed()) {
      doubleSolenoidTwo.set(kForward);
      doubleSolenoidThree.set(kForward);
      
    }
    if (ControllerTwo.getAButtonPressed()) {
      doubleSolenoidTwo.set(kReverse);
      doubleSolenoidThree.set(kReverse);
    }

    if (ControllerTwo.getBButtonPressed()) {
      doubleSolenoidTwo.set(kReverse);
      doubleSolenoidThree.set(kForward);
    }
    armMotor.set(VictorSPXControlMode.PercentOutput, ControllerTwo.getLeftX()*.375);
  }

  //Autonomous right away
  @Override
  public void autonomousInit(){
  }

  //Autonomous repeat
  @Override
  public void autonomousPeriodic(){
    SwerveDrive.GyroRotation2d = SwerveDrive.Gyro.getRotation2d(); 
    Autonomous.runAutonomous();
  }
}