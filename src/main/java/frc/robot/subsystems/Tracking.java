// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tracking extends SubsystemBase {
  public NetworkTableInstance Inst;
  public NetworkTable ArmLimelight;
  public NetworkTableEntry ArmHasTarget;
  public NetworkTableEntry ArmTargetOffsetH;
  public NetworkTableEntry ArmTargetOffsetV;
  public NetworkTableEntry ArmTargetArea;
  public NetworkTableEntry ArmTargetSkew;
  public NetworkTable IntakeLimelight;
  public NetworkTableEntry IntakeHasTarget;
  public NetworkTableEntry IntakeTargetOffsetH;
  public NetworkTableEntry IntakeTargetOffsetV;
  public NetworkTableEntry IntakeTargetArea;
  public NetworkTableEntry IntakeTargetSkew;
  public NetworkTableEntry IntakePipeline;
  private SwerveDrive Swerve;
  private Constraints PIDConstraints;
  private ProfiledPIDController PID;

  public Tracking(SwerveDrive SwerveDrive) {
    Swerve = SwerveDrive;
    Inst = NetworkTableInstance.getDefault();
    ArmLimelight = Inst.getTable("limelight-arm");
    ArmHasTarget = ArmLimelight.getEntry("tv");
    ArmTargetOffsetH = ArmLimelight.getEntry("tx");
    ArmTargetOffsetV = ArmLimelight.getEntry("ty");
    ArmTargetArea = ArmLimelight.getEntry("ta");
    ArmTargetSkew = ArmLimelight.getEntry("ts");

    IntakeLimelight = Inst.getTable("limelight-intake");
    IntakeHasTarget = IntakeLimelight.getEntry("tv");
    IntakeTargetOffsetH = IntakeLimelight.getEntry("tx");
    IntakeTargetOffsetV = IntakeLimelight.getEntry("ty");
    IntakeTargetArea = IntakeLimelight.getEntry("ta");
    IntakeTargetSkew = IntakeLimelight.getEntry("ts");
    IntakePipeline = IntakeLimelight.getEntry("pipeline");
    PIDConstraints = new Constraints(1, .5);
    PID = new ProfiledPIDController(1, 0, 0, PIDConstraints);
  }

  public void centerOnPole() {
    Swerve.swerveDrive(-PID.calculate(ArmTargetOffsetH.getDouble(0), 0.0), 0.0, 0.0, .1, 0.0);
    Swerve.setVariablesAndOptimize();
    Swerve.setSwerveOutputs();
  }
  public void centerOnCone() {
    IntakePipeline.setValue(0);
    Swerve.swerveDrive(-PID.calculate(IntakeTargetOffsetH.getDouble(0), 0.0), 0.0, 0.0, .1, 0.0);
    Swerve.setVariablesAndOptimize();
    Swerve.setSwerveOutputs();
  }
  public void centerOnCube() {
    IntakePipeline.setValue(1);
    Swerve.swerveDrive(-PID.calculate(IntakeTargetOffsetH.getDouble(0), 0.0), 0.0, 0.0, .1, 0.0);
    Swerve.setVariablesAndOptimize();
    Swerve.setSwerveOutputs();
  }
}
