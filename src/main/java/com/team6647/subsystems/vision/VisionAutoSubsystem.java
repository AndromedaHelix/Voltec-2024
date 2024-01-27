/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 25 01 2024
 */
package com.team6647.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.andromedalib.andromedaSwerve.utils.LocalADStarAK;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.team6647.util.Constants.DriveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionAutoSubsystem extends SubsystemBase {

  private static VisionAutoSubsystem instance;

  private final AndromedaSwerve andromedaSwerve;

  private VisionIO io;
  private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  /** Creates a new VisionSubsystem. */
  private VisionAutoSubsystem(VisionIO io, AndromedaSwerve swerve) {
    this.io = io;

    this.andromedaSwerve = swerve;
  }

  public static VisionAutoSubsystem getInstance(VisionIO io, AndromedaSwerve swerve) {
    if (instance == null) {
      instance = new VisionAutoSubsystem(io, swerve);
    }

    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    computeVisionMeasurements();
  }

  public void computeVisionMeasurements() {
    if (inputs.hasTarget) {
      andromedaSwerve.addVisionMeasurements(inputs.observedPose2d, inputs.timestampLatency);
    }
  }
}
