/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 25 02 2024
 */
package com.team6647.commands;

import com.team6647.subsystems.intake.IntakePivotSubsystem;
import com.team6647.subsystems.intake.IntakePivotSubsystem.IntakePivotState;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakePush extends Command {

  private IntakePivotSubsystem intakePivotSubsystem;

  public IntakePush(IntakePivotSubsystem intakePivotSubsystem) {
    this.intakePivotSubsystem = intakePivotSubsystem;

    addRequirements(intakePivotSubsystem);
  }

  @Override
  public void initialize() {
    intakePivotSubsystem.changeIntakePivotState(IntakePivotState.EXTENDED);
    intakePivotSubsystem.setPushingReference(IntakeConstants.pushingAcutatingPosition);
  }

  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
