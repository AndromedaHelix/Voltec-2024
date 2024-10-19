/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 13 02 2024
 */

package com.team6647.subsystems.shooter;

import com.team6647.RobotContainer;
import com.team6647.commands.FlywheelTarget;
import com.team6647.commands.ShooterPivotTarget;
import com.team6647.commands.ShooterRollerTarget;
import com.team6647.subsystems.SuperStructure;
import com.team6647.subsystems.SuperStructure.SuperStructureState;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem.ShooterFeederState;
import com.team6647.util.Constants.OperatorConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShooterCommands {
    private static ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
    private static ShooterPivotSubsystem pivotSubsystem = RobotContainer.shooterPivotSubsystem;
    private static ShooterRollerSubsystem rollerSubsystem = RobotContainer.shooterRollerSubsystem;

    public static final Command getShooterIntakingCommand() {
        return Commands.deadline(
                Commands.waitUntil(() -> !shooterSubsystem.getBeamBrake()),
                new ShooterPivotTarget(pivotSubsystem, ShooterPivotState.INDEXING),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING)
                //new FlywheelTarget(shooterSubsystem, FlywheelState.STOPPED)
                ).andThen(new WaitCommand(0));
                
                
    }
}

