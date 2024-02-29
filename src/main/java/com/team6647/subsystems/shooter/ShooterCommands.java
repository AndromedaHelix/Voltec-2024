/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 13 02 2024
 */

package com.team6647.subsystems.shooter;

import com.team6647.RobotContainer;
import com.team6647.commands.FlywheelTarget;
import com.team6647.commands.ShooterPivotTarget;
import com.team6647.commands.ShooterRollerStartEnd;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.util.Constants.RobotConstants.RollerState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterCommands {
    private static ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
    private static ShooterPivotSubsystem pivotSubsystem = RobotContainer.shooterPivotSubsystem;
    private static ShooterRollerSubsystem rollerSubsystem = RobotContainer.shooterRollerSubsystem;

    public static final Command getShooterIntakingCommand() {

        Debouncer debounce = new Debouncer(0.34, DebounceType.kRising);

        return Commands.deadline(
                Commands.waitUntil(() -> debounce.calculate(rollerSubsystem.getAmps() > 3)),
                new ShooterPivotTarget(pivotSubsystem, ShooterPivotState.INDEXING),
                new ShooterRollerStartEnd(rollerSubsystem, RollerState.IDLE, RollerState.STOPPED),
                new FlywheelTarget(shooterSubsystem, FlywheelState.STOPPED));

    }

}
