/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 3 02 2024
 */

package com.team6647.subsystems.intake;

import com.team6647.RobotContainer;
import com.team6647.commands.InitIntake;
import com.team6647.commands.IntakeExtend;
import com.team6647.commands.IntakeHome;
import com.team6647.commands.IntakeRollerTarget;
import com.team6647.subsystems.intake.pivot.IntakePivotSubsystem;
import com.team6647.subsystems.intake.roller.IntakeSubsystem;
import com.team6647.subsystems.intake.roller.IntakeSubsystem.IntakeRollerState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {

        private static IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
        private static IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;

        public static final Command getFullIntakeCommand() {

                Debouncer debounce = new Debouncer(0.1);

                return Commands.sequence(
                                new IntakeExtend().andThen(new InitIntake(intakePivotSubsystem)),
                                new IntakeRollerTarget(intakeSubsystem, IntakeRollerState.INTAKING),
                                Commands.waitUntil(() -> debounce.calculate(!intakeSubsystem.getBeamBrake())),
                                new IntakeRollerTarget(
                                                intakeSubsystem,
                                                IntakeRollerState.STOPPED),
                                new IntakeHome(intakePivotSubsystem),
                                new IntakeRollerTarget(
                                                intakeSubsystem,
                                                IntakeRollerState.INTAKING));
        }

        public static final Command getIntakeCommand() {
                Debouncer debounce = new Debouncer(0.1);

                return Commands.sequence(
                                new IntakeExtend().andThen(new InitIntake(intakePivotSubsystem)),
                                new IntakeRollerTarget(intakeSubsystem, IntakeRollerState.INTAKING),
                                Commands.waitUntil(() -> debounce.calculate(!intakeSubsystem.getBeamBrake())),
                                new IntakeRollerTarget(
                                                intakeSubsystem,
                                                IntakeRollerState.STOPPED),
                                new IntakeHome(intakePivotSubsystem));
        }
}
