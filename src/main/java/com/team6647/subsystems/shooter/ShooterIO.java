/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public double leftMotorVelocity = 0.0;
        public double rightMotorVelocity = 0.0;

        public boolean beamBrake = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setShooterVelocity(double velocity) {
    }
}
