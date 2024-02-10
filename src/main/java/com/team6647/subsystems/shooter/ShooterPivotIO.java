/**
 * Written by Juan Pablo Gutiérrez  
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {

    @AutoLog
    public static class ShooterPivotIOInputs {
        public double shooterAbsoluteEncoderPosition = 0.0;

        public boolean shooterBeamBrake = false;
    }

    public default void updateInputs(ShooterPivotIOInputs inputs) {
    }

    public default void setShooterVoltage(double volts) {
    }

}
