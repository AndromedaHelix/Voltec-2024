/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterPivotIOSparkMax implements ShooterPivotIO {

    private SuperSparkMax shooterPivotLeftMotor = new SuperSparkMax(
            ShooterConstants.shooterPivotMotorID,
            GlobalIdleMode.brake, ShooterConstants.shooterPivotMotorInverted,
            ShooterConstants.shooterMotorCurrentLimit,
            ShooterConstants.armEncoderPositionConversionFactor,
            ShooterConstants.armEncoderZeroOffset,
            ShooterConstants.armEncoderInverted);

    private static AbsoluteEncoder pivotEncoder;

    private static DigitalInput shooterBeamBrake = new DigitalInput(ShooterConstants.shooterBeamBrakeChannel);

    public ShooterPivotIOSparkMax() {
        pivotEncoder = shooterPivotLeftMotor.getAbsoluteEncoder(Type.kDutyCycle);
    }

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        inputs.shooterAbsoluteEncoderPosition = pivotEncoder.getPosition();

        inputs.shooterBeamBrake = shooterBeamBrake.get();
    }

    @Override
    public void setShooterVoltage(double volts) {
        shooterPivotLeftMotor.setVoltage(volts);
    }
}
