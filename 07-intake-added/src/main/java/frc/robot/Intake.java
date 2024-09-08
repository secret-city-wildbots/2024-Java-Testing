package frc.robot;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

public class Intake {
    public boolean enabled = false;
    private double outerIntakePower;
    private double innerIntakePower;
    private double indexerIntakePower;
    private boolean bbBroken = false;

    public double innerTemp = -1;
    public double frontTemp = -1;
    public double backTemp = -1;

    private final TalonFX inner = new TalonFX(19, "rio");
    private final TalonFX front = new TalonFX(24, "rio");
    private final TalonFX back = new TalonFX(25, "rio");
    private final CANSparkMax indexer = new CANSparkMax(18, MotorType.kBrushless);
    private final SparkLimitSwitch beamBreak = indexer.getForwardLimitSwitch(Type.kNormallyClosed);

    public Intake(
        double innerPower, 
        double outerPower, 
        double indexerPower) {
        outerIntakePower = outerPower;
        innerIntakePower = innerPower;
        indexerIntakePower = indexerPower;
    }

    public void updateSensors() {
        bbBroken = beamBreak.isLimitSwitchEnabled();
        innerTemp = inner.getDeviceTemp().getValueAsDouble();
        frontTemp = front.getDeviceTemp().getValueAsDouble();
        backTemp = back.getDeviceTemp().getValueAsDouble();
    }

    public void toggle() {
        if (enabled) {
            enabled = false;
        } else if (!bbBroken) {
            if (Wrist.stowed && Elevator.stowed) {
                enabled = true;
            } else {
                enabled = false;
            }
        } else {
            enabled = false;
        }
    }

    public void updateOutputs() {
        inner.set((enabled) ? innerIntakePower : 0);
        front.set((enabled) ? outerIntakePower : 0);
        back.set((enabled) ? outerIntakePower : 0);
        indexer.set((enabled && (!bbBroken)) ? indexerIntakePower : 0);
    }
}
