package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class NewSwerveModule {
    private static final double kWheelRadius = 0.0636;

    private final double driveRatio;
    private final double azimuthRatio;

    private final TalonFX drive;
    private final TalonFX azimuthTalon;
    private CANSparkMax azimuthSpark;
    private SparkAbsoluteEncoder azimuthEncoder;

    private boolean azimuthSparkActive;

    public NewSwerveModule(double driveRatio, double azimuthRatio, int moduleNumber, TalonFXConfiguration configs) {
        this.driveRatio = driveRatio;
        this.azimuthRatio = azimuthRatio;

        this.drive = new TalonFX(10 + moduleNumber, "canivore");
        this.azimuthTalon = new TalonFX(20 + moduleNumber, "canivore");

        azimuthSparkActive = false;

        try {
            this.azimuthTalon.get();
        } catch (Throwable err) {
            this.azimuthSpark = new CANSparkMax(20 + moduleNumber, MotorType.kBrushless);
            this.azimuthEncoder = azimuthSpark.getAbsoluteEncoder();
            azimuthSparkActive = true;
        }

        this.drive.getConfigurator().apply(configs);
    }



    public SwerveModulePosition getPosition() {
        Rotation2d rotation;
        if (azimuthSparkActive) {
            rotation = new Rotation2d(azimuthEncoder.getPosition() / azimuthRatio * 2 * Math.PI);
        } else {
            rotation = new Rotation2d((azimuthTalon.getRotorPosition().getValueAsDouble() / azimuthRatio) * 2 * Math.PI);
        }
        return new SwerveModulePosition(
        (drive.getRotorPosition().getValueAsDouble() / driveRatio) * (2 * Math.PI * kWheelRadius),
        rotation);
    }

    public void updateOutputs(SwerveModuleState moduleState) {
        
    }

    
}
