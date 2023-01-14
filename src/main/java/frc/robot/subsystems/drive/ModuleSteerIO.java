package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import lib.iotemplates.ClosedLoopIO;
import lib.talonconfiguration.BaseTalonFXConfiguration;
import lib.util.TunableNumber;

public class ModuleSteerIO implements ClosedLoopIO {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table;

    WPI_TalonFX steerMotor;
    CANCoder encoder;
    Rotation2d offset;
    double tempOffset;
    double positionSetPointRad;
    TunableNumber kP = new TunableNumber("KPTuningController", 0);
    TunableNumber kI = new TunableNumber("KITuningController", 0);
    TunableNumber kD = new TunableNumber("KDTuningController", 0);

    private PIDController m_canCoderSteeringPIDController = new PIDController(
            kP.get(), 
            kI.get(),
            kD.get());

    public ModuleSteerIO(int motorPort, int encoderPort, double encoderOffset, String corners) {
        table = inst.getTable(corners);
        steerMotor = new WPI_TalonFX(motorPort, "GertrudeGreyser");
        steerMotor.configAllSettings(new BaseTalonFXConfiguration());
        steerMotor.setNeutralMode(NeutralMode.Brake);
        // set status frame period of steer motor
        steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        m_canCoderSteeringPIDController.enableContinuousInput(-Math.PI, Math.PI);
        encoder = new CANCoder(encoderPort, "GertrudeGreyser");
        offset = new Rotation2d(encoderOffset);

        table.getEntry("KpTurning Controller").setDouble(1.1);

        //Configure Max & Min outputs of Falcon
        steerMotor.configNominalOutputForward(0);
        steerMotor.configNominalOutputReverse(0);
        steerMotor.configPeakOutputForward(1);
        steerMotor.configPeakOutputReverse(-1);
        steerMotor.configAllowableClosedloopError(0, Constants.ModuleConstants.MaxAllowableError);

        //Configure PID Values for built in PID on falcon
        steerMotor.config_kP(0, table.getEntry("KpTurning Controller").getDouble(0)); // Constants.ModuleConstants.kPModuleTurningController);
        steerMotor.config_kI(0, Constants.ModuleConstants.kIModuleTurningController);
        steerMotor.config_kD(0, Constants.ModuleConstants.kDModuleTurningController);
    }

    public void updateInputs(ClosedLoopIOInputs inputs) {
        steerMotor.config_kP(0, Constants.ModuleConstants.kPModuleTurningController);
        //table.getEntry("Rot Pos Act").setDouble(getPosition().getRadians());
        inputs.positionRad = getPosition().getRadians();
        inputs.toLog(table);

        m_canCoderSteeringPIDController.setP(Constants.ModuleConstants.kPModuleTurningController);
        m_canCoderSteeringPIDController.setI(Constants.ModuleConstants.kIModuleTurningController);
        m_canCoderSteeringPIDController.setD(Constants.ModuleConstants.kDModuleTurningController);


    }

    //Get position using encoder on Falcon
    private Rotation2d getPosition() {
        return Rotation2d.fromDegrees(steerMotor.getSelectedSensorPosition());// *
                // Constants.ModuleConstants.kSteerEncoderTicksPerRevolution);
    }

    //Get position using CanCoder
    private Rotation2d getCanCoderPosition() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition()).minus(offset);
    }

    //Calculates temporary offset as: tempOffset =  Falcon Encoder Value - (Cancoder value - Offset)
    public void calculateOffset() {
        tempOffset = steerMotor.getSelectedSensorPosition()/
                Constants.ModuleConstants.kSteerEncoderTicksPerRevolution - getCanCoderPosition().getRadians();
    }

    //Sets position using built in PID on motor
    public void setPosition(Rotation2d positionRad) {
        steerMotor.set(ControlMode.Position, (tempOffset + positionRad.getRadians())* (Constants.ModuleConstants.kSteerEncoderTicksPerRevolution)/(2*Math.PI));
        // table.getEntry("Rot Setpoint").setDouble(positionRad.getRadians());
    }

    //Sets position using CanCoderPID
    public void setPositionCanCoder(Rotation2d positionRad) {
        positionSetPointRad = positionRad.getRadians();
        final double turnOutput = m_canCoderSteeringPIDController.calculate(
                getCanCoderPosition().getRadians(),
                positionRad.getRadians()
        );

        steerMotor.setVoltage(turnOutput);
    }
}
