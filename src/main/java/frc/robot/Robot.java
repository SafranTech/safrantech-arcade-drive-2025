package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Robot extends TimedRobot {

    private final int lw_left_pin = 1;
    private final int lw_right_pin = 2;
    private final int ENCODER_CHANNEL_A = 3;
    private final int ENCODER_CHANNEL_B = 4;

    // private final Spark m_shooting = new Spark(20);
    SparkMax m_shooting = new SparkMax(20, MotorType.kBrushless);
    Timer m_timer = new Timer();

    private final Joystick joystick = new Joystick(0);
    WPI_VictorSPX m_leftmotor = new WPI_VictorSPX(10);
    WPI_VictorSPX m_rightmotor = new WPI_VictorSPX(11);

    WPI_VictorSPX shooting_m = new WPI_VictorSPX(13);

    WPI_VictorSPX motor3 = new WPI_VictorSPX(12);
    WPI_VictorSPX motor5 = new WPI_VictorSPX(14);

    private final DigitalInput lw_right = new DigitalInput(lw_right_pin);
    private final DigitalInput lw_left = new DigitalInput(lw_left_pin);
    DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftmotor, m_rightmotor);
    private final Encoder encoder = new Encoder(ENCODER_CHANNEL_A, ENCODER_CHANNEL_B);

    boolean shooting_boost = false;
    double shooting_speed = 0.5;
    double encoderMax = 5000;

    @Override
    public void robotInit() {
        encoder.setDistancePerPulse(1);
        encoder.reset();
        m_leftmotor.setInverted(true);
    }

    public void stopMotors() {
        m_leftmotor.stopMotor();
        m_rightmotor.stopMotor();
        m_robotDrive.stopMotor();
        shooting_m.stopMotor();
        motor3.stopMotor();
        motor5.stopMotor();
        SmartDashboard.putString("Motor Durumu", "Kapalı");
    }

    public void joyst() {
        double forward = +joystick.getRawAxis(1);
        double turn = joystick.getRawAxis(4);
        m_robotDrive.arcadeDrive(turn, forward);
        SmartDashboard.putNumber("Joystick X", joystick.getX());
        SmartDashboard.putNumber("Joystick Y", joystick.getY());
    }

    public void matchtime() {
        final Double[] values = new Double[1];
        values[0] = DriverStation.getMatchTime();
        SmartDashboard.putNumber("Mac Suresi", DriverStation.getMatchTime());
    }

    @Override
    public void testPeriodic() {
        if(m_timer.get() < 5.0) {
            m_robotDrive.arcadeDrive(0.3, 0.3);
        } else if (m_timer.get() > 5.0 && m_timer.get() <= 15) {
            shooting_m.set(0.8);
            m_robotDrive.arcadeDrive(0.6, 0.6);
        } else if (m_timer.get() > 15) {
            stopMotors();
        }
    }

    public void matchnum() {
        final Double[] values = new Double[1];
        values[0] = (double) DriverStation.getMatchNumber();
        SmartDashboard.putNumber("Mac Sayisi", DriverStation.getMatchNumber());
    }

    public void shooting() {
        if (joystick.getRawButton(8)) {
            if (shooting_boost) {
                shooting_boost = false;
                shooting_speed = 0.5;
            } else {
                shooting_boost = true;
                shooting_speed = 0.8;
            }
        }

        if (joystick.getRawButton(4)) {
            m_shooting.set(0.2);
        } else {
            shooting_m.set(0);
            m_shooting.stopMotor();
        }
        if (joystick.getRawButton(1)) {
            m_shooting.set(-0.2);
        } else {
            m_shooting.stopMotor();
        }

        SmartDashboard.putNumber("Shooting Speed", shooting_speed);
        SmartDashboard.putBoolean("Shooting Mode", shooting_boost);
    }

    @Override
    public void teleopPeriodic() {
        boolean lw_rightStatus = lw_right.get();
        boolean lw_leftStatus = lw_left.get();

        int encoderCount = encoder.get(); // Encoder'den gelen pulse sayısı
        double distance = encoder.getDistance(); // Ölçülen mesafe

        boolean encoderLimit = encoderCount >= encoderMax;

        SmartDashboard.putBoolean("Limit Switch Right:", lw_rightStatus);
        SmartDashboard.putBoolean("Limit Switch Left:", lw_leftStatus);
        SmartDashboard.putNumber("Encoder Distance", distance);
        SmartDashboard.putNumber("Encoder Count", encoderCount);
        SmartDashboard.putBoolean("Encoder Limit Reached", encoderLimit);

        if (lw_rightStatus || lw_leftStatus || encoderLimit) {
            stopMotors();
            System.out.println("Motorlar Durduruldu: " + (lw_rightStatus || lw_leftStatus ? "Limit Switch" : "Encoder Limit"));
        } else {
            SmartDashboard.putString("Motor Durumu", "Açık");
            matchtime();
            matchnum();
            joyst();
            shooting();
        }

    }
}
