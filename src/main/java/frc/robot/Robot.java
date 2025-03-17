package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DriverStation;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.MjpegServer;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Robot extends TimedRobot {
    Timer m_timer = new Timer();

    private final Joystick joystick = new Joystick(0);
    private UsbCamera usbCamera;
    private MjpegServer mjpegServer;

    WPI_VictorSPX m_leftmotor = new WPI_VictorSPX(14);
    WPI_VictorSPX m_rightmotor = new WPI_VictorSPX(12);
    WPI_VictorSPX m_climb = new WPI_VictorSPX(11);
    WPI_VictorSPX s_climb = new WPI_VictorSPX(16);
    SparkMax m_shooting = new SparkMax(20, MotorType.kBrushless);
    DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftmotor, m_rightmotor);

    boolean climb_mode = false;
    boolean shooting_boost = false;
    double shooting_speed = 0.2;

    @Override
    public void robotInit() {
        m_rightmotor.setInverted(true);

        usbCamera = CameraServer.startAutomaticCapture();
        usbCamera.setResolution(640, 480);
        usbCamera.setFPS(30);

        mjpegServer = new MjpegServer("serve_USB Camera 0", 1181);
        mjpegServer.setSource(usbCamera);
    }

    public void stopMotors() {
        m_shooting.stopMotor();
        m_robotDrive.stopMotor();
        m_climb.set(0);
        s_climb.set(0);
        SmartDashboard.putBoolean("Motor Durumu", false);
    }

    public void joyst() {
        double forward = +joystick.getRawAxis(4);
        double turn = joystick.getRawAxis(1);
        m_robotDrive.arcadeDrive((turn * 0.9), (forward * 0.9));
        SmartDashboard.putNumber("Joystick X", joystick.getX());
        SmartDashboard.putNumber("Joystick Y", joystick.getY());
    }

    public void matchtime() {
        final Double[] values = new Double[1];
        values[0] = DriverStation.getMatchTime();
        SmartDashboard.putNumber("Mac Suresi", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Mac Saniyesi", m_timer.get());
    }

    @Override
    public void testPeriodic() {
        m_climb.set(-0.5);
    }

    @Override
    public void autonomousInit() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void autonomousPeriodic() {
        if (m_timer.get() < 5.0) {
            m_robotDrive.arcadeDrive(-0.55,0);
            
        } else if (m_timer.get() > 6.0 && m_timer.get() <= 12.0) {
             m_shooting.set(-0.15);
            // m_climb.set(-0.5);
        } else if (m_timer.get() > 12.0 && m_timer.get() <= 15.0) {
            stopMotors();
        } else {
            m_robotDrive.stopMotor();
        }
        SmartDashboard.putNumber("Mac Saniyesi", m_timer.get());
    }

    public void matchnum() {
        final Double[] values = new Double[1];
        values[0] = (double) DriverStation.getMatchNumber();
        SmartDashboard.putNumber("Mac Sayisi", DriverStation.getMatchNumber());

    }

    public void climb() {
        if (joystick.getRawButtonReleased(7)) {
            if (climb_mode) {
                climb_mode = false;
            } else {
                climb_mode = true;
            }
        }
        if (climb_mode) {
            if (joystick.getRawButton(4)) {
                m_climb.set(-1);
            } else {
                m_climb.set(0);
            }
            if (joystick.getRawButton(1)) {
                m_climb.set(0.8);
            }
        }
    }

    public void shooting() {
        if (!climb_mode) {
            if (joystick.getRawButtonReleased(8)) {
                if (shooting_boost) {
                    shooting_boost = false;
                    shooting_speed = 0.2;
                } else {
                    shooting_boost = true;
                    shooting_speed = 0.3;
                }
            }
            if (joystick.getRawButton(1)) {
                s_climb.set(0.8);
            } else {
                s_climb.set(0);
            }
            if (joystick.getRawButton(4)) {
                s_climb.set(-1);
            }
            if (joystick.getRawButton(2)) {
                m_shooting.set(shooting_speed);
            } else {
                m_shooting.stopMotor();
            }
            if (joystick.getRawButton(3)) {
                m_shooting.set(-0.1);
            }
            SmartDashboard.putBoolean("Shooting", shooting_boost);
        }
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putBoolean("Motor Durumu", true);
        SmartDashboard.putBoolean("Tirmanma Modu", climb_mode);
        matchtime();
        matchnum();
        joyst();
        shooting();
        climb();
    }
}
