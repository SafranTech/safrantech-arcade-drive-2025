package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Robot extends TimedRobot {


    // Limit Switch DIO Pin
    private final int LIMIT_SWITCH_PIN = 1; // Limit switch DIO1 pinine bağlı

    // Encoder DIO Pins
    private final int ENCODER_CHANNEL_A = 2; // Encoder'in Channel A'sı
    private final int ENCODER_CHANNEL_B = 3; // Encoder'in Channel B'si

    // Motor Kontrolcüleri, Joystick, Limit Switch ve Encoder
    // private final Spark motor1 = new Spark(PWM_CHANNEL_1);
    WPI_VictorSPX motor1 = new WPI_VictorSPX(10);
    WPI_VictorSPX motor2 = new WPI_VictorSPX(11);
    WPI_VictorSPX motor3 = new WPI_VictorSPX(12);
    WPI_VictorSPX motor4 = new WPI_VictorSPX(13);
    WPI_VictorSPX motor5 = new WPI_VictorSPX(14);
    private final Joystick joystick = new Joystick(0);
    private final DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_PIN);
      DifferentialDrive m_robotDrive = new DifferentialDrive(motor1, motor2);
    private final Encoder encoder = new Encoder(ENCODER_CHANNEL_A, ENCODER_CHANNEL_B);

    // Önceki durum değişkeni
    private boolean prevLimitSwitchState = false;

    @Override
    public void robotInit() {
        encoder.setDistancePerPulse(1);
        encoder.reset();
    }

    public void joyst() {
        double forward = +joystick.getRawAxis(1);
        double turn = joystick.getRawAxis(4);
        m_robotDrive.arcadeDrive(turn, forward);
        SmartDashboard.putNumber("Joystick X", joystick.getX());
        SmartDashboard.putNumber("Joystick Y", joystick.getY());
      }

    @Override
    public void testPeriodic() {
        motor1.set(0.5);
        motor2.set(0.5);
        motor3.set(0.5);
        motor4.set(0.5);
        motor5.set(0.5);
    }

    @Override
    public void teleopPeriodic() {
        // Limit switch durumunu al
        
        boolean limitSwitchState = limitSwitch.get();

        // Encoder değerlerini oku
        int encoderCount = encoder.get(); // Encoder'den gelen pulse sayısı
        double distance = encoder.getDistance(); // Ölçülen mesafe

        // Encoder count kontrolü
        boolean encoderLimit = encoderCount >= 5000;

        // Durum değişikliği yakalamak için kontrol et
        if (limitSwitchState != prevLimitSwitchState) {
            System.out.println("Limit Switch State Changed: " + limitSwitchState);
            prevLimitSwitchState = limitSwitchState;
        }
        
        // Durumu SmartDashboard'a yazdır
        SmartDashboard.putBoolean("Limit Switch:", limitSwitchState);
        SmartDashboard.putNumber("Encoder Distance", distance);
        SmartDashboard.putNumber("Encoder Count", encoderCount);
        SmartDashboard.putBoolean("Encoder Limit Reached", encoderLimit);

        // Limit switch ve encoder limit durumlarını kontrol ederek motorları durdur
        // if (limitSwitchState || encoderLimit) {
        //     motor1.set(0.0); // Motor 1 durdur
        //     motor2.set(0.0); // Motor 2 durdur
        //     System.out.println("Motors Stopped: " +
        //         (limitSwitchState ? "Limit Switch Pressed" : "Encoder Limit Reached"));
        // } else {
            joyst();
            if(joystick.getRawButton(3)) {
                motor4.set(0.5);
            } else {
                motor4.set(0);
            }
            if(joystick.getRawButton(2)) {
                motor4.set(-1);
            } else {
                motor4.set(0);
            }
            if(joystick.getRawButton(1)) {
                motor5.set(0.5);
            } else {
                motor5.set(0);
            }
            if(joystick.getRawButton(4)) {
                motor5.set(-0.5);
            }
        // }
    }
}