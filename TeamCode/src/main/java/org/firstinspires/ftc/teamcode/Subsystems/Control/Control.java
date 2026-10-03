package org.firstinspires.ftc.teamcode.Subsystems.Control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.FeedForward;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
//import org.firstinspires.ftc.teamcode.Util.ServoEx;


/**
 * Control subsystem for controlling arms and claws
 */
public class Control extends Subsystem {
//    public final Servo shootFlap;
    public final DcMotorEx shootMotor;
    public final DcMotorEx intakeMotorG;

    public final DcMotorEx intakeMotorR;
    private final FeedForward feedFowardVelocity = new FeedForward(10, 10);
    private final PID PIDVelocity = new PID(0.002, 0, 0.0002);

    public Control(Telemetry telemetry, DcMotorEx shootMotor, DcMotorEx intakeMotorG, DcMotorEx intakeMotorR) {
        super(telemetry, "control");
        this.shootMotor = shootMotor;
        this.intakeMotorG = intakeMotorG;
        this.intakeMotorR = intakeMotorR;

    }

    /**
     * Gets all defaults, directions,etc. ready for the autonomous period
     */
    public void initDevicesAuto() {
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotorG.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorG.setDirection(DcMotor.Direction.FORWARD);
        intakeMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorR.setDirection(DcMotor.Direction.FORWARD);
    }


    /**
     * Gets all defaults, directions,etc. ready for the teleop period
     */
    public void initDevicesTeleop() {
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotorG.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void startShoot(double power) {
        shootMotor.setMotorEnable();
        shootMotor.setPower(-power);
    }


    public void stopShoot() {
        shootMotor.setMotorDisable();
        shootMotor.setPower(0);
    }

    public void holdShootVelocity(double targetVelocity) {
        double currentVelocity = shootMotor.getVelocity();
        double feedForward = feedFowardVelocity.calculate(targetVelocity, targetVelocity-currentVelocity+0.1);
        double PIDCorrect = PIDVelocity.calculate(targetVelocity, currentVelocity);

        double variable = feedForward + PIDCorrect;

        shootMotor.setVelocity(variable);
    }


    public double shootMotorVelocity(double distance){
        if(distance < 120){
            return 0.58;
        }
        else if(distance<130){
            return 0.60;
        }
        else if(distance<140){
            return 0.61;
        }
        else if(distance<150){
            return 0.65;
        }
        else if(distance<160){
            return 0.66;
        }
        else if(distance<170){
            return 0.67;
        }
        else if(distance<180){
            return 0.68;
        }
        else if(distance<190){
            return 0.69;
        }
        else if(distance<220){
            return 0.79;
        }
        else{
            return 0.85;
            // 0.85 for long distance shooting area, 0.6 for short
        }
    }

}