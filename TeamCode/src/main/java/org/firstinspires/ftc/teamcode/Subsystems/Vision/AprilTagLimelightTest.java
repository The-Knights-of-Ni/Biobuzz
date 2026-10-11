package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.Locale;

public class AprilTagLimelightTest {

    private Limelight3A limelight;
    private IMU imu;
    private double distance;
    private double botXmm;
    private double botYmm;
    private double targetX;
    private double targetY;
    private double botHeadingDeg;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    public boolean detectMotif = false;
    public boolean detectRed = false;
    public boolean detectBlue = false;

    public LLResultTypes.FiducialResult motif;
    public LLResultTypes.FiducialResult blueGoal;
    public LLResultTypes.FiducialResult redGoal;



    public AprilTagLimelightTest(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }


//    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight-camera");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(4);
        //4 is all tags combined into one pipeline.
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

//    @Override
    public void start() {
        limelight.start();
    }

//    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight.getLatestResult();
        detectMotif = false;
        detectRed = false;
        detectBlue = false;
        telemetry.addLine("Limelight is limelighting");
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            List<LLResultTypes.FiducialResult> res = llResult.getFiducialResults();
            botXmm = botPose.getPosition().x;
            botYmm = botPose.getPosition().y;
            botHeadingDeg = botPose.getOrientation().getYaw();

            telemetry.addLine("We see the vision...");

            // String data1 = String.format(Locale.US, "{Botpose: %s, X (mm): %.3f, Y (mm): %.3f, Heading (deg): %.3f}", botPose.toString(), botXmm, botYmm, botHeadingDeg);
            // telemetry.addData("", data1);

            for(int i = 0; i<res.size(); i++){
                targetX = res.get(i).getTargetXDegrees();
                targetY = res.get(i).getTargetYDegrees();

                distance = getDistanceFromTags(res.get(i));

//                String data = String.format(Locale.US, "{ID: %d, Distance: %.3f, Target X: %.3f, Target Y: %.3f, Area: %.5f}",
//                        res.get(i).getFiducialId(), distance,  targetX, targetY, res.get(i).getTargetArea());
                String data = String.format(Locale.US, "ID: %d, Distance: %.2f, Target X: %.2f",
                        res.get(i).getFiducialId(), distance,  targetX);
                telemetry.addData("", data);

                detectMotif = true;
                if(res.get(i).getFiducialId() < 24 && res.get(i).getFiducialId() > 20){
                    motif = res.get(i);

                }
                else if(res.get(i).getFiducialId() == 24){
                    // red
                    detectRed = true;
                    redGoal = res.get(i);
                }
                else if(res.get(i).getFiducialId() == 20){
                    // blue
                    detectBlue = true;
                    blueGoal = res.get(i);
                }
            }

        }
        else if(llResult == null) {
            telemetry.addLine("Null results :(");
        }
        else{
            telemetry.addLine(llResult.toString());
            telemetry.addLine("No results :(");
        }
    }





    public double getDistanceFromTags(LLResultTypes.FiducialResult fres) {
        double dX = fres.getTargetPoseCameraSpace().getPosition().x;
        double dY = fres.getTargetPoseCameraSpace().getPosition().y;
        double dZ = fres.getTargetPoseCameraSpace().getPosition().z;
        return 100*Math.sqrt(dX*dX + dY*dY + dZ*dZ);
//        return 18.3*(1/Math.sqrt(ta));
    }

    public double getBotXmm() {
        return botXmm;
    }

    public double getBotYmm() {
        return botYmm;
    }

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY()  {return targetY;}

    public double getBotHeadingDeg() {
        return botHeadingDeg;
    }

    public double getDis(){ return distance;}
}