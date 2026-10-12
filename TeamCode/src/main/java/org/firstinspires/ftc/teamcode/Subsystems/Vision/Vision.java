package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.Vector;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMemberMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

/**
 * The Vision Subsystem
 *
 * @see <a href="https://github.com/OpenFTC/EasyOpenCV">EasyOpenCV</a>
 */
public class Vision extends Subsystem {
    public static final int CAMERA_WIDTH = 640; // width of wanted camera resolution; original 1920
    public static final int CAMERA_HEIGHT = 480; // height of wanted camera resolutionl original 1080
    public static final String WEBCAM_NAME =
            "Webcam 1"; // insert webcam name from configuration if using webcam
    // Since ImageTarget trackable use mm to specify their dimensions, we must use mm for all the
    // physical dimension.
    // Define constants
    private static final double mmPerInch = 25.4;
    // Define where camera is in relation to center of robot in inches
    final static double CAMERA_FORWARD_DISPLACEMENT = 6.0f * mmPerInch; // TODO: CALIBRATE WHEN ROBOT IS BUILT
    final static double CAMERA_VERTICAL_DISPLACEMENT = 6.5f * mmPerInch;
    final static double CAMERA_RIGHT_DISPLACEMENT = 0.0f * mmPerInch;
    private final HardwareMap hardwareMap;
    private final AllianceColor allianceColor;
    private final Telemetry telemetry;

    public AprilTagDetectionThread aprilTagDetectionThread;

    private OpenCvCamera camera;

    /**
     * Class instantiation
     *
     * @param telemetry     Telemetry
     * @param hardwareMap   the hardware map
     * @param allianceColor the alliance color
     */
    public Vision(
            Telemetry telemetry,
            HardwareMap hardwareMap,
            AllianceColor allianceColor) {
        super(telemetry, "vision");
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.allianceColor = allianceColor;
        this.aprilTagDetectionThread = new AprilTagDetectionThread(this.hardwareMap.get(WebcamName.class, WEBCAM_NAME));
        this.aprilTagDetectionThread.start();
        // Telemetry
        logger.info("Vision init complete");
        // initDetectionPipeline() is not called here: it opened "Webcam 1" a second time through
        // EasyOpenCV while the VisionPortal in aprilTagDetectionThread already owns it.
    }

    public void getRobotPosition() {
        telemetry.addLine("Getting robot position");
        List<AprilTagDetection> tags = aprilTagDetectionThread.currentDetections;
       // Vector position = null;
        double distance = 100000;
        for (AprilTagDetection tag : tags) {
            // FIRST updated how vision works
            // https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-apriltag-clusters/tech-tip-apriltag-clusters.html <-- Read this

            if(tag instanceof AprilTagSingleDetection){
                telemetry.addLine("Single tag detected");
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) tag;
                telemetry.addLine(String.format("ID %d", singleDet.id));
                if (singleDet.ftcPose != null) {
                    addPoseTelemetry(singleDet.ftcPose);
                } else {
                    // ftcPose is only computed for tags whose ID is in the processor's tag library
                    telemetry.addLine("No pose (tag not in library)");
                }
                // TBD
            }
            else if(tag instanceof AprilTagClusterDetection){
                telemetry.addLine("Cluster tag detected");
                // TBD
                //
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) tag;
                VectorF fieldPosition = clusterDet.metadata.fieldPosition;

                telemetry.addData("Field location: ", fieldPosition);
                telemetry.addData("Cluster", "%s (%d%% found)", clusterDet.metadata.name, clusterDet.percentClusterFound);
                addPoseTelemetry(clusterDet.ftcPose);
                // id of elements in clusterDet.metadata.clusterMembers
            }
        }
        //return position;
    }
    private void addPoseTelemetry(AprilTagPoseFtc pose) {
        telemetry.addData("Range (in)", "%.1f", pose.range); // straight-line distance across the floor
        telemetry.addData("Bearing (deg)", "%.1f", pose.bearing);
        telemetry.addData("Elevation (deg)", "%.1f", pose.elevation);
        telemetry.addData("XYZ (in)", "%.1f  %.1f  %.1f", pose.x, pose.y, pose.z);
        telemetry.addData("PRY (deg)", "%.1f  %.1f  %.1f", pose.pitch, pose.roll, pose.yaw);
    }

    public void initDetectionPipelinePublic() {
        initDetectionPipeline();
    }

    private void initDetectionPipeline() {
        // Get the camera ID
        int cameraMonitorViewId =
                hardwareMap
                        .appContext
                        .getResources()
                        .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Obtain camera instance from ID
        camera =
                OpenCvCameraFactory.getInstance()
                        .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME));

        // Create a detection pipeline for detecting the position
        // TODO: Create a detection pipeline for detecting the position
//        pipeline = new [pipline name here]DetectionPipeline(allianceColor);
//        camera.setPipeline(pipeline);

        // Create listeners for the camera
        camera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() { // Listener for when the camera first starts
                        logger.info("Streaming");
                        camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) { // Listener to log if the camera stops abruptly
                        logger.error("Error Streaming, aborting with error code " + errorCode);
                    }
                });
    }

    /**
     * Stops AprilTag detection and releases the camera. Safe to call more than once.
     */
    public void stop() throws InterruptedException {
        aprilTagDetectionThread.shutdown();
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
            camera = null;
        }
    }
}
