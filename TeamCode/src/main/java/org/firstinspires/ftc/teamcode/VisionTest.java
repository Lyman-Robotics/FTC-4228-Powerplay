package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Signal Sleeve Test")
public class VisionTest extends LinearOpMode {

  SleeveDetection sleeveDetection;
  OpenCvCamera camera;

  // Name of the Webcam to be set in the config
  String webcamName = "Webcam 1";

  @Override
  public void runOpMode() throws InterruptedException {
    int cameraMonitorViewId = hardwareMap.appContext
      .getResources()
      .getIdentifier(
        "cameraMonitorViewId",
        "id",
        hardwareMap.appContext.getPackageName()
      );
    camera =
      OpenCvCameraFactory
        .getInstance()
        .createWebcam(
          hardwareMap.get(WebcamName.class, webcamName),
          cameraMonitorViewId
        );
    sleeveDetection = new SleeveDetection();
    camera.setPipeline(sleeveDetection);

    camera.openCameraDeviceAsync(
      new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
          camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);/*
           * SIDEWAYS_LEFT
           * SIDEWAYS_RIGHT
           * UPSIDE_DOWN
           * UPRIGHT
           * are the only valid values
           */
        }

        @Override
        public void onError(int errorCode) {}
      }
    );

    while (!isStarted()) {
      telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
      telemetry.update();
    }
    // sleevedetection get position variable

    waitForStart();
  }
}
