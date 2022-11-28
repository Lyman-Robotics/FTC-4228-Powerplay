package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;

// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.teamcode.common.powerplay.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotClass {
  // ** Declare OpMode members
  public DcMotor FLDrive;
  public DcMotor FRDrive;
  public DcMotor BLDrive;
  public DcMotor BRDrive;
  public DcMotor SlideMotor;
  public Servo ClawServo;
  public DigitalChannel SlideTouchSensor;

  public SleeveDetection sleeveDetection;
  public OpenCvCamera camera;
  public String webcamName = "Webcam 1";

  public enum ParkingPosition {
    LEFT,
    CENTER,
    RIGHT
  }

  public ParkingPosition position;

  HardwareMap hwMap = null;
  public ElapsedTime timeElapsed = new ElapsedTime();

  public RobotClass(HardwareMap hwMap, Boolean initServo) {
    init(hwMap, initServo);
  }

  /* Initialize standard Hardware interfaces */
  public void init(HardwareMap hwMap, Boolean initServo) {
    // Save reference to Hardware map
    hwMap = hwMap;

    // Define and initialize motors
    FLDrive = hwMap.get(DcMotor.class, "FLDrive");
    FRDrive = hwMap.get(DcMotor.class, "FRDrive");
    BLDrive = hwMap.get(DcMotor.class, "BLDrive");
    BRDrive = hwMap.get(DcMotor.class, "BRDrive");
    SlideMotor = hwMap.get(DcMotor.class, "SlideMotor");
    ClawServo = hwMap.get(Servo.class, "ClawServo");
    SlideTouchSensor = hwMap.get(DigitalChannel.class, "SlideTouchSensor");
    // ? ServoPlaceholder = hwMap.get(Servo.class, "ServoPlaceholder");

    SlideTouchSensor.setMode(DigitalChannel.Mode.INPUT);

    // Make robot drive straight
    BRDrive.setDirection(DcMotor.Direction.REVERSE);
    BLDrive.setDirection(DcMotor.Direction.REVERSE);
    ClawServo.setDirection(Servo.Direction.REVERSE);

    FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // If init servo, set to init position
    if (initServo) {
      ClawServo.setPosition(0.7494);
    }

    // TODO Open CV
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
        hardwareMap.appContext.getPackageName());
    camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName),
        cameraMonitorViewId);
    sleeveDetection = new SleeveDetection();
    camera.setPipeline(sleeveDetection);

    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
      }

      @Override
      public void onError(int errorCode) {
      }
    });

    position = sleeveDetection.getPosition();
    // TODO End Open CV
  }

  public void resetDrive() {
    FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    FLDrive.setTargetPosition(0);
    FRDrive.setTargetPosition(0);
    BLDrive.setTargetPosition(0);
    BRDrive.setTargetPosition(0);
  }

  public void runToPosDrive() {
    // Runs to position set by setTargetPosition
    FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  public void stopDrive() {
    FLDrive.setPower(0);
    FRDrive.setPower(0);
    BLDrive.setPower(0);
    BRDrive.setPower(0);
  }

  public void setDrivePower(double FL, double FR, double BL, double BR) {
    FLDrive.setPower(FL);
    FRDrive.setPower(FR);
    BLDrive.setPower(BL);
    BRDrive.setPower(BR);
  }

  // Radians
  public void omnidrive(double power, double angle, double rotation) {
    double FL = power * Math.sin(angle + Math.PI / 4) + rotation;
    double FR = power * Math.cos(angle + Math.PI / 4) - rotation;
    double BL = power * Math.cos(angle + Math.PI / 4) + rotation;
    double BR = power * Math.sin(angle + Math.PI / 4) - rotation;

    FLDrive.setPower(FL);
    FRDrive.setPower(FR);
    BLDrive.setPower(BL);
    BRDrive.setPower(BR);
  }
}
