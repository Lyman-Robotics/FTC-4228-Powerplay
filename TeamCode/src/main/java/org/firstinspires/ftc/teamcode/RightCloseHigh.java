package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

// import org.openftc.easyopencv.OpenCvCamera;
// import org.openftc.easyopencv.OpenCvCameraFactory;
// import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Right Close High", group = "Autonomous")
// @Disabled
public class RightCloseHigh extends LinearOpMode {

  @Override
  public void runOpMode() {
    // Initialize the hardware variables.
    RobotClass robot = new RobotClass(hardwareMap, true);

    // ! Runs upon initialization
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    robot.BRDrive.setDirection(DcMotor.Direction.FORWARD);
    robot.FLDrive.setDirection(DcMotor.Direction.REVERSE);
    robot.FRDrive.setDirection(DcMotor.Direction.REVERSE);
    robot.BLDrive.setDirection(DcMotor.Direction.FORWARD);

    robot.setToEncoderMode();
    robot.resetDrive();

    while (!isStarted()) {
      telemetry.addData("Position", robot.sleeveDetection.getPosition());
      telemetry.addData("PRELOAD CONE", "PRELOAD IT DUMBASS");
      telemetry.update();
      robot.position = robot.sleeveDetection.getPosition();
    }

    // ! Runs until the end of the match after play is pressed
    waitForStart();
    robot.timeElapsed.reset();

    while (opModeIsActive()) {
      robot.resetDrive();
      robot.ClawServo.setPosition(robot.servoClosePos);
      realSleep(1000, "Close claw", robot);
      robot.SlideMotor.setPower(robot.slidePowerUp);
      realSleep(2400, "Raise slide", robot);
      robot.SlideMotor.setPower(0);
      realSleep(500, "Stop slide", robot);
      robot.encoderDrive(0.2, 859, 747, 778, 862);
      // sleep(2000);
      robot.encoderDrive(0.2, 509, 448, 466, 508);
      // sleep(1000);
      robot.stopDrive();
      // sleep(1000);
      robot.encoderDrive(0.2, -509, -448, -466, -508);
      // sleep(2000);
      robot.stopDrive(); // might need to add sleep
      robot.encoderDrive(0.2, -1140, 1140, 1140, -1140);
      robot.encoderDrive(0.2, 130, 130, 130, 130);
      // sleep(1000);
      // sleep(600);
      robot.stopDrive();
      robot.SlideMotor.setPower(robot.slidePowerDown);
      sleep(1500);
      robot.SlideMotor.setPower(0);

      robot.ClawServo.setPosition(robot.servoOpenPos);
      sleep(300);
      robot.encoderDrive(0.2, -80, -80, -80, -80); // ,move back was -150 before
      // sleep(1000);
      // sleep(2000);

      // robot.encoderDrive(0.2,-278,458,350,-90); //move to pole
      // sleep(1000);
      // robot.encoderDrive(0.2,132,129,130,137); //go forwdard
      // sleep(600);
      // robot.stopDrive();
      // robot.SlideMotor.setPower(robot.slidePowerDown);
      // sleep(1500);
      // robot.SlideMotor.setPower(0);

      // robot.ClawServo.setPosition(robot.servoOpenPos);
      // sleep(300);
      // robot.encoderDrive(0.2,-122,-119,-120,-127); //,move back
      // sleep(1000);
      // robot.encoderDrive(0.2,278,-458,-350,90); //move away from pole
      // sleep(2000);

      if (robot.position.equals("Left")) {
        // ** Good
        robot.stopDrive();
        robot.encoderDrive(0.2, 410, -410, -410, 410); // last one! :D
        // realSleep(2900, "omni to pole", robot);
        robot.stopDrive();
      } else if (robot.position.equals("Center")) {
        robot.encoderDrive(0.2, 1100, -1100, -1100, 1100); // last one! :D
        robot.stopDrive();
      } else if (robot.position.equals("Right")) {
        // ** Good
        robot.stopDrive();
        robot.encoderDrive(0.2, 2000, -2000, -2000, 2000); // last one! :D
        // realSleep(2900, "omni to pole", robot);
        robot.stopDrive();
      }
      robot.stopDrive();
      realSleep(9999999, "Done", robot);
    }
  }

  public void realSleep(int n, String customAdd, RobotClass robot) { // better sleep method, dont use other crappy
    // stuffs
    telemetry.addData("Status", customAdd);
    telemetry.addData("FL Encoder", -robot.FLDrive.getCurrentPosition());
    telemetry.addData("FR Encoder", -robot.FRDrive.getCurrentPosition());
    telemetry.addData("BL Encoder", -robot.BLDrive.getCurrentPosition());
    telemetry.addData("BR Encoder", -robot.BRDrive.getCurrentPosition());
    telemetry.addData("Cone Pos", robot.position);
    telemetry.update();

    sleep(n);
  }

  public void encoderTelemetry(RobotClass robot) {
    telemetry.addData("FL Encoder", robot.FLDrive.getCurrentPosition());
    telemetry.addData("FR Encoder", robot.FRDrive.getCurrentPosition());
    telemetry.addData("BL Encoder", robot.BLDrive.getCurrentPosition());
    telemetry.addData("BR Encoder", robot.BRDrive.getCurrentPosition());
    telemetry.update();
  }
}
