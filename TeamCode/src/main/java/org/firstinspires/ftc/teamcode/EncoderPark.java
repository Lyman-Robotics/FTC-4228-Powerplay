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

@Autonomous(name = "Encoder Park", group = "Autonomous")
// @Disabled
public class EncoderPark extends LinearOpMode {

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
      realSleep(1200, "Raise slide", robot);
      robot.SlideMotor.setPower(0);
      realSleep(500, "Stop slide", robot);
      robot.encoderDrive(0.2, 859, 747, 778, 862);
      sleep(2000);
      robot.encoderDrive(0.2, 509, 448, 466, 508);
      sleep(1000);
      robot.stopDrive();
      sleep(1000);
      robot.encoderDrive(0.2, -509, -448, -466, -508);

      sleep(2000);
      if (robot.position.equals("Left")) {
        // ** Good
        robot.stopDrive();
        robot.encoderDrive(0.2, -768, 711, 835, -822); // last one! :D
        realSleep(2900, "omni to pole", robot);
        robot.stopDrive();
      } else if (robot.position.equals("Center")) {
        // ** Good
        robot.stopDrive();
      } else if (robot.position.equals("Right")) {
        // ** Good
        robot.stopDrive();
        robot.encoderDrive(0.2, 700, -661, -728, 791);
        realSleep(2900, "omni to pole", robot);
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
