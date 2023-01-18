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

@Autonomous(name = "Left Encoder 2+ Cone", group = "Autonomous")
// @Disabled
public class LeftEncoder2Cone extends LinearOpMode {

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
      for (int i = 0; i < 10; i++) {
        telemetry.addData("PRELOAD CONE", "PRELOAD IT DUMBASS");
      }
      telemetry.update();
      robot.position = robot.sleeveDetection.getPosition();
    }

    // ! Runs until the end of the match after play is pressed
    waitForStart();
    robot.timeElapsed.reset();

    while (opModeIsActive()) {
      robot.resetDrive();
      robot.ClawServo.setPosition(robot.servoClosePos);
      sleep(1000);
      robot.SlideMotor.setPower(robot.slidePowerUp);
      sleep(2000);
      robot.SlideMotor.setPower(0);
      sleep(500);
      robot.encoderDrive(0.3, 860, 750, 750, 860); // Forward to cone
      robot.encoderDrive(0.3, 920, 1020, 1020, 920); // Push cone to halfway on field
      robot.encoderDrive(0.3, -920, -1020, -1020, -920); // Reverse halfway push
      robot.encoderDrive(0.3, 360, -325, -325, 360); // Go to front of pole
      robot.encoderDrive(0.3, 190, 220, 220, 190); // Center cone on pole
      robot.SlideMotor.setPower(robot.slidePowerDown); // Lower onto pole
      sleep(1600);
      robot.SlideMotor.setPower(0); // Stop lowering
      robot.ClawServo.setPosition(robot.servoOpenPos); // Open servo
      sleep(300);
      robot.encoderDrive(0.3, -170, -170, -170, -170); // Back away from pole
      robot.encoderDrive(0.3, -400, 375, 375, -400); // Move left to sleeve spot
      robot.encoderDrive(0.3, 770, 750, 750, 770); // Forward
      robot.encoderDrive(0.3, -560, 620, -640, 620); // Turn to face stack
      robot.SlideMotor.setPower(robot.slidePowerDown); // Lower onto stack height
      sleep(1500);
      robot.SlideMotor.setPower(0); // Stop lowering
      robot.encoderDrive(0.2, 800, 760, 760, 800); // Drive to stack
      robot.ClawServo.setPosition(robot.servoClosePos); // Close servo
      sleep(300);
      robot.SlideMotor.setPower(robot.slidePowerUp);
      sleep(2000);
      robot.SlideMotor.setPower(0);

      // robot.encoderDrive(0.2, 1509, 1448, 1466, 1508);
      // robot.encoderDrive(0.2, -1509, -1448, -1466, -1508);
      // robot.encoderDrive(0.2, 448, -401, -440, 433);
      // robot.encoderDrive(0.2, 224, 190, 197, 215);

      // robot.encoderDrive(0.2, -100, -100, -100, -100); //,move back was -150 bnefore
      // robot.encoderDrive(0.2, -448, 401, 440, -433); //move away from pole

      // if (robot.position.equals("Left")) { //shouldnt need to be changed much
      //   // ** Good
      //   robot.stopDrive();
      //   robot.encoderDrive(0.2, -768, 711, 835, -822); // last one! :D
      //   robot.stopDrive();
      // } else if (robot.position.equals("Center")) {
      //   // ** Good
      //   robot.stopDrive();
      // } else if (robot.position.equals("Right")) {
      //   // ** Good
      //   robot.stopDrive();
      //   robot.encoderDrive(0.2, 780, -741, -808, 861);
      //   robot.stopDrive();
      // }

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