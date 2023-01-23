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

@Autonomous(name = "Right Far High", group = "Autonomous")
// @Disabled
public class RightEncoder2Cone extends LinearOpMode {

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
      sleep(999);
      robot.encoderDrive(
        0.26,
        1780,
        1770,
        1770,
        1780,
        robot.slidePowerUp,
        2000.0
      ); // Forward to cone (used to be 2000)
      robot.encoderDrive(0.2, -180, -170, -170, -180); // Back away from cone
      robot.encoderDrive(0.2, -420, 420, 420, -420); // Omni to pole
      robot.encoderDrive(0.2, 90, 90, 90, 90); // Forward to pole
      robot.SlideMotor.setPower(robot.slidePowerDown - 0.1); // Lower onto pole
      sleep(700);
      robot.SlideMotor.setPower(0); // Stop lowering
      robot.ClawServo.setPosition(robot.servoOpenPos); // Open servo
      robot.encoderDrive(0.2, -90, -90, -90, -90); // Back from pole

      // sleep(150);

      // robot.encoderDrive(0.2, -920, -1020, -1020, -920); // Reverse halfway push
      // robot.encoderDrive(0.2, 360, -325, -325, 360); // Go to front of pole
      // robot.encoderDrive(0.2, 190, 220, 220, 190); // Center cone on pole
      // robot.SlideMotor.setPower(robot.slidePowerDown - 0.1); // Lower onto pole
      // sleep(1333);
      // robot.SlideMotor.setPower(0); // Stop lowering
      // robot.ClawServo.setPosition(robot.servoOpenPos); // Open servo
      // sleep(150);
      // robot.encoderDrive(0.2, -170, -170, -170, -170); // Back away from pole
      // robot.encoderDrive(0.2, -400, 375, 375, -400); // Move left to sleeve spot
      // robot.encoderDrive(0.2, 770, 750, 750, 770); // Forward
      // robot.encoderDrive(
      //   0.2,
      //   -560,
      //   620,
      //   -640,
      //   620,
      //   robot.slidePowerDown,
      //   1500.0
      // ); // Turn to face stack
      // robot.encoderDrive(0.2, 800, 760, 760, 800); // Drive to stack
      // robot.ClawServo.setPosition(robot.servoClosePos); // Close servo
      // sleep(300);
      // robot.SlideMotor.setPower(robot.slidePowerUp);
      // sleep(2000);
      // robot.SlideMotor.setPower(0);

       if (robot.position.equals("Left")) { //shouldnt need to be changed much
      //   // ** Good
         robot.stopDrive();
         robot.encoderDrive(0.2, -768, 711, 835, -822); // last one! :D
         robot.stopDrive();
       } else if (robot.position.equals("Center")) {
      //   // ** Good
          robot.encoderDrive(0.2, -471, 395, 445, -461);
         robot.stopDrive();
       } else if (robot.position.equals("Right")) {
      //   // ** Good
         robot.stopDrive();
          robot.encoderDrive(0.2, -471, 395, 445, -461);
          robot.encoderDrive(0.2, -743, 653, 665, -691);

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
  //tedt so we don need sleep, motors should say "im done" by themselves
}
