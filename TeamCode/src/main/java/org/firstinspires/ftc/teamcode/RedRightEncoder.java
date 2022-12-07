package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// import org.openftc.easyopencv.OpenCvCamera;
// import org.openftc.easyopencv.OpenCvCameraFactory;
// import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Left Encoder", group = "Autonomous")
// @Disabled
public class RedRightEncoder extends LinearOpMode {
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

    // TODO Open CV
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
      // ! New Stuff
      robot.ClawServo.setPosition(robot.servoClosePos);
      realSleep(1000, "Close Servo with cone", robot);
      robot.SlideMotor.setPower(robot.slidePowerUp);
      realSleep(350, "Slide up with cone", robot);
      robot.SlideMotor.setPower(0);
      realSleep(500, "Wait to move", robot);
      robot.setDrivePower(.25, .25, .25, .25);
      realSleep(1560, "Move to signal cone", robot);
      robot.stopDrive();
      realSleep(500, "Wait to show cone", robot);

      if (robot.position.equals("Left")) {
        // ! Bad
        robot.stopDrive();
        robot.omnidrive(0.5, robot.omniLeftVal, 0);
        realSleep(1300, "omni to pole", robot);
        robot.stopDrive();
      } else if (robot.position.equals("Center")) {
        // !** Good
        robot.stopDrive();
      } else if (robot.position.equals("Right")) {
        // ! Bad
        robot.stopDrive();
        robot.omnidrive(0.5, robot.omniRightVal, 0);
        realSleep(1300, "omni to pole", robot);
        robot.stopDrive();
      }

      realSleep(9999999, "Done", robot);
    }
  }

  public void realSleep(int n, String customAdd, RobotClass robot) { // better sleep method, dont use other crappy
                                                                     // stuffs
    telemetry.addData("Status", customAdd);
    telemetry.addData("Claw Servo Position", robot.ClawServo.getPosition());
    telemetry.addData("Slide Touch Sensor", !(robot.SlideTouchSensor.getState()));
    telemetry.addData("Elapsed Time", robot.timeElapsed.toString());
    telemetry.addData("Cone Pos", robot.position);
    telemetry.update();

    sleep(n);
  }
}
