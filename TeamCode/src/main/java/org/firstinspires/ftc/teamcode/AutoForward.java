package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

<<<<<<< HEAD
@Autonomous(name = "Forward & Back", group = "Autonomous Old")
=======
@Autonomous(name = "tedt", group = "tedt")
>>>>>>> parent of 1a73325 (among us)
public class AutoForward extends LinearOpMode {
  @Override
  public void runOpMode() {
    // Initialize the hardware variables.
    RobotClass robot = new RobotClass(hardwareMap, true);

    // ! Runs upon initialization
    telemetry.addData("Status", "Initialized");
    telemetry.update();
<<<<<<< HEAD

    robot.BRDrive.setDirection(DcMotor.Direction.FORWARD);
    robot.FLDrive.setDirection(DcMotor.Direction.REVERSE);
    robot.FRDrive.setDirection(DcMotor.Direction.REVERSE);
    robot.BLDrive.setDirection(DcMotor.Direction.FORWARD);

    // ! Runs until the end of the match after play is pressed
    waitForStart();
    robot.timeElapsed.reset();

    while (opModeIsActive()) {
      robot.ClawServo.setPosition(0.355);
      realSleep(1500, "Servo closed", robot);
      robot.SlideMotor.setPower(1);
      realSleep(2000, "Slide motor up", robot);
      robot.SlideMotor.setPower(0);
      realSleep(200, "Slide motor stopped", robot);

      robot.setDrivePower(0.25, 0.25, 0.25, 0.25);
      realSleep(2500, "Forward", robot);
      robot.setDrivePower(0, 0, 0, 0);
      realSleep(500, "Stopped", robot);
      // robot.setDrivePower(-0.25, -0.25, -0.25, -0.25);
      // realSleep(2200, "Backward", robot);

      robot.setDrivePower(0, 0, 0, 0);
      realSleep(200, "stop", robot);
      robot.setDrivePower(-0.25, -0.25, -0.25, -0.25);
      realSleep(500, "Backward", robot);
      robot.SlideMotor.setPower(-1);
      realSleep(1000, "Lower slide", robot);
      robot.SlideMotor.setPower(0);

      // robot.SlideMotor.setPower(1);
      // realSleep(6000, "Slide motor up", robot);
      // robot.SlideMotor.setPower(0);
      // realSleep(200, "Slide motor stopped", robot);

      // robot.omnidrive(0.5, (Math.PI / 2), 0);
      // realSleep(2000, "Forward", robot);
      // robot.setDrivePower(0, 0, 0, 0);
      // realSleep(200, "Thing", robot);
      // robot.omnidrive(0.5, (Math.PI / 2), 0);
      // realSleep(200, "tgk", robot);
      // robot.omnidrive(0.5, (Math.PI / 2), 0);

      realSleep(9999999, "Done", robot);
=======
    waitForStart();
    robot.timeElapsed.reset();

    robot.ClawServo.setPosition(0.4);

    // Initialize drive variables
    float vertical;
    float horizontal;
    float pivot;
    double speedScalar;

    // ! Runs until the end of the match after play is pressed
    while (opModeIsActive()) {
      // double max;

      // vertical = gamepad1.left_stick_y;
      // horizontal = -gamepad1.left_stick_x;
      // pivot = -gamepad1.right_stick_x;

      // if (gamepad1.right_bumper) {
      // slowMode = true;
      // } else if (gamepad1.left_bumper) {
      // slowMode = false;
      // }
      // speedScalar = slowMode ? 0.15 : 0.65;

      // double FRPower = ((-pivot + (vertical - horizontal)) * speedScalar);
      // double BRPower = ((-pivot + vertical + horizontal) * speedScalar);
      // double FLPower = ((pivot + vertical + horizontal) * speedScalar);
      // double BLPower = ((pivot + (vertical - horizontal)) * speedScalar);

      // // ? Nerd stuff to make sure the robot doesn't go too fast
      // max = Math.max(Math.abs(FLPower), Math.abs(FRPower));
      // max = Math.max(max, Math.abs(BLPower));
      // max = Math.max(max, Math.abs(BRPower));

      // if (max > 1.0) {
      // FLPower /= max;
      // FRPower /= max;
      // BLPower /= max;
      // BRPower /= max;
      // }
      // // ? Nerd stuff ends here

      // // Send calculated power to wheels
      // // robot.setDrivePower(FLPower, FRPower, BLPower, BRPower);

      robot.setDrivePower(-.5, -.5, -.5, -.5);

      sleep(500);
      robot.setDrivePower(0, 0, 0, 0);
      sleep(9999999);
>>>>>>> parent of 1a73325 (among us)
    }
  }

  public void realSleep(int n, String customAdd, RobotClass robot) {
    telemetry.addData("Status", customAdd);
    telemetry.addData("Claw Servo Position", robot.ClawServo.getPosition());
    telemetry.addData("Slide Touch Sensor", !(robot.SlideTouchSensor.getState()));
    telemetry.addData("Elapsed Time", robot.timeElapsed.toString());

    sleep(n);

    // telemetry.addData("Front left/Right", "%4.2f, %4.2f", robot.FLPower,
    // robot.FRPower);
    // telemetry.addData("Back left/Right", "%4.2f, %4.2f", robot.BLPower,
    // robot.BRPower);
    telemetry.update();
  }
}
