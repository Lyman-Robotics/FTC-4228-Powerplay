package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "tedt", group = "tedt")
public class AutoForward extends LinearOpMode {
  @Override
  public void runOpMode() {
    // Initialize the hardware variables.
    RobotClass robot = new RobotClass(hardwareMap);

    // ! Runs upon initialization
    telemetry.addData("Status", "Initialized");
    telemetry.update();
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
    }
  }
}
