package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Auto Encoder Dev", group = "Autonomous Dev")
public class AutoEncoderDev extends LinearOpMode {

  @Override
  public void runOpMode() {
    // Initialize the hardware variables.
    RobotClass robot = new RobotClass(hardwareMap, false);

    // ! Runs upon initialization
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    robot.setToEncoderMode();
    robot.resetDrive();

    // Initialize drive variables
    float vertical;
    float horizontal;
    float pivot;
    double speedScalar;

    // ! Runs until the end of the match after play is pressed
    waitForStart();
    robot.timeElapsed.reset();

    while (opModeIsActive()) {
      if (gamepad1.left_stick_y < 0) {
          robot.setDrivePower(0.25, 0.25, 0.25, 0.25);
        } else if (gamepad1.left_stick_y > 0) {
          robot.setDrivePower(-0.25, -0.25, -0.25, -0.25);
        } else if (gamepad1.left_stick_x > 0) {
          robot.omnidrive(0.5, (Math.PI / 2), 0);
        } else if (gamepad1.left_stick_x < 0) {
          robot.omnidrive(-0.5, (Math.PI / 2), 0);
        } else {
          robot.setDrivePower(0, 0, 0, 0);
        }

      if (gamepad1.right_stick_button) {
        robot.resetDrive();
        robot.setToPowerMode();
      }

      if (gamepad1.left_stick_button) {
        robot.resetDrive();
        robot.setToPowerMode();
      }

      telemetry.addData("FL Encoder", -robot.FLDrive.getCurrentPosition());
      telemetry.addData("FR Encoder", -robot.FRDrive.getCurrentPosition());
      telemetry.addData("BL Encoder", -robot.BLDrive.getCurrentPosition());
      telemetry.addData("BR Encoder", -robot.BRDrive.getCurrentPosition());
      telemetry.update();
    }
  }
}
