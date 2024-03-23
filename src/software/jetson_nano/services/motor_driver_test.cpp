#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/services/motor.h"
#include "software/logger/logger.h"
#include "software/estop/threaded_estop_reader.h"
#include "software/uart/boost_uart_communication.h"
#include <experimental/filesystem>

extern "C"
{
#include "external/trinamic/tmc/ic/TMC4671/TMC4671.h"
#include "external/trinamic/tmc/ic/TMC4671/TMC4671_Variants.h"
#include "external/trinamic/tmc/ic/TMC6100/TMC6100.h"
}

std::unique_ptr<MotorService> motor_service_;
int read_value;

static const uint8_t CHIP_SELECT[] = {motor_service_->FRONT_LEFT_MOTOR_CHIP_SELECT,
                                      motor_service_->FRONT_RIGHT_MOTOR_CHIP_SELECT,
                                      motor_service_->BACK_LEFT_MOTOR_CHIP_SELECT,
                                      motor_service_->BACK_RIGHT_MOTOR_CHIP_SELECT,
                                      motor_service_->DRIBBLER_MOTOR_CHIP_SELECT
                                      };

constexpr int ASCII_4671_IN_HEXADECIMAL = 0x34363731;
constexpr int DELAY_MICROSECONDS        = 500000;
std::string runtime_dir                 = "/tmp/tbots/yellow_test";

const std::string ESTOP_PATH_1 = "/dev/ttyACM0";
const std::string ESTOP_PATH_2 = "/dev/ttyUSB0";

int main(int argc, char **argv)
{
    LoggerSingleton::initializeLogger(runtime_dir);
    LOG(INFO) << "Running on the Jetson Nano!";

    LOG(INFO) << "Configuring the e-stop";

    std::string estop_path = "";

    if (std::experimental::filesystem::exists(ESTOP_PATH_1))
    {
        estop_path = ESTOP_PATH_1;
    }
    else if (std::experimental::filesystem::exists(ESTOP_PATH_2))
    {
        estop_path = ESTOP_PATH_2;
    }
    else
    {
        LOG(FATAL) << "No e-stop detected! Make sure an e-stop is plugged into a port on the Jetson.";
    }


    ThreadedEstopReader estop =
            ThreadedEstopReader(std::make_unique<BoostUartCommunication>(115200, estop_path));

    LOG(INFO) << "E-stop configured successfully";

    motor_service_ =
            std::make_unique<MotorService>(create2021RobotConstants(), THUNDERLOOP_HZ);

    // Initialize motors
    motor_service_->setUpDriveMotor(motor_service_->FRONT_LEFT_MOTOR_CHIP_SELECT);
    motor_service_->setUpDriveMotor(motor_service_->FRONT_RIGHT_MOTOR_CHIP_SELECT);
    motor_service_->setUpDriveMotor(motor_service_->BACK_LEFT_MOTOR_CHIP_SELECT);
    motor_service_->setUpDriveMotor(motor_service_->BACK_RIGHT_MOTOR_CHIP_SELECT);


    while (true)
    {

        // Testing each motor SPI transfer, then open loop movement

        bool stop = !estop.isEstopPlay();
        for (uint8_t chip_select : CHIP_SELECT)
        {
            //if estop is not in play, stop motor and continue
            if (stop)
            {
                motor_service_->writeToControllerOrDieTrying(chip_select, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000, false);
                continue;
            }

            motor_service_->writeIntToTMC4671(chip_select, TMC4671_CHIPINFO_ADDR,
                                              0x000000000);

            read_value = motor_service_->readIntFromTMC4671(chip_select, TMC4671_CHIPINFO_DATA);

            // Check if CHIPINFO_DATA returns 0x34363731
            if (read_value != ASCII_4671_IN_HEXADECIMAL)
            {
                LOG(INFO) << motor_service_->getMotorName(chip_select) << " motor: SPI Transfer is not successful";
                motor_service_->writeToControllerOrDieTrying(chip_select, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000, false);
                continue;
            }

            // open loop mode can be used without an encoder, set open loop phi positive
            // direction
            motor_service_->writeToControllerOrDieTrying(chip_select, TMC4671_OPENLOOP_MODE, 0x00000000);

            motor_service_->writeToControllerOrDieTrying(chip_select, TMC4671_PHI_E_SELECTION,
                                         TMC4671_PHI_E_OPEN_LOOP);
            motor_service_->writeToControllerOrDieTrying(chip_select, TMC4671_OPENLOOP_ACCELERATION, 0x0000003C);

            // represents effective voltage applied to the motors (% voltage)
            motor_service_->writeToControllerOrDieTrying(chip_select, TMC4671_UQ_UD_EXT, 0x00000799);

            // uq_ud_ext mode
            motor_service_->writeToControllerOrDieTrying(chip_select, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);

            // 200 RPM
            motor_service_->writeToControllerOrDieTrying(chip_select, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000058);

            LOG(INFO) << motor_service_->readIntFromTMC4671(chip_select, TMC4671_OPENLOOP_VELOCITY_ACTUAL);
            LOG(INFO) << "Moved robot";
        }

        usleep(DELAY_MICROSECONDS);
    }

}