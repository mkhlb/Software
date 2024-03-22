#include "shared/2021_robot_constants.h"
#include "shared/constants.h"
#include "software/jetson_nano/services/motor.h"
#include "software/logger/logger.h"
#include "software/estop/threaded_estop_reader.h"
#include "software/uart/boost_uart_communication.h"
#include <filesystem>

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
                                      motor_service_->DRIBBLER_MOTOR_CHIP_SELECT};

constexpr int ASCII_4671_IN_HEXADECIMAL = 0x34363731;
constexpr int DELAY_NS                  = 100000000;
std::string runtime_dir                 = "/tmp/tbots/yellow_test";

const std::string ESTOP_PATH_1 = "/dev/ttyACM0";
const std::string ESTOP_PATH_2 = "/dev/ttyUSB0";

int main(int argc, char **argv)
{
    LoggerSingleton::initializeLogger(runtime_dir);
    LOG(INFO) << "Running on the Jetson Nano!";

    LOG(INFO) << "Configuring the e-stop";

    std::string estop_path = "";

    if (std::filesystem::exists(ESTOP_PATH_1))
    {
        estop_path = ESTOP_PATH_1;
    }
    else if (std::filesystem::exists(ESTOP_PATH_2))
    {
        estop_path = ESTOP_PATH_2;
    }
    else
    {
        LOG(FATAL) << "No e-stop detected! Make sure an e-stop is plugged into a port on the jetson.";
    }


    ThreadedEstopReader estop =
            ThreadedEstopReader(std::make_unique<BoostUartCommunication>(115200, estop_path));

    LOG(INFO) << "E-stop configured successfully";

    motor_service_ =
            std::make_unique<MotorService>(create2021RobotConstants(), THUNDERLOOP_HZ);

    for (;;)
    {

        // Testing each motor SPI transfer, then open loop movement

        bool stop = !estop.isEstopPlay();
        for (uint8_t chip_select : CHIP_SELECT)
        {
            // open loop mode can be used without an encoder, set open loop phi positive
            // direction
            writeToControllerOrDieTrying(chip_select, TMC4671_OPENLOOP_MODE, 0x00000000, false);
            writeToControllerOrDieTrying(chip_select, TMC4671_PHI_E_SELECTION,
                                         TMC4671_PHI_E_OPEN_LOOP, false);
            writeToControllerOrDieTrying(chip_select, TMC4671_OPENLOOP_ACCELERATION, 0x0000003C, false);

            // represents effective voltage applied to the motors (% voltage)
            writeToControllerOrDieTrying(chip_select, TMC4671_UQ_UD_EXT, 0x00000799, false);

            // uq_ud_ext mode
            writeToControllerOrDieTrying(chip_select, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008, false);

            //if estop is not in play, stop motor and continue
            if (stop)
            {
                LOG(INFO) << "Stopping motor at chip select " << chip_select;
                writeToControllerOrDieTrying(chip_select, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000, false);
                continue;
            }
            std::make_unique<MotorService>(create2021RobotConstants(), THUNDERLOOP_HZ);

            LOG(INFO) << "Testing motor at chip select " << chip_select;

            motor_service_->writeIntToTMC4671(chip_select, TMC4671_CHIPINFO_ADDR,
                                              0x000000000);

            read_value = motor_service_->readIntFromTMC4671(chip_select, TMC4671_CHIPINFO_DATA);

            // Check if CHIPINFO_DATA returns 0x34363731
            if (read_value == ASCII_4671_IN_HEXADECIMAL)
            {
                LOG(INFO) << "SPI Transfer is successful";
            }
            else
            {
                LOG(INFO) << "SPI Transfer is not successful";
            }
        }

        usleep(DELAY_NS);
    }

}