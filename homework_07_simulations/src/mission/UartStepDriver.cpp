#include "mission/UartStepDriver.h"

#include <thread>
#include <chrono>

#include "common/macros.h"
#include "mission/UartDroneState.h"
#include "protocol/IUartLink.h"
#include "protocol/IDroneGpioController.h"
#include "protocol/IMissionCommandSource.h"
#include "protocol/IUartTelemetryProvider.h"

UartStepDriver::UartStepDriver(IUartLink* uart,
                               IDroneGpioController* gpio,
                               IMissionCommandSource* cmdSource,
                               IUartTelemetryProvider* telProvider,
                               UartDroneState* droneState,
                               double maxTurnPerStep,
                               float accelPerStep,
                               int maxSteps)
    : uart_(uart),
      gpio_(gpio),
      cmdSource_(cmdSource),
      telProvider_(telProvider),
      droneState_(droneState),
      maxTurnPerStep_(maxTurnPerStep),
      accelPerStep_(accelPerStep),
      maxSteps_(maxSteps)
{
}

bool UartStepDriver::waitNextTick()
{
    // Обробляємо кожен кадр телеметрії чекера РІВНО раз. Чекер оновлює
    // телеметрію у своєму такті (~timeStep); опитування частіше лише
    // повторно запускає наведення на тих самих (застарілих) даних, що
    // розганяє логічний годинник місії і стан повороту швидше за реальний
    // дрон і збиває тайминг скиду. Гейтимо по таймстемпу телеметрії, щоб
    // місія крокувала в лок-степ з чекером (одна CONTROL на кадр).
    while (true) {
        if (step_ >= maxSteps_) {
            LOG("StepDriver: reached max steps (" << maxSteps_ << ")");
            return false;
        }

        if (!telProvider_->isSimulationRunning()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        const auto& t = telProvider_->getTelemetry();
        if (!firstFrame_ && t.t_ms == lastTelemetryMs_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        lastTelemetryMs_ = t.t_ms;
        firstFrame_ = false;
        return true;
    }
}

void UartStepDriver::beforeStep()
{
    // Подати кадр телеметрії чекера у джерело стану дрона.
    const auto& t = telProvider_->getTelemetry();

    dt_ = DroneTelemetry{};
    dt_.pos = {t.x, t.y};
    dt_.direction = t.dir;
    dt_.speed = t.speed;
    dt_.state = static_cast<int8_t>(t.state);
    dt_.timeSecSinceStart = t.t_ms / 1000.0f;

    droneState_->setTelemetry(dt_);
}

void UartStepDriver::afterStep()
{
    // Перетворити рішення місії на нормовану команду CONTROL і надіслати.
    DroneCommand cmd = droneState_->getLastCommand();
    float accel = 0.0f;
    float turnRate = 0.0f;
    cmdSource_->computeControl(cmd, dt_, maxTurnPerStep_, accelPerStep_, accel, turnRate);

    if (!uart_->sendControl(accel, turnRate)) {
        LOG("Failed to send CONTROL command at step " << step_);
    }

    if (step_ % 100 == 0) {
        LOG("Step " << step_ << ": tel_pos=(" << dt_.pos.x << "," << dt_.pos.y
                    << ") speed=" << dt_.speed << " dir=" << dt_.direction
                    << " cmd=[accel=" << accel << " turn=" << turnRate << "]");
    }

    // Невелика затримка опитування; цикл сам пейситься на нових кадрах вище.
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    step_++;
}

void UartStepDriver::onDrop()
{
    LOG("*** DROP PULSE TRIGGERED at step " << step_ << " ***");
    uart_->sendControl(0.0f, 0.0f);
    gpio_->pulseDrop(180); // ~180ms імпульс скиду
}
