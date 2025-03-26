#include <string>
#include <drivers/xbee/xbee.hpp>
#include "controller/controller.hpp"
#include "commander/commander.hpp"

namespace payload::telemetry {

    enum class flight_mode_state_t : std::uint8_t {
        ENABLE  = 0,
        ACTIVE  = 1,
        DISABLE = 2
    };
    class CommandsHandler {
        public:
            explicit CommandsHandler(drivers::xbee::Xbee& xbee, controller::Controller& controller, commander::Commander& commander);
            ~CommandsHandler();

            void handleCommand(std::string command);
        private:
            void cxHandler(bool isActive);
            void stHandler(const char* missionTime);
            void simpHandler(uint32_t pressure);
            void simHandler(flight_mode_state_t state);
            void calHandler();

            int pressure_counter_from_ground = 0;

            bool simEnabled = false;

            drivers::xbee::Xbee& xbee_;
            controller::Controller& controller_;
            commander::Commander& commander_;
    };
}