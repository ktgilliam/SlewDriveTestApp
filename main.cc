// #include "df2_filter.h"
#include <cstring>
#include <memory>
#include <string>
#include <iostream>
#include <stdexcept>
#include <future>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>

#include "KincoDriver.h"

constexpr int driveID = 1;

const int N = 10;
const char devPath[] = "/dev/ttyUSB0";

double velVals[N]{0};

int main()
{
    TerminalInterface *terminal = new TerminalInterface("SLEW DRIVE TEST INTERFACE");
    KincoDriver *kincoDrivePtr = new KincoDriver(1);

    kincoDrivePtr->connectTerminalInterface(terminal);
    terminal->printHeader();
    kincoDrivePtr->connectRTU(devPath, 19200, 'N', 8, 1);
    kincoDrivePtr->setDriverState(KINKO::MOTOR_STATE_ENABLE);
    kincoDrivePtr->setDriverState(KINKO::MOTOR_STATE_ON);
    kincoDrivePtr->setControlMode(KINKO::MOTOR_MODE_SPEED);

    std::mutex m;
    std::condition_variable cv;
    std::string new_string;
    bool error = false;

    auto io_thread = std::thread([&]
                                 {
        std::string s;
        while(!error && std::getline(std::cin, s, '\n'))
        {
            auto lock = std::unique_lock<std::mutex>(m);
            new_string = std::move(s);
            if (new_string == "quit") 
            {
                error = true;
            }
            lock.unlock();
            cv.notify_all();
        }
        auto lock = std::unique_lock<std::mutex>(m);
        error = true;
        lock.unlock();
        cv.notify_all(); });

    auto current_string = std::string();
    for (;;)
    {
        auto lock = std::unique_lock<std::mutex>(m);
        cv.wait(lock, [&]
                { return error || (current_string != new_string); });
        if (error)
        {
            break;
        }
        current_string = new_string;
        lock.unlock();
        unsigned long int cnt = 0;
        try
        {
            kincoDrivePtr->updateVelocityCommand(200);

            bool keepGoing = true;
            while (keepGoing)
            {
                velVals[cnt++] = kincoDrivePtr->getVelocityFeedback(true);
                // terminal->serviceCLI();
            }

            std::cout << std::endl;
        }

        catch (const std::exception &e)
        {
            std::cerr << std::endl
                      << e.what() << '\n';
            std::cout << "ERROR";
        }

        std::stringstream ss;
        ss << cnt << ";" << current_string;
        terminal->addDebugMessage(ss.str());
        // now use the string that arrived from our non-blocking stream
        // std::cout << "new string: " << current_string;
        // std::cout.flush();
        // for (int i = 0; i < 10; ++i)
        // {
        //     std::this_thread::sleep_for(std::chrono::seconds(1));
        //     std::cout << " " << i;
        //     std::cout.flush();
        // }
        // std::cout << ". done. next?\n";
        // std::cout.flush();
    }
    io_thread.join();

    kincoDrivePtr->updateVelocityCommand(0);
    kincoDrivePtr->setDriverState(KINKO::MOTOR_STATE_ENABLE);

    return 0;

    // std::unique_ptr<KincoDriver>kincoDrivePtr(new KincoDriver(driveID, devPath));
    // kincoDrivePtr->getControlMode();
}