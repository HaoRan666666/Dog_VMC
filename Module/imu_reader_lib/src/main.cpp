#include "imu_reader.h"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>

namespace {

std::atomic<bool> g_running{true};

void handle_signal(int) {
    g_running = false;
}

} // namespace

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <serial_device> [baud_rate]\n", argv[0]);  
        return 1;
    }

    std::string device = argv[1];
    int baud = argc > 2 ? atoi(argv[2]) : 921600;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    IMUReader reader;
    if (!reader.open(device, baud)) {
        return 1;
    }
    printf("Opened %s at %d baud\n", device.c_str(), baud);

    reader.configure();

    reader.setCallback([](const IMUData &d) {
        printf("accel(g): %+8.4f %+8.4f %+8.4f | "
               "gyro(°/s): %+8.2f %+8.2f %+8.2f | "
               "euler(°): %+8.2f %+8.2f %+8.2f | "
               "quat: %+8.5f %+8.5f %+8.5f %+8.5f\n",
               d.accel.x, d.accel.y, d.accel.z,
               d.gyro.x, d.gyro.y, d.gyro.z,
               d.euler.x, d.euler.y, d.euler.z,
               d.quat.w, d.quat.x, d.quat.y, d.quat.z);
    });

    while (g_running) {
        if (reader.update() < 0) break;
    }

    reader.close();
    return 0;
}
