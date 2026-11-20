#include <bertec_am6817/BertecAM6817.h>

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

namespace
{
volatile std::sig_atomic_t g_stop = 0;
void onSignal(int)
{
  g_stop = 1;
}
} // namespace

int main(int argc, char ** argv)
{
  std::string port = "/dev/ttyUSB0";
  double duration_s = 3.0;
  std::size_t tare_samples = 200;

  if(argc > 1)
  {
    port = argv[1];
  }
  if(argc > 2)
  {
    duration_s = std::stod(argv[2]);
  }

  std::signal(SIGINT, onSignal);

  try
  {
    bertec::BertecAM6817 dev(port);
    dev.open();
    const auto info = dev.getInfo();
    const auto cal = dev.readCalibration();

    std::cout << "Model:       " << cal.model << "\n"
              << "Serial:      " << info.serial << "\n"
              << "Sample rate: " << info.sample_rate_hz << " Hz (" << static_cast<int>(info.num_channels)
              << " channels)\n";

    if(tare_samples > 0)
    {
      std::cout << "Taring (" << tare_samples << " samples)...\n";
      dev.tareBlocking(tare_samples);
    }

    dev.startAsync();
    const auto t0 = std::chrono::steady_clock::now();
    while(!g_stop)
    {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if(duration_s > 0.0 && elapsed >= duration_s)
      {
        break;
      }
      const auto s = dev.latestSample();
      std::cout << std::fixed << std::setprecision(2)
                << "n=" << std::setw(6) << s.sample_count << " drop=" << s.dropped_frames
                << " | F=[" << std::setw(7) << s.wrench[0] << ", " << std::setw(7) << s.wrench[1]
                << ", " << std::setw(7) << s.wrench[2] << "] N"
                << " | M=[" << std::setw(6) << s.wrench[3] << ", " << std::setw(6) << s.wrench[4]
                << ", " << std::setw(6) << s.wrench[5] << "] Nm\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    dev.stopAsync();
    const auto final_sample = dev.latestSample();
    std::cout << "Done: " << final_sample.sample_count << " samples, "
              << final_sample.dropped_frames << " dropped frames.\n";
  }
  catch(const std::exception & e)
  {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
