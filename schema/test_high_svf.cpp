#include <ATK/Core/InPointerFilter.h>
#include <ATK/Core/OutPointerFilter.h>
#include <ATK/EQ/SecondOrderSVFFilter.h>

#include <boost/math/constants/constants.hpp>

#include <fstream>
#include <memory>
#include <vector>

constexpr gsl::index PROCESSSIZE = 4 * 1024 * 1024;
constexpr size_t SAMPLING_RATE = 96000;

int main(int argc, const char** argv)
{
  std::vector<double> input(PROCESSSIZE);
  std::vector<double> output(PROCESSSIZE);
  for(size_t i = 0; i < PROCESSSIZE; ++i)
  {
    auto frequency = (20. + i) / PROCESSSIZE * (20000 - 20);
    input[i] = std::sin(i * boost::math::constants::pi<double>() * (frequency / SAMPLING_RATE));
  }

  ATK::InPointerFilter<double> inFilter(input.data(), 1, PROCESSSIZE, false);
  ATK::SecondOrderSVFFilter<ATK::SecondOrderSVFHighShelfCoefficients<double>> highToneControlFilter(1);
  ATK::OutPointerFilter<double> outFilter(output.data(), 1, PROCESSSIZE, false);

  highToneControlFilter.set_input_port(0, &inFilter, 0);
  outFilter.set_input_port(0, &highToneControlFilter, 0);

  inFilter.set_input_sampling_rate(SAMPLING_RATE);
  inFilter.set_output_sampling_rate(SAMPLING_RATE);
  highToneControlFilter.set_input_sampling_rate(SAMPLING_RATE);
  highToneControlFilter.set_output_sampling_rate(SAMPLING_RATE);
  outFilter.set_input_sampling_rate(SAMPLING_RATE);
  outFilter.set_output_sampling_rate(SAMPLING_RATE);

  highToneControlFilter.set_cut_frequency(1000);
  highToneControlFilter.set_gain(2);
  highToneControlFilter.set_Q(.25);

  for(gsl::index i = 0; i < PROCESSSIZE; i += 1024)
  {
    outFilter.process(1024);
  }

  std::ofstream out(argv[1]);
  for(size_t i = 0; i < PROCESSSIZE; ++i)
  {
    out << input[i] << "\t" << output[i] << std::endl;
  }
}
