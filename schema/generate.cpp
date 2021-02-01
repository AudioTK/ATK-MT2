
#include <ATK/Core/InPointerFilter.h>
#include <ATK/Core/OutPointerFilter.h>
#include <ATK/Modelling/ModellerFilter.h>

#include <boost/math/constants/constants.hpp>

#include <fstream>
#include <memory>
#include <vector>

constexpr gsl::index PROCESSSIZE = 4 * 1024 * 1024;
constexpr size_t SAMPLING_RATE = 48000;

extern "C"
{
  std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter();
}

int main(int argc, const char** argv)
{
  std::vector<double> input(PROCESSSIZE);
  std::vector<double> output(PROCESSSIZE);
  for(size_t i = 0; i < PROCESSSIZE; ++i)
  {
    auto frequency = (20. + i) / PROCESSSIZE * (20000 - 20);
    input[i] = std::sin(i * boost::math::constants::pi<double>() * (frequency / SAMPLING_RATE));
  }

  ATK::InPointerFilter<double> generator(input.data(), 1, PROCESSSIZE, false);
  generator.set_output_sampling_rate(SAMPLING_RATE);

  std::unique_ptr<ATK::ModellerFilter<double>> filter = createStaticFilter();
  for(gsl::index i = 0; i < filter->get_number_parameters(); ++i)
  {
    filter->set_parameter(i, 0.5);
  }
  filter->set_input_sampling_rate(SAMPLING_RATE);
  filter->set_output_sampling_rate(SAMPLING_RATE);
  filter->set_input_port(filter->find_input_pin("vin"), &generator, 0);

  ATK::OutPointerFilter<double> sink(output.data(), 1, PROCESSSIZE, false);
  sink.set_input_sampling_rate(SAMPLING_RATE);
  sink.set_input_port(0, filter.get(), filter->find_dynamic_pin("vout"));

  sink.process(PROCESSSIZE);

  std::ofstream out(argv[1]);
  for(size_t i = 0; i < PROCESSSIZE; ++i)
  {
    out << input[i] << "\t" << output[i] << std::endl;
  }
}
