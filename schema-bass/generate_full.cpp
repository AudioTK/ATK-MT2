#include "../MTB/Source/static_elements.h"

#include <ATK/Core/InPointerFilter.h>
#include <ATK/Core/OutPointerFilter.h>
#include <ATK/EQ/ButterworthFilter.h>
#include <ATK/EQ/IIRFilter.h>
#include <ATK/Modelling/ModellerFilter.h>
#include <ATK/Tools/DecimationFilter.h>
#include <ATK/Tools/OversamplingFilter.h>

#include <boost/math/constants/constants.hpp>

#include <fstream>
#include <memory>
#include <vector>

constexpr gsl::index PROCESSSIZE = 4 * 1024 * 1024;
constexpr size_t SAMPLING_RATE = 96000;
constexpr size_t OVERSAMPLING = 8;

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
  std::unique_ptr<ATK::ModellerFilter<double>> highPassFilter = MTB::createStaticFilter_stage1();
  ATK::OversamplingFilter<double, ATK::Oversampling6points5order_8<double>> oversamplingFilter;
  std::unique_ptr<ATK::ModellerFilter<double>> preDistortionToneShapingFilter = MTB::createStaticFilter_stage2();
  std::unique_ptr<ATK::ModellerFilter<double>> bandPassFilter = MTB::createStaticFilter_stage3();
  std::unique_ptr<ATK::ModellerFilter<double>> distLevelFilter = MTB::createStaticFilter_stage4();
  std::unique_ptr<ATK::ModellerFilter<double>> distFilter = MTB::createStaticFilter_stage5();
  std::unique_ptr<ATK::ModellerFilter<double>> postDistortionToneShapingFilter = MTB::createStaticFilter_stage6();
  ATK::IIRFilter<ATK::ButterworthLowPassCoefficients<double>> lowpassFilter;
  ATK::DecimationFilter<double> decimationFilter;
  ATK::OutPointerFilter<double> outFilter(output.data(), 1, PROCESSSIZE, false);

  highPassFilter->set_input_port(highPassFilter->find_input_pin("vin"), &inFilter, 0);
  oversamplingFilter.set_input_port(0, highPassFilter.get(), highPassFilter->find_dynamic_pin("vout"));
  preDistortionToneShapingFilter->set_input_port(
      preDistortionToneShapingFilter->find_input_pin("vin"), &oversamplingFilter, 0);
  bandPassFilter->set_input_port(bandPassFilter->find_input_pin("vin"),
      preDistortionToneShapingFilter.get(),
      preDistortionToneShapingFilter->find_dynamic_pin("vout"));
  distLevelFilter->set_input_port(
      distLevelFilter->find_input_pin("vin"), bandPassFilter.get(), bandPassFilter->find_dynamic_pin("vout"));
  distFilter->set_input_port(
      distFilter->find_input_pin("vin"), distLevelFilter.get(), distLevelFilter->find_dynamic_pin("vout"));
  postDistortionToneShapingFilter->set_input_port(
      postDistortionToneShapingFilter->find_input_pin("vin"), distFilter.get(), distFilter->find_dynamic_pin("vout"));
  lowpassFilter.set_input_port(
      0, postDistortionToneShapingFilter.get(), postDistortionToneShapingFilter->find_dynamic_pin("vout"));
  decimationFilter.set_input_port(0, &lowpassFilter, 0);
  outFilter.set_input_port(0, &decimationFilter, 0);

  lowpassFilter.set_cut_frequency(20000);
  lowpassFilter.set_order(6);

  inFilter.set_input_sampling_rate(SAMPLING_RATE);
  inFilter.set_output_sampling_rate(SAMPLING_RATE);
  highPassFilter->set_input_sampling_rate(SAMPLING_RATE);
  highPassFilter->set_output_sampling_rate(SAMPLING_RATE);
  oversamplingFilter.set_input_sampling_rate(SAMPLING_RATE);
  oversamplingFilter.set_output_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  preDistortionToneShapingFilter->set_input_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  preDistortionToneShapingFilter->set_output_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  bandPassFilter->set_input_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  bandPassFilter->set_output_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  distLevelFilter->set_input_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  distLevelFilter->set_output_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  distFilter->set_input_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  distFilter->set_output_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  postDistortionToneShapingFilter->set_input_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  postDistortionToneShapingFilter->set_output_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  lowpassFilter.set_input_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  lowpassFilter.set_output_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  decimationFilter.set_input_sampling_rate(SAMPLING_RATE * OVERSAMPLING);
  decimationFilter.set_output_sampling_rate(SAMPLING_RATE);
  outFilter.set_input_sampling_rate(SAMPLING_RATE);
  outFilter.set_output_sampling_rate(SAMPLING_RATE);

  distLevelFilter->set_parameter(0, 0.1);

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
