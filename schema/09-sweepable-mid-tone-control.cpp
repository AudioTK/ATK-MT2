#include <cstdlib>
#include <memory>

#include <ATK/Core/Utilities.h>
#include <ATK/Modelling/ModellerFilter.h>
#include <ATK/Modelling/StaticComponent/StaticCapacitor.h>
#include <ATK/Modelling/StaticComponent/StaticCoil.h>
#include <ATK/Modelling/StaticComponent/StaticCurrent.h>
#include <ATK/Modelling/StaticComponent/StaticDiode.h>
#include <ATK/Modelling/StaticComponent/StaticResistor.h>
#include <ATK/Modelling/StaticComponent/StaticTransistor.h>

#include <Eigen/Eigen>

constexpr gsl::index MAX_ITERATION = 10;
constexpr gsl::index MAX_ITERATION_STEADY_STATE = 200;

constexpr gsl::index INIT_WARMUP = 10;
constexpr double EPS = 1e-8;
constexpr double MAX_DELTA = 1e-1;

namespace
{
using namespace ATK;

class StaticFilter: public ATK::ModellerFilter<double>
{
  using typename ATK::TypedBaseFilter<double>::DataType;
  bool initialized = false;

  Eigen::Matrix<DataType, 2, 1> static_state{Eigen::Matrix<DataType, 2, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 1, 1> input_state{Eigen::Matrix<DataType, 1, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 14, 1> dynamic_state{Eigen::Matrix<DataType, 14, 1>::Zero()};
  StaticCapacitor<DataType> c011{1e-06};
  StaticResistor<DataType> r038{47000};
  StaticResistor<DataType> r045{47000};
  StaticCapacitor<DataType> c026{1e-10};
  StaticResistor<DataType> r050{330};
  StaticCapacitor<DataType> c005{1e-05};
  StaticCapacitor<DataType> c037{1e-06};
  StaticResistor<DataType> r049{330};
  StaticResistor<DataType> r039{0.001};
  StaticCapacitor<DataType> c038{1e-07};
  StaticCapacitor<DataType> c043{8.2e-09};
  DataType pr2aa{50000};
  DataType pr2aa_trimmer{0};
  StaticResistor<DataType> r062{2200};
  DataType pr2ab{50000};
  DataType pr2ab_trimmer{0};
  StaticResistor<DataType> r048{2200};
  StaticCapacitor<DataType> c036{2.2e-08};
  DataType pr02{100000};
  DataType pr02_trimmer{0};

public:
  StaticFilter(): ModellerFilter<DataType>(14, 1)
  {
    static_state << 0.000000, -4.500000;
  }

  gsl::index get_nb_dynamic_pins() const override
  {
    return 14;
  }

  gsl::index get_nb_input_pins() const override
  {
    return 1;
  }

  gsl::index get_nb_static_pins() const override
  {
    return 2;
  }

  Eigen::Matrix<DataType, Eigen::Dynamic, 1> get_static_state() const override
  {
    return static_state;
  }

  gsl::index get_nb_components() const override
  {
    return 19;
  }

  std::string get_dynamic_pin_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 12:
      return "11";
    case 13:
      return "9";
    case 0:
      return "2";
    case 8:
      return "14";
    case 9:
      return "15";
    case 5:
      return "vout";
    case 4:
      return "6";
    case 6:
      return "7";
    case 3:
      return "5";
    case 11:
      return "12";
    case 10:
      return "13";
    case 7:
      return "8";
    case 2:
      return "4";
    case 1:
      return "3";
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  std::string get_input_pin_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 0:
      return "vin";
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  std::string get_static_pin_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 1:
      return "vdd";
    case 0:
      return "0";
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  gsl::index get_number_parameters() const override
  {
    return 0;
  }

  std::string get_parameter_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 0:
    {
      return "pr2aa";
    }
    case 1:
    {
      return "pr2ab";
    }
    case 2:
    {
      return "pr02";
    }
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  DataType get_parameter(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 0:
    {
      return pr2aa_trimmer;
    }
    case 1:
    {
      return pr2ab_trimmer;
    }
    case 2:
    {
      return pr02_trimmer;
    }
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  void set_parameter(gsl::index identifier, DataType value) override
  {
    switch(identifier)
    {
    case 0:
    {
      pr2aa_trimmer = value;
      break;
    }
    case 1:
    {
      pr2ab_trimmer = value;
      break;
    }
    case 2:
    {
      pr02_trimmer = value;
      break;
    }
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  /// Setup the inner state of the filter, slowly incrementing the static state
  void setup() override
  {
    assert(input_sampling_rate == output_sampling_rate);

    if(!initialized)
    {
      setup_inverse<true>();
      auto target_static_state = static_state;

      for(gsl::index i = 0; i < INIT_WARMUP; ++i)
      {
        static_state = target_static_state * ((i + 1.) / INIT_WARMUP);
        init();
      }
      static_state = target_static_state;
    }
    setup_inverse<false>();
  }

  template <bool steady_state>
  void setup_inverse()
  {
  }

  void init()
  {
    // update_steady_state
    c011.update_steady_state(1. / input_sampling_rate, input_state[0], dynamic_state[0]);
    c026.update_steady_state(1. / input_sampling_rate, dynamic_state[1], dynamic_state[3]);
    c005.update_steady_state(1. / input_sampling_rate, dynamic_state[3], dynamic_state[5]);
    c037.update_steady_state(1. / input_sampling_rate, dynamic_state[3], dynamic_state[6]);
    c038.update_steady_state(1. / input_sampling_rate, dynamic_state[2], dynamic_state[8]);
    c043.update_steady_state(1. / input_sampling_rate, dynamic_state[8], static_state[0]);
    c036.update_steady_state(1. / input_sampling_rate, dynamic_state[11], dynamic_state[12]);

    solve<true>();

    // update_steady_state
    c011.update_steady_state(1. / input_sampling_rate, input_state[0], dynamic_state[0]);
    c026.update_steady_state(1. / input_sampling_rate, dynamic_state[1], dynamic_state[3]);
    c005.update_steady_state(1. / input_sampling_rate, dynamic_state[3], dynamic_state[5]);
    c037.update_steady_state(1. / input_sampling_rate, dynamic_state[3], dynamic_state[6]);
    c038.update_steady_state(1. / input_sampling_rate, dynamic_state[2], dynamic_state[8]);
    c043.update_steady_state(1. / input_sampling_rate, dynamic_state[8], static_state[0]);
    c036.update_steady_state(1. / input_sampling_rate, dynamic_state[11], dynamic_state[12]);

    initialized = true;
  }

  void process_impl(gsl::index size) const override
  {
    for(gsl::index i = 0; i < size; ++i)
    {
      for(gsl::index j = 0; j < nb_input_ports; ++j)
      {
        input_state[j] = converted_inputs[j][i];
      }

      solve<false>();

      // Update state
      c011.update_state(input_state[0], dynamic_state[0]);
      c026.update_state(dynamic_state[1], dynamic_state[3]);
      c005.update_state(dynamic_state[3], dynamic_state[5]);
      c037.update_state(dynamic_state[3], dynamic_state[6]);
      c038.update_state(dynamic_state[2], dynamic_state[8]);
      c043.update_state(dynamic_state[8], static_state[0]);
      c036.update_state(dynamic_state[11], dynamic_state[12]);
      for(gsl::index j = 0; j < nb_output_ports; ++j)
      {
        outputs[j][i] = dynamic_state[j];
      }
    }
  }

  /// Solve for steady state and non steady state the system
  template <bool steady_state>
  void solve() const
  {
    gsl::index iteration = 0;

    constexpr int current_max_iter = steady_state ? MAX_ITERATION_STEADY_STATE : MAX_ITERATION;

    while(iteration < current_max_iter && !iterate<steady_state>())
    {
      ++iteration;
    }
  }

  template <bool steady_state>
  bool iterate() const
  {
    // Static states
    auto s0_ = static_state[0];
    auto s1_ = static_state[1];

    // Input states
    auto i0_ = input_state[0];

    // Dynamic states
    auto d0_ = dynamic_state[0];
    auto d1_ = dynamic_state[1];
    auto d2_ = dynamic_state[2];
    auto d3_ = dynamic_state[3];
    auto d4_ = dynamic_state[4];
    auto d5_ = dynamic_state[5];
    auto d6_ = dynamic_state[6];
    auto d7_ = dynamic_state[7];
    auto d8_ = dynamic_state[8];
    auto d9_ = dynamic_state[9];
    auto d10_ = dynamic_state[10];
    auto d11_ = dynamic_state[11];
    auto d12_ = dynamic_state[12];
    auto d13_ = dynamic_state[13];

    // Precomputes

    Eigen::Matrix<DataType, 14, 1> eqs(Eigen::Matrix<DataType, 14, 1>::Zero());
    auto eq0
        = -(steady_state ? 0 : c011.get_current(i0_, d0_)) + r038.get_current(d0_, d1_) + r050.get_current(d0_, d4_);
    auto eq1
        = -r038.get_current(d0_, d1_) + r045.get_current(d1_, d3_) + (steady_state ? 0 : c026.get_current(d1_, d3_));
    auto eq2 = +r039.get_current(d2_, s0_) + (steady_state ? 0 : c038.get_current(d2_, d8_));
    auto eq3 = dynamic_state[2] - dynamic_state[1];
    auto eq4 = -r050.get_current(d0_, d4_) + (pr02_trimmer != 0 ? (d13_ - d4_) / (pr02_trimmer * pr02) : 0);
    auto eq5 = -(steady_state ? 0 : c005.get_current(d3_, d5_));
    auto eq6 = -(steady_state ? 0 : c037.get_current(d3_, d6_)) + r049.get_current(d6_, d7_);
    auto eq7 = -r049.get_current(d6_, d7_) + (pr02_trimmer != 1 ? (d13_ - d7_) / ((1 - pr02_trimmer) * pr02) : 0);
    auto eq8 = -(steady_state ? 0 : c038.get_current(d2_, d8_)) + (steady_state ? 0 : c043.get_current(d8_, s0_))
             + (pr2aa_trimmer != 1 ? (d9_ - d8_) / ((1 - pr2aa_trimmer) * pr2aa) : 0)
             + (pr2ab_trimmer != 1 ? (d10_ - d8_) / ((1 - pr2ab_trimmer) * pr2ab) : 0);
    auto eq9 = +(pr2aa_trimmer != 1 ? (d8_ - d9_) / ((1 - pr2aa_trimmer) * pr2aa) : 0) + r062.get_current(d9_, s1_);
    auto eq10 = +(pr2ab_trimmer != 1 ? (d8_ - d10_) / ((1 - pr2ab_trimmer) * pr2ab) : 0) - r048.get_current(d11_, d10_);
    auto eq11 = +r048.get_current(d11_, d10_) + (steady_state ? 0 : c036.get_current(d11_, d12_));
    auto eq12 = dynamic_state[13] - dynamic_state[12];
    auto eq13 = +(pr02_trimmer != 0 ? (d4_ - d13_) / (pr02_trimmer * pr02) : 0)
              + (pr02_trimmer != 1 ? (d7_ - d13_) / ((1 - pr02_trimmer) * pr02) : 0);
    eqs << eq0, eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, eq10, eq11, eq12, eq13;

    // Check if the equations have converged
    if((eqs.array().abs() < EPS).all())
    {
      return true;
    }

    Eigen::Matrix<DataType, 14, 14> jacobian(Eigen::Matrix<DataType, 14, 14>::Zero());
    auto jac0_0 = 0 - (steady_state ? 0 : c011.get_gradient()) - r038.get_gradient() - r050.get_gradient();
    auto jac0_1 = 0 + r038.get_gradient();
    auto jac0_2 = 0;
    auto jac0_3 = 0;
    auto jac0_4 = 0 + r050.get_gradient();
    auto jac0_5 = 0;
    auto jac0_6 = 0;
    auto jac0_7 = 0;
    auto jac0_8 = 0;
    auto jac0_9 = 0;
    auto jac0_10 = 0;
    auto jac0_11 = 0;
    auto jac0_12 = 0;
    auto jac0_13 = 0;
    auto jac1_0 = 0 + r038.get_gradient();
    auto jac1_1 = 0 - r038.get_gradient() - r045.get_gradient() - (steady_state ? 0 : c026.get_gradient());
    auto jac1_2 = 0;
    auto jac1_3 = 0 + r045.get_gradient() + (steady_state ? 0 : c026.get_gradient());
    auto jac1_4 = 0;
    auto jac1_5 = 0;
    auto jac1_6 = 0;
    auto jac1_7 = 0;
    auto jac1_8 = 0;
    auto jac1_9 = 0;
    auto jac1_10 = 0;
    auto jac1_11 = 0;
    auto jac1_12 = 0;
    auto jac1_13 = 0;
    auto jac2_0 = 0;
    auto jac2_1 = 0;
    auto jac2_2 = 0 - r039.get_gradient() - (steady_state ? 0 : c038.get_gradient());
    auto jac2_3 = 0;
    auto jac2_4 = 0;
    auto jac2_5 = 0;
    auto jac2_6 = 0;
    auto jac2_7 = 0;
    auto jac2_8 = 0 + (steady_state ? 0 : c038.get_gradient());
    auto jac2_9 = 0;
    auto jac2_10 = 0;
    auto jac2_11 = 0;
    auto jac2_12 = 0;
    auto jac2_13 = 0;
    auto jac3_0 = 0;
    auto jac3_1 = 0 + -1;
    auto jac3_2 = 0 + 1;
    auto jac3_3 = 0;
    auto jac3_4 = 0;
    auto jac3_5 = 0;
    auto jac3_6 = 0;
    auto jac3_7 = 0;
    auto jac3_8 = 0;
    auto jac3_9 = 0;
    auto jac3_10 = 0;
    auto jac3_11 = 0;
    auto jac3_12 = 0;
    auto jac3_13 = 0;
    auto jac4_0 = 0 + r050.get_gradient();
    auto jac4_1 = 0;
    auto jac4_2 = 0;
    auto jac4_3 = 0;
    auto jac4_4 = 0 - r050.get_gradient() + (pr02_trimmer != 0 ? -1 / (pr02_trimmer * pr02) : 0);
    auto jac4_5 = 0;
    auto jac4_6 = 0;
    auto jac4_7 = 0;
    auto jac4_8 = 0;
    auto jac4_9 = 0;
    auto jac4_10 = 0;
    auto jac4_11 = 0;
    auto jac4_12 = 0;
    auto jac4_13 = 0 + (pr02_trimmer != 0 ? 1 / (pr02_trimmer * pr02) : 0);
    auto jac5_0 = 0;
    auto jac5_1 = 0;
    auto jac5_2 = 0;
    auto jac5_3 = 0 + (steady_state ? 0 : c005.get_gradient());
    auto jac5_4 = 0;
    auto jac5_5 = 0 - (steady_state ? 0 : c005.get_gradient());
    auto jac5_6 = 0;
    auto jac5_7 = 0;
    auto jac5_8 = 0;
    auto jac5_9 = 0;
    auto jac5_10 = 0;
    auto jac5_11 = 0;
    auto jac5_12 = 0;
    auto jac5_13 = 0;
    auto jac6_0 = 0;
    auto jac6_1 = 0;
    auto jac6_2 = 0;
    auto jac6_3 = 0 + (steady_state ? 0 : c037.get_gradient());
    auto jac6_4 = 0;
    auto jac6_5 = 0;
    auto jac6_6 = 0 - (steady_state ? 0 : c037.get_gradient()) - r049.get_gradient();
    auto jac6_7 = 0 + r049.get_gradient();
    auto jac6_8 = 0;
    auto jac6_9 = 0;
    auto jac6_10 = 0;
    auto jac6_11 = 0;
    auto jac6_12 = 0;
    auto jac6_13 = 0;
    auto jac7_0 = 0;
    auto jac7_1 = 0;
    auto jac7_2 = 0;
    auto jac7_3 = 0;
    auto jac7_4 = 0;
    auto jac7_5 = 0;
    auto jac7_6 = 0 + r049.get_gradient();
    auto jac7_7 = 0 - r049.get_gradient() + (pr02_trimmer != 1 ? -1 / ((1 - pr02_trimmer) * pr02) : 0);
    auto jac7_8 = 0;
    auto jac7_9 = 0;
    auto jac7_10 = 0;
    auto jac7_11 = 0;
    auto jac7_12 = 0;
    auto jac7_13 = 0 + (pr02_trimmer != 1 ? 1 / ((1 - pr02_trimmer) * pr02) : 0);
    auto jac8_0 = 0;
    auto jac8_1 = 0;
    auto jac8_2 = 0 + (steady_state ? 0 : c038.get_gradient());
    auto jac8_3 = 0;
    auto jac8_4 = 0;
    auto jac8_5 = 0;
    auto jac8_6 = 0;
    auto jac8_7 = 0;
    auto jac8_8 = 0 - (steady_state ? 0 : c038.get_gradient()) - (steady_state ? 0 : c043.get_gradient())
                + (pr2aa_trimmer != 1 ? -1 / ((1 - pr2aa_trimmer) * pr2aa) : 0)
                + (pr2ab_trimmer != 1 ? -1 / ((1 - pr2ab_trimmer) * pr2ab) : 0);
    auto jac8_9 = 0 + (pr2aa_trimmer != 1 ? 1 / ((1 - pr2aa_trimmer) * pr2aa) : 0);
    auto jac8_10 = 0 + (pr2ab_trimmer != 1 ? 1 / ((1 - pr2ab_trimmer) * pr2ab) : 0);
    auto jac8_11 = 0;
    auto jac8_12 = 0;
    auto jac8_13 = 0;
    auto jac9_0 = 0;
    auto jac9_1 = 0;
    auto jac9_2 = 0;
    auto jac9_3 = 0;
    auto jac9_4 = 0;
    auto jac9_5 = 0;
    auto jac9_6 = 0;
    auto jac9_7 = 0;
    auto jac9_8 = 0;
    auto jac9_9 = 0 - r062.get_gradient();
    auto jac9_10 = 0;
    auto jac9_11 = 0;
    auto jac9_12 = 0;
    auto jac9_13 = 0;
    auto jac10_0 = 0;
    auto jac10_1 = 0;
    auto jac10_2 = 0;
    auto jac10_3 = 0;
    auto jac10_4 = 0;
    auto jac10_5 = 0;
    auto jac10_6 = 0;
    auto jac10_7 = 0;
    auto jac10_8 = 0;
    auto jac10_9 = 0;
    auto jac10_10 = 0 - r048.get_gradient();
    auto jac10_11 = 0 + r048.get_gradient();
    auto jac10_12 = 0;
    auto jac10_13 = 0;
    auto jac11_0 = 0;
    auto jac11_1 = 0;
    auto jac11_2 = 0;
    auto jac11_3 = 0;
    auto jac11_4 = 0;
    auto jac11_5 = 0;
    auto jac11_6 = 0;
    auto jac11_7 = 0;
    auto jac11_8 = 0;
    auto jac11_9 = 0;
    auto jac11_10 = 0 + r048.get_gradient();
    auto jac11_11 = 0 - r048.get_gradient() - (steady_state ? 0 : c036.get_gradient());
    auto jac11_12 = 0 + (steady_state ? 0 : c036.get_gradient());
    auto jac11_13 = 0;
    auto jac12_0 = 0;
    auto jac12_1 = 0;
    auto jac12_2 = 0;
    auto jac12_3 = 0;
    auto jac12_4 = 0;
    auto jac12_5 = 0;
    auto jac12_6 = 0;
    auto jac12_7 = 0;
    auto jac12_8 = 0;
    auto jac12_9 = 0;
    auto jac12_10 = 0;
    auto jac12_11 = 0;
    auto jac12_12 = 0 + -1;
    auto jac12_13 = 0 + 1;
    auto jac13_0 = 0;
    auto jac13_1 = 0;
    auto jac13_2 = 0;
    auto jac13_3 = 0;
    auto jac13_4 = 0 + (pr02_trimmer != 0 ? 1 / (pr02_trimmer * pr02) : 0);
    auto jac13_5 = 0;
    auto jac13_6 = 0;
    auto jac13_7 = 0 + (pr02_trimmer != 1 ? 1 / ((1 - pr02_trimmer) * pr02) : 0);
    auto jac13_8 = 0;
    auto jac13_9 = 0;
    auto jac13_10 = 0;
    auto jac13_11 = 0;
    auto jac13_12 = 0;
    auto jac13_13 = 0 + (pr02_trimmer != 0 ? -1 / (pr02_trimmer * pr02) : 0)
                  + (pr02_trimmer != 1 ? -1 / ((1 - pr02_trimmer) * pr02) : 0);
    jacobian << jac0_0, jac0_1, jac0_2, jac0_3, jac0_4, jac0_5, jac0_6, jac0_7, jac0_8, jac0_9, jac0_10, jac0_11,
        jac0_12, jac0_13, jac1_0, jac1_1, jac1_2, jac1_3, jac1_4, jac1_5, jac1_6, jac1_7, jac1_8, jac1_9, jac1_10,
        jac1_11, jac1_12, jac1_13, jac2_0, jac2_1, jac2_2, jac2_3, jac2_4, jac2_5, jac2_6, jac2_7, jac2_8, jac2_9,
        jac2_10, jac2_11, jac2_12, jac2_13, jac3_0, jac3_1, jac3_2, jac3_3, jac3_4, jac3_5, jac3_6, jac3_7, jac3_8,
        jac3_9, jac3_10, jac3_11, jac3_12, jac3_13, jac4_0, jac4_1, jac4_2, jac4_3, jac4_4, jac4_5, jac4_6, jac4_7,
        jac4_8, jac4_9, jac4_10, jac4_11, jac4_12, jac4_13, jac5_0, jac5_1, jac5_2, jac5_3, jac5_4, jac5_5, jac5_6,
        jac5_7, jac5_8, jac5_9, jac5_10, jac5_11, jac5_12, jac5_13, jac6_0, jac6_1, jac6_2, jac6_3, jac6_4, jac6_5,
        jac6_6, jac6_7, jac6_8, jac6_9, jac6_10, jac6_11, jac6_12, jac6_13, jac7_0, jac7_1, jac7_2, jac7_3, jac7_4,
        jac7_5, jac7_6, jac7_7, jac7_8, jac7_9, jac7_10, jac7_11, jac7_12, jac7_13, jac8_0, jac8_1, jac8_2, jac8_3,
        jac8_4, jac8_5, jac8_6, jac8_7, jac8_8, jac8_9, jac8_10, jac8_11, jac8_12, jac8_13, jac9_0, jac9_1, jac9_2,
        jac9_3, jac9_4, jac9_5, jac9_6, jac9_7, jac9_8, jac9_9, jac9_10, jac9_11, jac9_12, jac9_13, jac10_0, jac10_1,
        jac10_2, jac10_3, jac10_4, jac10_5, jac10_6, jac10_7, jac10_8, jac10_9, jac10_10, jac10_11, jac10_12, jac10_13,
        jac11_0, jac11_1, jac11_2, jac11_3, jac11_4, jac11_5, jac11_6, jac11_7, jac11_8, jac11_9, jac11_10, jac11_11,
        jac11_12, jac11_13, jac12_0, jac12_1, jac12_2, jac12_3, jac12_4, jac12_5, jac12_6, jac12_7, jac12_8, jac12_9,
        jac12_10, jac12_11, jac12_12, jac12_13, jac13_0, jac13_1, jac13_2, jac13_3, jac13_4, jac13_5, jac13_6, jac13_7,
        jac13_8, jac13_9, jac13_10, jac13_11, jac13_12, jac13_13;
    Eigen::Matrix<DataType, 14, 1> delta = jacobian.colPivHouseholderQr().solve(eqs);

    // Check if the update is big enough
    if((delta.array().abs() < EPS).all())
    {
      return true;
    }

    // Big variations are only in steady state mode
    if(steady_state)
    {
      auto max_delta = delta.array().abs().maxCoeff();
      if(max_delta > MAX_DELTA)
      {
        delta *= MAX_DELTA / max_delta;
      }
    }

    dynamic_state -= delta;

    return false;
  }

  void populate_sparse_jacobian(Eigen::SparseMatrix<DataType, Eigen::ColMajor>& sparse_jacobian)
  {
    std::vector<Eigen::Triplet<double>> coefficients{{0, 0},
        {0, 1},
        {0, 4},
        {1, 0},
        {1, 1},
        {1, 3},
        {2, 2},
        {2, 8},
        {3, 1},
        {3, 2},
        {4, 0},
        {4, 4},
        {4, 13},
        {5, 3},
        {5, 5},
        {6, 3},
        {6, 6},
        {6, 7},
        {7, 6},
        {7, 7},
        {7, 13},
        {8, 2},
        {8, 8},
        {8, 9},
        {8, 10},
        {9, 9},
        {10, 10},
        {10, 11},
        {11, 10},
        {11, 11},
        {11, 12},
        {12, 12},
        {12, 13},
        {13, 4},
        {13, 7},
        {13, 13}};
    sparse_jacobian.setFromTriplets(coefficients.begin(), coefficients.end());
    sparse_jacobian.makeCompressed();
  }
};
} // namespace

extern "C"
{
  std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter()
  {
    return std::make_unique<StaticFilter>();
  }
} // namespace
