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

namespace
{
constexpr gsl::index MAX_ITERATION = 1;
constexpr gsl::index MAX_ITERATION_STEADY_STATE = 1;

constexpr gsl::index INIT_WARMUP = 1;
constexpr double EPS = 1e-8;
constexpr double MAX_DELTA = 1e-1;

class StaticFilter: public ATK::ModellerFilter<double>
{
  using typename ATK::TypedBaseFilter<double>::DataType;
  bool initialized = false;

  Eigen::Matrix<DataType, 1, 1> static_state{Eigen::Matrix<DataType, 1, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 1, 1> input_state{Eigen::Matrix<DataType, 1, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 9, 1> dynamic_state{Eigen::Matrix<DataType, 9, 1>::Zero()};
  ATK::StaticResistor<DataType> r014{22000};
  ATK::StaticCapacitor<DataType> c010{1e-11};
  ATK::StaticResistor<DataType> r015{22000};
  DataType pr03a{100000};
  DataType pr03a_trimmer{0};
  ATK::StaticResistor<DataType> r061{2200};
  ATK::StaticCapacitor<DataType> c044{1e-08};
  DataType pr03b{100000};
  DataType pr03b_trimmer{0};
  ATK::StaticCapacitor<DataType> c008{2.2e-07};
  ATK::StaticResistor<DataType> r012{2200};
  ATK::StaticCapacitor<DataType> c009{4.7e-08};
  ATK::StaticResistor<DataType> r013{1e+06};

public:
  StaticFilter(): ModellerFilter<DataType>(9, 1)
  {
    static_state << 0.000000;
  }

  gsl::index get_nb_dynamic_pins() const override
  {
    return 9;
  }

  gsl::index get_nb_input_pins() const override
  {
    return 1;
  }

  gsl::index get_nb_static_pins() const override
  {
    return 1;
  }

  Eigen::Matrix<DataType, Eigen::Dynamic, 1> get_static_state() const override
  {
    return static_state;
  }

  gsl::index get_nb_components() const override
  {
    return 13;
  }

  std::string get_dynamic_pin_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 8:
      return "9";
    case 4:
      return "6";
    case 5:
      return "7";
    case 3:
      return "5";
    case 2:
      return "vout";
    case 7:
      return "10";
    case 6:
      return "8";
    case 1:
      return "4";
    case 0:
      return "2";
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
    case 0:
      return "0";
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  gsl::index get_number_parameters() const override
  {
    return 2;
  }

  std::string get_parameter_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 0:
    {
      return "pr03a";
    }
    case 1:
    {
      return "pr03b";
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
      return pr03a_trimmer;
    }
    case 1:
    {
      return pr03b_trimmer;
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
      pr03a_trimmer = value;
      break;
    }
    case 1:
    {
      pr03b_trimmer = value;
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
    c010.update_steady_state(1. / input_sampling_rate, dynamic_state[1], dynamic_state[2]);
    c044.update_steady_state(1. / input_sampling_rate, dynamic_state[4], static_state[0]);
    c008.update_steady_state(1. / input_sampling_rate, dynamic_state[5], dynamic_state[6]);
    c009.update_steady_state(1. / input_sampling_rate, dynamic_state[6], dynamic_state[8]);

    solve<true>();

    // update_steady_state
    c010.update_steady_state(1. / input_sampling_rate, dynamic_state[1], dynamic_state[2]);
    c044.update_steady_state(1. / input_sampling_rate, dynamic_state[4], static_state[0]);
    c008.update_steady_state(1. / input_sampling_rate, dynamic_state[5], dynamic_state[6]);
    c009.update_steady_state(1. / input_sampling_rate, dynamic_state[6], dynamic_state[8]);

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
      c010.update_state(dynamic_state[1], dynamic_state[2]);
      c044.update_state(dynamic_state[4], static_state[0]);
      c008.update_state(dynamic_state[5], dynamic_state[6]);
      c009.update_state(dynamic_state[6], dynamic_state[8]);
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

    // Precomputes

    Eigen::Matrix<DataType, 9, 1> eqs(Eigen::Matrix<DataType, 9, 1>::Zero());
    auto eq0 = -r014.get_current(i0_, d0_) + (pr03a_trimmer != 0 ? (d3_ - d0_) / (pr03a_trimmer * pr03a) : 0)
             + (pr03b_trimmer != 0 ? (d5_ - d0_) / (pr03b_trimmer * pr03b) : 0);
    auto eq1 = +(steady_state ? 0 : c010.get_current(d1_, d2_)) + r015.get_current(d1_, d2_)
             + (pr03a_trimmer != 1 ? (d3_ - d1_) / ((1 - pr03a_trimmer) * pr03a) : 0)
             + (pr03b_trimmer != 1 ? (d5_ - d1_) / ((1 - pr03b_trimmer) * pr03b) : 0);
    auto eq2 = dynamic_state[0] - dynamic_state[1];
    auto eq3 = +(pr03a_trimmer != 0 ? (d0_ - d3_) / (pr03a_trimmer * pr03a) : 0)
             + (pr03a_trimmer != 1 ? (d1_ - d3_) / ((1 - pr03a_trimmer) * pr03a) : 0) + r061.get_current(d3_, d4_);
    auto eq4 = -r061.get_current(d3_, d4_) + (steady_state ? 0 : c044.get_current(d4_, s0_));
    auto eq5 = +(pr03b_trimmer != 0 ? (d0_ - d5_) / (pr03b_trimmer * pr03b) : 0)
             + (pr03b_trimmer != 1 ? (d1_ - d5_) / ((1 - pr03b_trimmer) * pr03b) : 0)
             + (steady_state ? 0 : c008.get_current(d5_, d6_));
    auto eq6 = -(steady_state ? 0 : c008.get_current(d5_, d6_)) + r012.get_current(d6_, d7_)
             + (steady_state ? 0 : c009.get_current(d6_, d8_));
    auto eq7 = dynamic_state[8] - dynamic_state[7];
    auto eq8 = -(steady_state ? 0 : c009.get_current(d6_, d8_)) + r013.get_current(d8_, s0_);
    eqs << eq0, eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8;

    // Check if the equations have converged
    if((eqs.array().abs() < EPS).all())
    {
      return true;
    }

    auto jac0_0 = 0 - r014.get_gradient() + (pr03a_trimmer != 0 ? -1 / (pr03a_trimmer * pr03a) : 0)
                + (pr03b_trimmer != 0 ? -1 / (pr03b_trimmer * pr03b) : 0);
    auto jac0_1 = 0;
    auto jac0_2 = 0;
    auto jac0_3 = 0 + (pr03a_trimmer != 0 ? 1 / (pr03a_trimmer * pr03a) : 0);
    auto jac0_4 = 0;
    auto jac0_5 = 0 + (pr03b_trimmer != 0 ? 1 / (pr03b_trimmer * pr03b) : 0);
    auto jac0_6 = 0;
    auto jac0_7 = 0;
    auto jac0_8 = 0;
    auto jac1_0 = 0;
    auto jac1_1 = 0 - (steady_state ? 0 : c010.get_gradient()) - r015.get_gradient()
                + (pr03a_trimmer != 1 ? -1 / ((1 - pr03a_trimmer) * pr03a) : 0)
                + (pr03b_trimmer != 1 ? -1 / ((1 - pr03b_trimmer) * pr03b) : 0);
    auto jac1_2 = 0 + (steady_state ? 0 : c010.get_gradient()) + r015.get_gradient();
    auto jac1_3 = 0 + (pr03a_trimmer != 1 ? 1 / ((1 - pr03a_trimmer) * pr03a) : 0);
    auto jac1_4 = 0;
    auto jac1_5 = 0 + (pr03b_trimmer != 1 ? 1 / ((1 - pr03b_trimmer) * pr03b) : 0);
    auto jac1_6 = 0;
    auto jac1_7 = 0;
    auto jac1_8 = 0;
    auto jac2_0 = 0 + 1;
    auto jac2_1 = 0 + -1;
    auto jac2_2 = 0;
    auto jac2_3 = 0;
    auto jac2_4 = 0;
    auto jac2_5 = 0;
    auto jac2_6 = 0;
    auto jac2_7 = 0;
    auto jac2_8 = 0;
    auto jac3_0 = 0 + (pr03a_trimmer != 0 ? 1 / (pr03a_trimmer * pr03a) : 0);
    auto jac3_1 = 0 + (pr03a_trimmer != 1 ? 1 / ((1 - pr03a_trimmer) * pr03a) : 0);
    auto jac3_2 = 0;
    auto jac3_3 = 0 + (pr03a_trimmer != 0 ? -1 / (pr03a_trimmer * pr03a) : 0)
                + (pr03a_trimmer != 1 ? -1 / ((1 - pr03a_trimmer) * pr03a) : 0) - r061.get_gradient();
    auto jac3_4 = 0 + r061.get_gradient();
    auto jac3_5 = 0;
    auto jac3_6 = 0;
    auto jac3_7 = 0;
    auto jac3_8 = 0;
    auto jac4_0 = 0;
    auto jac4_1 = 0;
    auto jac4_2 = 0;
    auto jac4_3 = 0 + r061.get_gradient();
    auto jac4_4 = 0 - r061.get_gradient() - (steady_state ? 0 : c044.get_gradient());
    auto jac4_5 = 0;
    auto jac4_6 = 0;
    auto jac4_7 = 0;
    auto jac4_8 = 0;
    auto jac5_0 = 0 + (pr03b_trimmer != 0 ? 1 / (pr03b_trimmer * pr03b) : 0);
    auto jac5_1 = 0 + (pr03b_trimmer != 1 ? 1 / ((1 - pr03b_trimmer) * pr03b) : 0);
    auto jac5_2 = 0;
    auto jac5_3 = 0;
    auto jac5_4 = 0;
    auto jac5_5 = 0 + (pr03b_trimmer != 0 ? -1 / (pr03b_trimmer * pr03b) : 0)
                + (pr03b_trimmer != 1 ? -1 / ((1 - pr03b_trimmer) * pr03b) : 0)
                - (steady_state ? 0 : c008.get_gradient());
    auto jac5_6 = 0 + (steady_state ? 0 : c008.get_gradient());
    auto jac5_7 = 0;
    auto jac5_8 = 0;
    auto jac6_0 = 0;
    auto jac6_1 = 0;
    auto jac6_2 = 0;
    auto jac6_3 = 0;
    auto jac6_4 = 0;
    auto jac6_5 = 0 + (steady_state ? 0 : c008.get_gradient());
    auto jac6_6
        = 0 - (steady_state ? 0 : c008.get_gradient()) - r012.get_gradient() - (steady_state ? 0 : c009.get_gradient());
    auto jac6_7 = 0 + r012.get_gradient();
    auto jac6_8 = 0 + (steady_state ? 0 : c009.get_gradient());
    auto jac7_0 = 0;
    auto jac7_1 = 0;
    auto jac7_2 = 0;
    auto jac7_3 = 0;
    auto jac7_4 = 0;
    auto jac7_5 = 0;
    auto jac7_6 = 0;
    auto jac7_7 = 0 + -1;
    auto jac7_8 = 0 + 1;
    auto jac8_0 = 0;
    auto jac8_1 = 0;
    auto jac8_2 = 0;
    auto jac8_3 = 0;
    auto jac8_4 = 0;
    auto jac8_5 = 0;
    auto jac8_6 = 0 + (steady_state ? 0 : c009.get_gradient());
    auto jac8_7 = 0;
    auto jac8_8 = 0 - (steady_state ? 0 : c009.get_gradient()) - r013.get_gradient();
    auto det
        = (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac2_1
                        * (1 * jac3_3
                                * (1 * jac4_4
                                    * (1 * jac5_5
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                        + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                            + -1 * jac3_4
                                  * (1 * jac4_3
                                      * (1 * jac5_5
                                              * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                  + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                  + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                          + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))))
            + -1 * jac0_3
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (1 * jac3_1
                                  * (1 * jac4_4
                                      * (1 * jac5_5
                                              * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                  + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                  + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                          + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))
                          + -1 * jac2_1
                                * (1 * jac3_0
                                    * (1 * jac4_4
                                        * (1 * jac5_5
                                                * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                    + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                    + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                            + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))))
            + -1 * jac0_5
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (-1 * jac3_3
                                      * (-1 * jac4_4
                                          * (1 * jac5_1
                                              * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                  + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                  + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                                  + 1 * jac3_4
                                        * (-1 * jac4_3
                                            * (1 * jac5_1
                                                * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                    + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                    + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))
                          + -1 * jac2_1
                                * (-1 * jac3_3
                                        * (-1 * jac4_4
                                            * (1 * jac5_0
                                                * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                    + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                    + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                                    + 1 * jac3_4
                                          * (-1 * jac4_3
                                              * (1 * jac5_0
                                                  * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                      + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                      + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))))));
    auto invdet = 1 / det;
    auto com0_0
        = (-1 * jac1_2
            * (1 * jac2_1
                * (1 * jac3_3
                        * (1 * jac4_4
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                    + -1 * jac3_4
                          * (1 * jac4_3
                              * (1 * jac5_5
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                  + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com1_0
        = -1
        * (-1 * jac1_2
            * (1 * jac2_0
                * (1 * jac3_3
                        * (1 * jac4_4
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                    + -1 * jac3_4
                          * (1 * jac4_3
                              * (1 * jac5_5
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                  + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com2_0
        = (-1 * jac1_1
                * (1 * jac2_0
                    * (1 * jac3_3
                            * (1 * jac4_4
                                * (1 * jac5_5
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                    + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                        + -1 * jac3_4
                              * (1 * jac4_3
                                  * (1 * jac5_5
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                      + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac1_3
                  * (1 * jac2_0
                          * (1 * jac3_1
                              * (1 * jac4_4
                                  * (1 * jac5_5
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                      + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))
                      + -1 * jac2_1
                            * (1 * jac3_0
                                * (1 * jac4_4
                                    * (1 * jac5_5
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                        + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac1_5
                  * (1 * jac2_0
                          * (-1 * jac3_3
                                  * (-1 * jac4_4
                                      * (1 * jac5_1
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                              + 1 * jac3_4
                                    * (-1 * jac4_3
                                        * (1 * jac5_1
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))
                      + -1 * jac2_1
                            * (-1 * jac3_3
                                    * (-1 * jac4_4
                                        * (1 * jac5_0
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                                + 1 * jac3_4
                                      * (-1 * jac4_3
                                          * (1 * jac5_0
                                              * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                  + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                  + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com3_0
        = -1
        * (1 * jac1_2
            * (1 * jac2_0
                    * (1 * jac3_1
                        * (1 * jac4_4
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))
                + -1 * jac2_1
                      * (1 * jac3_0
                          * (1 * jac4_4
                              * (1 * jac5_5
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                  + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com4_0
        = (1 * jac1_2
            * (1 * jac2_0
                    * (1 * jac3_1
                        * (1 * jac4_3
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))
                + -1 * jac2_1
                      * (1 * jac3_0
                          * (1 * jac4_3
                              * (1 * jac5_5
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                  + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com5_0
        = -1
        * (1 * jac1_2
            * (1 * jac2_0
                    * (-1 * jac3_3
                            * (-1 * jac4_4
                                * (1 * jac5_1
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                        + 1 * jac3_4
                              * (-1 * jac4_3
                                  * (1 * jac5_1
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))
                + -1 * jac2_1
                      * (-1 * jac3_3
                              * (-1 * jac4_4
                                  * (1 * jac5_0
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + 1 * jac3_4
                                * (-1 * jac4_3
                                    * (1 * jac5_0
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com6_0 = (1 * jac1_2
                   * (1 * jac2_0
                           * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                               + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))
                       + -1 * jac2_1
                             * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                                 + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_0 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com7_0
        = -1
        * (1 * jac1_2
            * (1 * jac2_0
                    * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6))))
                        + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6)))))
                + -1 * jac2_1
                      * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6))))
                          + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_0 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6)))))));
    auto com8_0 = (1 * jac1_2
                   * (1 * jac2_0
                           * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6))))
                               + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6)))))
                       + -1 * jac2_1
                             * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6))))
                                 + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_0 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6)))))));
    auto com0_1 = -1 * 0;
    auto com1_1 = 0;
    auto com2_1
        = -1
        * (1 * jac0_0
                * (1 * jac2_1
                    * (1 * jac3_3
                            * (1 * jac4_4
                                * (1 * jac5_5
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                    + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                        + -1 * jac3_4
                              * (1 * jac4_3
                                  * (1 * jac5_5
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                      + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_3
                  * (1 * jac2_0
                          * (1 * jac3_1
                              * (1 * jac4_4
                                  * (1 * jac5_5
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                      + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))
                      + -1 * jac2_1
                            * (1 * jac3_0
                                * (1 * jac4_4
                                    * (1 * jac5_5
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                        + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_5
                  * (1 * jac2_0
                          * (-1 * jac3_3
                                  * (-1 * jac4_4
                                      * (1 * jac5_1
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                              + 1 * jac3_4
                                    * (-1 * jac4_3
                                        * (1 * jac5_1
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))
                      + -1 * jac2_1
                            * (-1 * jac3_3
                                    * (-1 * jac4_4
                                        * (1 * jac5_0
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                                + 1 * jac3_4
                                      * (-1 * jac4_3
                                          * (1 * jac5_0
                                              * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                  + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                  + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com3_1 = 0;
    auto com4_1 = -1 * 0;
    auto com5_1 = 0;
    auto com6_1 = -1 * 0;
    auto com7_1 = 0;
    auto com8_1 = -1 * 0;
    auto com0_2
        = (1 * jac0_3
                * (-1 * jac1_2
                    * (1 * jac3_1
                        * (1 * jac4_4
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_5
                  * (-1 * jac1_2
                      * (-1 * jac3_3
                              * (-1 * jac4_4
                                  * (1 * jac5_1
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + 1 * jac3_4
                                * (-1 * jac4_3
                                    * (1 * jac5_1
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com1_2
        = -1
        * (1 * jac0_0
                * (1 * jac1_2
                    * (1 * jac3_3
                            * (1 * jac4_4
                                * (1 * jac5_5
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                    + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                        + -1 * jac3_4
                              * (1 * jac4_3
                                  * (1 * jac5_5
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                      + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_3
                  * (-1 * jac1_2
                      * (1 * jac3_0
                          * (1 * jac4_4
                              * (1 * jac5_5
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                  + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_5
                  * (-1 * jac1_2
                      * (-1 * jac3_3
                              * (-1 * jac4_4
                                  * (1 * jac5_0
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + 1 * jac3_4
                                * (-1 * jac4_3
                                    * (1 * jac5_0
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com2_2
        = (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac3_3
                                * (1 * jac4_4
                                    * (1 * jac5_5
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                        + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                            + -1 * jac3_4
                                  * (1 * jac4_3
                                      * (1 * jac5_5
                                              * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                  + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                  + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                          + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))
                    + -1 * jac1_3
                          * (1 * jac3_1
                              * (1 * jac4_4
                                  * (1 * jac5_5
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                      + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))
                    + -1 * jac1_5
                          * (-1 * jac3_3
                                  * (-1 * jac4_4
                                      * (1 * jac5_1
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                              + 1 * jac3_4
                                    * (-1 * jac4_3
                                        * (1 * jac5_1
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))))
            + 1 * jac0_3
                  * (-1 * jac1_1
                          * (1 * jac3_0
                              * (1 * jac4_4
                                  * (1 * jac5_5
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                      + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))
                      + -1 * jac1_5
                            * (1 * jac3_0
                                    * (-1 * jac4_4
                                        * (1 * jac5_1
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                                + -1 * jac3_1
                                      * (-1 * jac4_4
                                          * (1 * jac5_0
                                              * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                  + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                  + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))))
            + 1 * jac0_5
                  * (-1 * jac1_1
                          * (-1 * jac3_3
                                  * (-1 * jac4_4
                                      * (1 * jac5_0
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                              + 1 * jac3_4
                                    * (-1 * jac4_3
                                        * (1 * jac5_0
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))
                      + 1 * jac1_3
                            * (1 * jac3_0
                                    * (-1 * jac4_4
                                        * (1 * jac5_1
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                                + -1 * jac3_1
                                      * (-1 * jac4_4
                                          * (1 * jac5_0
                                              * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                  + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                  + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com3_2
        = -1
        * (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac3_1
                        * (1 * jac4_4
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_5
                  * (1 * jac1_2
                      * (1 * jac3_0
                              * (-1 * jac4_4
                                  * (1 * jac5_1
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac3_1
                                * (-1 * jac4_4
                                    * (1 * jac5_0
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com4_2
        = (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac3_1
                        * (1 * jac4_3
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_5
                  * (1 * jac1_2
                      * (1 * jac3_0
                              * (-1 * jac4_3
                                  * (1 * jac5_1
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac3_1
                                * (-1 * jac4_3
                                    * (1 * jac5_0
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com5_2
        = -1
        * (1 * jac0_0
                * (-1 * jac1_2
                    * (-1 * jac3_3
                            * (-1 * jac4_4
                                * (1 * jac5_1
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                        + 1 * jac3_4
                              * (-1 * jac4_3
                                  * (1 * jac5_1
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))))
            + -1 * jac0_3
                  * (1 * jac1_2
                      * (1 * jac3_0
                              * (-1 * jac4_4
                                  * (1 * jac5_1
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac3_1
                                * (-1 * jac4_4
                                    * (1 * jac5_0
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com6_2 = (1 * jac0_0
                       * (-1 * jac1_2
                           * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                               + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
                   + -1 * jac0_3
                         * (1 * jac1_2
                             * (1 * jac3_0 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                                 + -1 * jac3_1 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com7_2
        = -1
        * (1 * jac0_0
                * (-1 * jac1_2
                    * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6))))
                        + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6))))))
            + -1 * jac0_3
                  * (1 * jac1_2
                      * (1 * jac3_0 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6))))
                          + -1 * jac3_1 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6)))))));
    auto com8_2
        = (1 * jac0_0
                * (-1 * jac1_2
                    * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6))))
                        + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6))))))
            + -1 * jac0_3
                  * (1 * jac1_2
                      * (1 * jac3_0 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac3_1 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6)))))));
    auto com0_3 = -1
                * (1 * jac0_3
                    * (-1 * jac1_2
                        * (1 * jac2_1
                            * (1 * jac4_4
                                * (1 * jac5_5
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                    + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com1_3 = (1 * jac0_3
                   * (-1 * jac1_2
                       * (1 * jac2_0
                           * (1 * jac4_4
                               * (1 * jac5_5
                                       * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                           + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                   + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com2_3
        = -1
        * (1 * jac0_0
                * (-1 * jac1_3
                    * (1 * jac2_1
                        * (1 * jac4_4
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_3
                  * (-1 * jac1_1
                          * (1 * jac2_0
                              * (1 * jac4_4
                                  * (1 * jac5_5
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                      + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))
                      + -1 * jac1_5
                            * (1 * jac2_0
                                    * (-1 * jac4_4
                                        * (1 * jac5_1
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                                + -1 * jac2_1
                                      * (-1 * jac4_4
                                          * (1 * jac5_0
                                              * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                  + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                  + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))))
            + 1 * jac0_5
                  * (1 * jac1_3
                      * (1 * jac2_0
                              * (-1 * jac4_4
                                  * (1 * jac5_1
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac2_1
                                * (-1 * jac4_4
                                    * (1 * jac5_0
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com3_3
        = (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac2_1
                        * (1 * jac4_4
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_5
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (-1 * jac4_4
                                  * (1 * jac5_1
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac2_1
                                * (-1 * jac4_4
                                    * (1 * jac5_0
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com4_3
        = -1
        * (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac2_1
                        * (1 * jac4_3
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_5
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (-1 * jac4_3
                                  * (1 * jac5_1
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac2_1
                                * (-1 * jac4_3
                                    * (1 * jac5_0
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com5_3 = (-1 * jac0_3
                   * (1 * jac1_2
                       * (1 * jac2_0
                               * (-1 * jac4_4
                                   * (1 * jac5_1
                                       * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                           + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                           + -1 * jac2_1
                                 * (-1 * jac4_4
                                     * (1 * jac5_0
                                         * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                             + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com6_3 = -1
                * (-1 * jac0_3
                    * (1 * jac1_2
                        * (1 * jac2_0 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                            + -1 * jac2_1 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com7_3 = (-1 * jac0_3
                   * (1 * jac1_2
                       * (1 * jac2_0 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6))))
                           + -1 * jac2_1 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6)))))));
    auto com8_3 = -1
                * (-1 * jac0_3
                    * (1 * jac1_2
                        * (1 * jac2_0 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6))))
                            + -1 * jac2_1 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6)))))));
    auto com0_4 = (1 * jac0_3
                   * (-1 * jac1_2
                       * (1 * jac2_1
                           * (1 * jac3_4
                               * (1 * jac5_5
                                       * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                           + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                   + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com1_4 = -1
                * (1 * jac0_3
                    * (-1 * jac1_2
                        * (1 * jac2_0
                            * (1 * jac3_4
                                * (1 * jac5_5
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                    + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com2_4
        = (1 * jac0_0
                * (-1 * jac1_3
                    * (1 * jac2_1
                        * (1 * jac3_4
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_3
                  * (-1 * jac1_1
                          * (1 * jac2_0
                              * (1 * jac3_4
                                  * (1 * jac5_5
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                      + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))
                      + -1 * jac1_5
                            * (1 * jac2_0
                                    * (-1 * jac3_4
                                        * (1 * jac5_1
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                                + -1 * jac2_1
                                      * (-1 * jac3_4
                                          * (1 * jac5_0
                                              * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                  + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                  + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))))
            + 1 * jac0_5
                  * (1 * jac1_3
                      * (1 * jac2_0
                              * (-1 * jac3_4
                                  * (1 * jac5_1
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac2_1
                                * (-1 * jac3_4
                                    * (1 * jac5_0
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com3_4
        = -1
        * (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac2_1
                        * (1 * jac3_4
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_5
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (-1 * jac3_4
                                  * (1 * jac5_1
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac2_1
                                * (-1 * jac3_4
                                    * (1 * jac5_0
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com4_4
        = (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac2_1
                        * (1 * jac3_3
                            * (1 * jac5_5
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + -1 * jac0_3
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (1 * jac3_1
                                  * (1 * jac5_5
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                      + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                          + -1 * jac2_1
                                * (1 * jac3_0
                                    * (1 * jac5_5
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))
                                        + -1 * jac5_6 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_5
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (-1 * jac3_3
                                  * (1 * jac5_1
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac2_1
                                * (-1 * jac3_3
                                    * (1 * jac5_0
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com5_4 = -1
                * (-1 * jac0_3
                    * (1 * jac1_2
                        * (1 * jac2_0
                                * (-1 * jac3_4
                                    * (1 * jac5_1
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                            + -1 * jac2_1
                                  * (-1 * jac3_4
                                      * (1 * jac5_0
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com6_4 = (-1 * jac0_3
                   * (1 * jac1_2
                       * (1 * jac2_0 * (-1 * jac3_4 * (1 * jac5_1 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                           + -1 * jac2_1 * (-1 * jac3_4 * (1 * jac5_0 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com7_4 = -1
                * (-1 * jac0_3
                    * (1 * jac1_2
                        * (1 * jac2_0 * (-1 * jac3_4 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6))))
                            + -1 * jac2_1 * (-1 * jac3_4 * (1 * jac5_0 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6)))))));
    auto com8_4 = (-1 * jac0_3
                   * (1 * jac1_2
                       * (1 * jac2_0 * (-1 * jac3_4 * (1 * jac5_1 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6))))
                           + -1 * jac2_1 * (-1 * jac3_4 * (1 * jac5_0 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6)))))));
    auto com0_5 = -1
                * (1 * jac0_5
                    * (-1 * jac1_2
                        * (1 * jac2_1
                            * (1 * jac3_3
                                    * (1 * jac4_4
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))
                                + -1 * jac3_4
                                      * (1 * jac4_3
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com1_5 = (1 * jac0_5
                   * (-1 * jac1_2
                       * (1 * jac2_0
                           * (1 * jac3_3
                                   * (1 * jac4_4
                                       * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                           + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))
                               + -1 * jac3_4
                                     * (1 * jac4_3
                                         * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                             + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com2_5
        = -1
        * (1 * jac0_0
                * (-1 * jac1_5
                    * (1 * jac2_1
                        * (1 * jac3_3
                                * (1 * jac4_4
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))
                            + -1 * jac3_4
                                  * (1 * jac4_3
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))))
            + 1 * jac0_3
                  * (-1 * jac1_5
                      * (1 * jac2_0
                              * (1 * jac3_1
                                  * (1 * jac4_4
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac2_1
                                * (1 * jac3_0
                                    * (1 * jac4_4
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))))
            + 1 * jac0_5
                  * (-1 * jac1_1
                          * (1 * jac2_0
                              * (1 * jac3_3
                                      * (1 * jac4_4
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))
                                  + -1 * jac3_4
                                        * (1 * jac4_3
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))
                      + 1 * jac1_3
                            * (1 * jac2_0
                                    * (1 * jac3_1
                                        * (1 * jac4_4
                                            * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                                + -1 * jac2_1
                                      * (1 * jac3_0
                                          * (1 * jac4_4
                                              * (1 * jac6_6 * (1 * jac7_7 * jac8_8)
                                                  + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                                  + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com3_5 = (1 * jac0_5
                   * (1 * jac1_2
                       * (1 * jac2_0
                               * (1 * jac3_1
                                   * (1 * jac4_4
                                       * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                           + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                           + -1 * jac2_1
                                 * (1 * jac3_0
                                     * (1 * jac4_4
                                         * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                             + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com4_5 = -1
                * (1 * jac0_5
                    * (1 * jac1_2
                        * (1 * jac2_0
                                * (1 * jac3_1
                                    * (1 * jac4_3
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                            + -1 * jac2_1
                                  * (1 * jac3_0
                                      * (1 * jac4_3
                                          * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                              + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com5_5
        = (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac2_1
                        * (1 * jac3_3
                                * (1 * jac4_4
                                    * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                        + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))
                            + -1 * jac3_4
                                  * (1 * jac4_3
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))))
            + -1 * jac0_3
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (1 * jac3_1
                                  * (1 * jac4_4
                                      * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                          + 1 * jac6_8 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac2_1
                                * (1 * jac3_0
                                    * (1 * jac4_4
                                        * (1 * jac6_6 * (1 * jac7_7 * jac8_8) + -1 * jac6_7 * (-1 * jac7_8 * jac8_6)
                                            + 1 * jac6_8 * (-1 * jac7_7 * jac8_6)))))));
    auto com6_5 = -1
                * (1 * jac0_0
                        * (-1 * jac1_2
                            * (1 * jac2_1
                                * (1 * jac3_3 * (1 * jac4_4 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))
                                    + -1 * jac3_4 * (1 * jac4_3 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))))
                    + -1 * jac0_3
                          * (1 * jac1_2
                              * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac6_5 * (1 * jac7_7 * jac8_8))))
                                  + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac6_5 * (1 * jac7_7 * jac8_8)))))));
    auto com7_5 = (1 * jac0_0
                       * (-1 * jac1_2
                           * (1 * jac2_1
                               * (1 * jac3_3 * (1 * jac4_4 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6)))
                                   + -1 * jac3_4 * (1 * jac4_3 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6))))))
                   + -1 * jac0_3
                         * (1 * jac1_2
                             * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6))))
                                 + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac6_5 * (-1 * jac7_8 * jac8_6)))))));
    auto com8_5
        = -1
        * (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac2_1
                        * (1 * jac3_3 * (1 * jac4_4 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6)))
                            + -1 * jac3_4 * (1 * jac4_3 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6))))))
            + -1 * jac0_3
                  * (1 * jac1_2
                      * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6))))
                          + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac6_5 * (-1 * jac7_7 * jac8_6)))))));
    auto com0_6 = (1 * jac0_5
                   * (-1 * jac1_2
                       * (1 * jac2_1
                           * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8)))
                               + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac7_7 * jac8_8)))))));
    auto com1_6 = -1
                * (1 * jac0_5
                    * (-1 * jac1_2
                        * (1 * jac2_0
                            * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8)))
                                + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac7_7 * jac8_8)))))));
    auto com2_6
        = (1 * jac0_0
                * (-1 * jac1_5
                    * (1 * jac2_1
                        * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8)))
                            + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_3
                  * (-1 * jac1_5
                      * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8))))
                          + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8))))))
            + 1 * jac0_5
                  * (-1 * jac1_1
                          * (1 * jac2_0
                              * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8)))
                                  + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac7_7 * jac8_8)))))
                      + 1 * jac1_3
                            * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8))))
                                + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8)))))));
    auto com3_6 = -1
                * (1 * jac0_5
                    * (1 * jac1_2
                        * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8))))
                            + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8)))))));
    auto com4_6 = (1 * jac0_5
                   * (1 * jac1_2
                       * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac7_7 * jac8_8))))
                           + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac7_7 * jac8_8)))))));
    auto com5_6 = -1
                * (1 * jac0_0
                        * (-1 * jac1_2
                            * (1 * jac2_1
                                * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8)))
                                    + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac7_7 * jac8_8))))))
                    + -1 * jac0_3
                          * (1 * jac1_2
                              * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8))))
                                  + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac7_7 * jac8_8)))))));
    auto com6_6 = (1 * jac0_0
                       * (-1 * jac1_2
                           * (1 * jac2_1
                               * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_5 * (1 * jac7_7 * jac8_8)))
                                   + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_5 * (1 * jac7_7 * jac8_8))))))
                   + -1 * jac0_3
                         * (1 * jac1_2
                             * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_5 * (1 * jac7_7 * jac8_8))))
                                 + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_5 * (1 * jac7_7 * jac8_8))))))
                   + -1 * jac0_5
                         * (1 * jac1_2
                             * (1 * jac2_0
                                     * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac7_7 * jac8_8)))
                                         + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (1 * jac7_7 * jac8_8))))
                                 + -1 * jac2_1
                                       * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac7_7 * jac8_8)))
                                           + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_0 * (1 * jac7_7 * jac8_8)))))));
    auto com7_6 = -1
                * (1 * jac0_0
                        * (-1 * jac1_2
                            * (1 * jac2_1
                                * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_5 * (-1 * jac7_8 * jac8_6)))
                                    + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_5 * (-1 * jac7_8 * jac8_6))))))
                    + -1 * jac0_3
                          * (1 * jac1_2
                              * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_5 * (-1 * jac7_8 * jac8_6))))
                                  + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_5 * (-1 * jac7_8 * jac8_6))))))
                    + -1 * jac0_5
                          * (1 * jac1_2
                              * (1 * jac2_0
                                      * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (-1 * jac7_8 * jac8_6)))
                                          + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (-1 * jac7_8 * jac8_6))))
                                  + -1 * jac2_1
                                        * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_0 * (-1 * jac7_8 * jac8_6)))
                                            + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_0 * (-1 * jac7_8 * jac8_6)))))));
    auto com8_6 = (1 * jac0_0
                       * (-1 * jac1_2
                           * (1 * jac2_1
                               * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_5 * (-1 * jac7_7 * jac8_6)))
                                   + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_5 * (-1 * jac7_7 * jac8_6))))))
                   + -1 * jac0_3
                         * (1 * jac1_2
                             * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_5 * (-1 * jac7_7 * jac8_6))))
                                 + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_5 * (-1 * jac7_7 * jac8_6))))))
                   + -1 * jac0_5
                         * (1 * jac1_2
                             * (1 * jac2_0
                                     * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (-1 * jac7_7 * jac8_6)))
                                         + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (-1 * jac7_7 * jac8_6))))
                                 + -1 * jac2_1
                                       * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_0 * (-1 * jac7_7 * jac8_6)))
                                           + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_0 * (-1 * jac7_7 * jac8_6)))))));
    auto com0_7 = -1
                * (1 * jac0_5
                    * (-1 * jac1_2
                        * (1 * jac2_1
                            * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8)))
                                + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac8_8)))))));
    auto com1_7 = (1 * jac0_5
                   * (-1 * jac1_2
                       * (1 * jac2_0
                           * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8)))
                               + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac8_8)))))));
    auto com2_7
        = -1
        * (1 * jac0_0
                * (-1 * jac1_5
                    * (1 * jac2_1
                        * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8)))
                            + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac8_8))))))
            + 1 * jac0_3
                  * (-1 * jac1_5
                      * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8))))
                          + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8))))))
            + 1 * jac0_5
                  * (-1 * jac1_1
                          * (1 * jac2_0
                              * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8)))
                                  + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac8_8)))))
                      + 1 * jac1_3
                            * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8))))
                                + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8)))))));
    auto com3_7 = (1 * jac0_5
                   * (1 * jac1_2
                       * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8))))
                           + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8)))))));
    auto com4_7 = -1
                * (1 * jac0_5
                    * (1 * jac1_2
                        * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac8_8))))
                            + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac8_8)))))));
    auto com5_7 = (1 * jac0_0
                       * (-1 * jac1_2
                           * (1 * jac2_1
                               * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8)))
                                   + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac8_8))))))
                   + -1 * jac0_3
                         * (1 * jac1_2
                             * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8))))
                                 + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac8_8)))))));
    auto com6_7 = -1
                * (1 * jac0_0
                        * (-1 * jac1_2
                            * (1 * jac2_1
                                * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_5 * (1 * jac6_7 * jac8_8)))
                                    + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_5 * (1 * jac6_7 * jac8_8))))))
                    + -1 * jac0_3
                          * (1 * jac1_2
                              * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_5 * (1 * jac6_7 * jac8_8))))
                                  + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_5 * (1 * jac6_7 * jac8_8))))))
                    + -1 * jac0_5
                          * (1 * jac1_2
                              * (1 * jac2_0
                                      * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_7 * jac8_8)))
                                          + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (1 * jac6_7 * jac8_8))))
                                  + -1 * jac2_1
                                        * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_7 * jac8_8)))
                                            + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_0 * (1 * jac6_7 * jac8_8)))))));
    auto com7_7
        = (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac2_1
                        * (1 * jac3_3
                                * (1 * jac4_4
                                    * (1 * jac5_5 * (1 * jac6_6 * jac8_8 + -1 * jac6_8 * jac8_6)
                                        + -1 * jac5_6 * (1 * jac6_5 * jac8_8)))
                            + -1 * jac3_4
                                  * (1 * jac4_3
                                      * (1 * jac5_5 * (1 * jac6_6 * jac8_8 + -1 * jac6_8 * jac8_6)
                                          + -1 * jac5_6 * (1 * jac6_5 * jac8_8))))))
            + -1 * jac0_3
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (1 * jac3_1
                                  * (1 * jac4_4
                                      * (1 * jac5_5 * (1 * jac6_6 * jac8_8 + -1 * jac6_8 * jac8_6)
                                          + -1 * jac5_6 * (1 * jac6_5 * jac8_8))))
                          + -1 * jac2_1
                                * (1 * jac3_0
                                    * (1 * jac4_4
                                        * (1 * jac5_5 * (1 * jac6_6 * jac8_8 + -1 * jac6_8 * jac8_6)
                                            + -1 * jac5_6 * (1 * jac6_5 * jac8_8))))))
            + -1 * jac0_5
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (-1 * jac3_3
                                      * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_6 * jac8_8 + -1 * jac6_8 * jac8_6)))
                                  + 1 * jac3_4
                                        * (-1 * jac4_3 * (1 * jac5_1 * (1 * jac6_6 * jac8_8 + -1 * jac6_8 * jac8_6))))
                          + -1 * jac2_1
                                * (-1 * jac3_3
                                        * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_6 * jac8_8 + -1 * jac6_8 * jac8_6)))
                                    + 1 * jac3_4
                                          * (-1 * jac4_3
                                              * (1 * jac5_0 * (1 * jac6_6 * jac8_8 + -1 * jac6_8 * jac8_6)))))));
    auto com8_7 = -1
                * (1 * jac0_0
                        * (-1 * jac1_2
                            * (1 * jac2_1
                                * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_5 * (-1 * jac6_7 * jac8_6)))
                                    + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_5 * (-1 * jac6_7 * jac8_6))))))
                    + -1 * jac0_3
                          * (1 * jac1_2
                              * (1 * jac2_0 * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_5 * (-1 * jac6_7 * jac8_6))))
                                  + -1 * jac2_1 * (1 * jac3_0 * (1 * jac4_4 * (1 * jac5_5 * (-1 * jac6_7 * jac8_6))))))
                    + -1 * jac0_5
                          * (1 * jac1_2
                              * (1 * jac2_0
                                      * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (-1 * jac6_7 * jac8_6)))
                                          + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (-1 * jac6_7 * jac8_6))))
                                  + -1 * jac2_1
                                        * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_0 * (-1 * jac6_7 * jac8_6)))
                                            + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_0 * (-1 * jac6_7 * jac8_6)))))));
    auto com0_8
        = (1 * jac0_5
            * (-1 * jac1_2
                * (1 * jac2_1
                    * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))
                        + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))))));
    auto com1_8
        = -1
        * (1 * jac0_5
            * (-1 * jac1_2
                * (1 * jac2_0
                    * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))
                        + -1 * jac3_4 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))))));
    auto com2_8
        = (1 * jac0_0
                * (-1 * jac1_5
                    * (1 * jac2_1
                        * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))
                            + -1 * jac3_4
                                  * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7))))))
            + 1 * jac0_3
                  * (-1 * jac1_5
                      * (1 * jac2_0
                              * (1 * jac3_1
                                  * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7))))
                          + -1 * jac2_1
                                * (1 * jac3_0
                                    * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7))))))
            + 1 * jac0_5
                  * (-1 * jac1_1
                          * (1 * jac2_0
                              * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))
                                  + -1 * jac3_4
                                        * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))))
                      + 1 * jac1_3
                            * (1 * jac2_0
                                    * (1 * jac3_1
                                        * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7))))
                                + -1 * jac2_1
                                      * (1 * jac3_0
                                          * (1 * jac4_4
                                              * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))))));
    auto com3_8
        = -1
        * (1 * jac0_5
            * (1 * jac1_2
                * (1 * jac2_0
                        * (1 * jac3_1 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7))))
                    + -1 * jac2_1
                          * (1 * jac3_0
                              * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))))));
    auto com4_8
        = (1 * jac0_5
            * (1 * jac1_2
                * (1 * jac2_0
                        * (1 * jac3_1 * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7))))
                    + -1 * jac2_1
                          * (1 * jac3_0
                              * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))))));
    auto com5_8
        = -1
        * (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac2_1
                        * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))
                            + -1 * jac3_4
                                  * (1 * jac4_3 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7))))))
            + -1 * jac0_3
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (1 * jac3_1
                                  * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7))))
                          + -1 * jac2_1
                                * (1 * jac3_0
                                    * (1 * jac4_4 * (1 * jac5_6 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))))));
    auto com6_8
        = (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac2_1
                        * (1 * jac3_3 * (1 * jac4_4 * (1 * jac5_5 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))
                            + -1 * jac3_4
                                  * (1 * jac4_3 * (1 * jac5_5 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7))))))
            + -1 * jac0_3
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (1 * jac3_1
                                  * (1 * jac4_4 * (1 * jac5_5 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7))))
                          + -1 * jac2_1
                                * (1 * jac3_0
                                    * (1 * jac4_4 * (1 * jac5_5 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7))))))
            + -1 * jac0_5
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (-1 * jac3_3
                                      * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))
                                  + 1 * jac3_4
                                        * (-1 * jac4_3 * (1 * jac5_1 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7))))
                          + -1 * jac2_1
                                * (-1 * jac3_3
                                        * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))
                                    + 1 * jac3_4
                                          * (-1 * jac4_3
                                              * (1 * jac5_0 * (1 * jac6_7 * jac7_8 + -1 * jac6_8 * jac7_7)))))));
    auto com7_8
        = -1
        * (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac2_1
                        * (1 * jac3_3
                                * (1 * jac4_4
                                    * (1 * jac5_5 * (1 * jac6_6 * jac7_8) + -1 * jac5_6 * (1 * jac6_5 * jac7_8)))
                            + -1 * jac3_4
                                  * (1 * jac4_3
                                      * (1 * jac5_5 * (1 * jac6_6 * jac7_8) + -1 * jac5_6 * (1 * jac6_5 * jac7_8))))))
            + -1 * jac0_3
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (1 * jac3_1
                                  * (1 * jac4_4
                                      * (1 * jac5_5 * (1 * jac6_6 * jac7_8) + -1 * jac5_6 * (1 * jac6_5 * jac7_8))))
                          + -1 * jac2_1
                                * (1 * jac3_0
                                    * (1 * jac4_4
                                        * (1 * jac5_5 * (1 * jac6_6 * jac7_8) + -1 * jac5_6 * (1 * jac6_5 * jac7_8))))))
            + -1 * jac0_5
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_6 * jac7_8)))
                                  + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (1 * jac6_6 * jac7_8))))
                          + -1 * jac2_1
                                * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_6 * jac7_8)))
                                    + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_0 * (1 * jac6_6 * jac7_8)))))));
    auto com8_8
        = (1 * jac0_0
                * (-1 * jac1_2
                    * (1 * jac2_1
                        * (1 * jac3_3
                                * (1 * jac4_4
                                    * (1 * jac5_5 * (1 * jac6_6 * jac7_7) + -1 * jac5_6 * (1 * jac6_5 * jac7_7)))
                            + -1 * jac3_4
                                  * (1 * jac4_3
                                      * (1 * jac5_5 * (1 * jac6_6 * jac7_7) + -1 * jac5_6 * (1 * jac6_5 * jac7_7))))))
            + -1 * jac0_3
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (1 * jac3_1
                                  * (1 * jac4_4
                                      * (1 * jac5_5 * (1 * jac6_6 * jac7_7) + -1 * jac5_6 * (1 * jac6_5 * jac7_7))))
                          + -1 * jac2_1
                                * (1 * jac3_0
                                    * (1 * jac4_4
                                        * (1 * jac5_5 * (1 * jac6_6 * jac7_7) + -1 * jac5_6 * (1 * jac6_5 * jac7_7))))))
            + -1 * jac0_5
                  * (1 * jac1_2
                      * (1 * jac2_0
                              * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_1 * (1 * jac6_6 * jac7_7)))
                                  + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_1 * (1 * jac6_6 * jac7_7))))
                          + -1 * jac2_1
                                * (-1 * jac3_3 * (-1 * jac4_4 * (1 * jac5_0 * (1 * jac6_6 * jac7_7)))
                                    + 1 * jac3_4 * (-1 * jac4_3 * (1 * jac5_0 * (1 * jac6_6 * jac7_7)))))));
    Eigen::Matrix<DataType, 9, 9> cojacobian(Eigen::Matrix<DataType, 9, 9>::Zero());

    cojacobian << com0_0, com0_1, com0_2, com0_3, com0_4, com0_5, com0_6, com0_7, com0_8, com1_0, com1_1, com1_2,
        com1_3, com1_4, com1_5, com1_6, com1_7, com1_8, com2_0, com2_1, com2_2, com2_3, com2_4, com2_5, com2_6, com2_7,
        com2_8, com3_0, com3_1, com3_2, com3_3, com3_4, com3_5, com3_6, com3_7, com3_8, com4_0, com4_1, com4_2, com4_3,
        com4_4, com4_5, com4_6, com4_7, com4_8, com5_0, com5_1, com5_2, com5_3, com5_4, com5_5, com5_6, com5_7, com5_8,
        com6_0, com6_1, com6_2, com6_3, com6_4, com6_5, com6_6, com6_7, com6_8, com7_0, com7_1, com7_2, com7_3, com7_4,
        com7_5, com7_6, com7_7, com7_8, com8_0, com8_1, com8_2, com8_3, com8_4, com8_5, com8_6, com8_7, com8_8;
    Eigen::Matrix<DataType, 9, 1> delta = cojacobian * eqs * invdet;

    // Check if the update is big enough
    if((delta.array().abs() < EPS).all())
    {
      return true;
    }

    dynamic_state -= delta;

    return false;
  }
};
} // namespace
namespace MT2
{
std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter_stage7()
{
  return std::make_unique<StaticFilter>();
}
} // namespace MT2
