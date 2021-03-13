#include <cstdlib>
#include <memory>

#include <ATK/Core/Utilities.h>
#include <ATK/Modelling/ModellerFilter.h>
#include <ATK/Modelling/StaticComponent/StaticCapacitor.h>
#include <ATK/Modelling/StaticComponent/StaticCoil.h>
#include <ATK/Modelling/StaticComponent/StaticCurrent.h>
#include <ATK/Modelling/StaticComponent/StaticDiode.h>
#include <ATK/Modelling/StaticComponent/StaticEbersMollTransistor.h>
#include <ATK/Modelling/StaticComponent/StaticMOSFETTransistor.h>
#include <ATK/Modelling/StaticComponent/StaticResistor.h>
#include <ATK/Modelling/StaticComponent/StaticResistorCapacitor.h>

#include <Eigen/Eigen>

namespace
{
constexpr gsl::index MAX_ITERATION = 10;
constexpr gsl::index MAX_ITERATION_STEADY_STATE{200};

constexpr gsl::index INIT_WARMUP{10};
constexpr double EPS{1e-8};
constexpr double MAX_DELTA{1e-1};

class StaticFilter final: public ATK::ModellerFilter<double>
{
  using typename ATK::TypedBaseFilter<double>::DataType;
  bool initialized{false};

  Eigen::Matrix<DataType, 3, 1> static_state{Eigen::Matrix<DataType, 3, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 1, 1> input_state{Eigen::Matrix<DataType, 1, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 8, 1> dynamic_state{Eigen::Matrix<DataType, 8, 1>::Zero()};
  ATK::StaticEBNPN<DataType> q008{
      1e-12,
      0.026,
      1,
      1,
      100,
  };
  ATK::StaticResistor<DataType> r036{47000};
  ATK::StaticCapacitor<DataType> c024{1.5e-07};
  ATK::StaticResistor<DataType> r025{470000};
  ATK::StaticCapacitor<DataType> c017{4.7e-08};
  ATK::StaticResistor<DataType> r024{10000};
  ATK::StaticResistor<DataType> r037{10000};
  ATK::StaticEBNPN<DataType> q007{
      1e-12,
      0.026,
      1,
      1,
      100,
  };
  ATK::StaticResistor<DataType> r027{470};
  ATK::StaticCapacitor<DataType> c025{7e-09};
  ATK::StaticCapacitor<DataType> c020{2.2e-07};
  ATK::StaticCapacitor<DataType> c022{4.7e-11};
  ATK::StaticResistor<DataType> r030{3300};
  ATK::StaticResistor<DataType> r034{400};

public:
  StaticFilter(): ModellerFilter<DataType>(8, 1)
  {
    static_state << 0.000000, 4.500000, -4.500000;
  }

  ~StaticFilter() override = default;

  gsl::index get_nb_dynamic_pins() const override
  {
    return 8;
  }

  gsl::index get_nb_input_pins() const override
  {
    return 1;
  }

  gsl::index get_nb_static_pins() const override
  {
    return 3;
  }

  Eigen::Matrix<DataType, Eigen::Dynamic, 1> get_static_state() const override
  {
    return static_state;
  }

  gsl::index get_nb_components() const override
  {
    return 15;
  }

  std::string get_dynamic_pin_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 7:
      return "vout";
    case 5:
      return "3";
    case 1:
      return "7";
    case 4:
      return "5";
    case 3:
      return "6";
    case 2:
      return "2";
    case 6:
      return "4";
    case 0:
      return "8";
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
    case 2:
      return "vdd";
    case 1:
      return "vcc";
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
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  DataType get_parameter(gsl::index identifier) const override
  {
    switch(identifier)
    {
    default:
      throw ATK::RuntimeError("No such pin");
    }
  }

  void set_parameter(gsl::index identifier, DataType value) override
  {
    switch(identifier)
    {
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
    c024.update_steady_state(1. / input_sampling_rate, dynamic_state[2], dynamic_state[3]);
    c017.update_steady_state(1. / input_sampling_rate, dynamic_state[5], dynamic_state[4]);
    c025.update_steady_state(1. / input_sampling_rate, dynamic_state[3], dynamic_state[0]);
    c020.update_steady_state(1. / input_sampling_rate, dynamic_state[2], dynamic_state[5]);
    c022.update_steady_state(1. / input_sampling_rate, dynamic_state[2], dynamic_state[7]);

    solve<true>();

    // update_steady_state
    c024.update_steady_state(1. / input_sampling_rate, dynamic_state[2], dynamic_state[3]);
    c017.update_steady_state(1. / input_sampling_rate, dynamic_state[5], dynamic_state[4]);
    c025.update_steady_state(1. / input_sampling_rate, dynamic_state[3], dynamic_state[0]);
    c020.update_steady_state(1. / input_sampling_rate, dynamic_state[2], dynamic_state[5]);
    c022.update_steady_state(1. / input_sampling_rate, dynamic_state[2], dynamic_state[7]);

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
      c024.update_state(dynamic_state[2], dynamic_state[3]);
      c017.update_state(dynamic_state[5], dynamic_state[4]);
      c025.update_state(dynamic_state[3], dynamic_state[0]);
      c020.update_state(dynamic_state[2], dynamic_state[5]);
      c022.update_state(dynamic_state[2], dynamic_state[7]);
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
    auto s2_ = static_state[2];

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

    // Precomputes
    q008.precompute(dynamic_state[0], static_state[1], dynamic_state[1]);
    q007.precompute(dynamic_state[4], static_state[1], dynamic_state[6]);

    Eigen::Matrix<DataType, 8, 1> eqs(Eigen::Matrix<DataType, 8, 1>::Zero());
    auto eq0 = -q008.ib() - r036.get_current(s0_, d0_) - (steady_state ? 0 : c025.get_current(d3_, d0_));
    auto eq1 = +q008.ib() + q008.ic() + r037.get_current(d1_, s2_) - r034.get_current(d3_, d1_);
    auto eq2 = +(steady_state ? 0 : c024.get_current(d2_, d3_)) + (steady_state ? 0 : c020.get_current(d2_, d5_))
             + (steady_state ? 0 : c022.get_current(d2_, d7_)) + r030.get_current(d2_, d7_);
    auto eq3 = -(steady_state ? 0 : c024.get_current(d2_, d3_)) + (steady_state ? 0 : c025.get_current(d3_, d0_))
             + r034.get_current(d3_, d1_);
    auto eq4 = -r025.get_current(s0_, d4_) - (steady_state ? 0 : c017.get_current(d5_, d4_)) - q007.ib();
    auto eq5 = +(steady_state ? 0 : c017.get_current(d5_, d4_)) + r027.get_current(d5_, d6_)
             - (steady_state ? 0 : c020.get_current(d2_, d5_));
    auto eq6 = +r024.get_current(d6_, s2_) + q007.ib() + q007.ic() - r027.get_current(d5_, d6_);
    auto eq7 = input_state[0] - dynamic_state[2];
    eqs << eq0, eq1, eq2, eq3, eq4, eq5, eq6, eq7;

    // Check if the equations have converged
    if((eqs.array().abs() < EPS).all())
    {
      return true;
    }

    auto jac0_0 = 0 - q008.ib_Vbc() - q008.ib_Vbe() - r036.get_gradient() - (steady_state ? 0 : c025.get_gradient());
    auto jac0_1 = 0 + q008.ib_Vbe();
    auto jac0_2 = 0;
    auto jac0_3 = 0 + (steady_state ? 0 : c025.get_gradient());
    auto jac0_4 = 0;
    auto jac0_5 = 0;
    auto jac0_6 = 0;
    auto jac0_7 = 0;
    auto jac1_0 = 0 + q008.ib_Vbc() + q008.ib_Vbe() + q008.ic_Vbc() + q008.ic_Vbe();
    auto jac1_1 = 0 - q008.ib_Vbe() - q008.ic_Vbe() - r037.get_gradient() - r034.get_gradient();
    auto jac1_2 = 0;
    auto jac1_3 = 0 + r034.get_gradient();
    auto jac1_4 = 0;
    auto jac1_5 = 0;
    auto jac1_6 = 0;
    auto jac1_7 = 0;
    auto jac2_0 = 0;
    auto jac2_1 = 0;
    auto jac2_2 = 0 - (steady_state ? 0 : c024.get_gradient()) - (steady_state ? 0 : c020.get_gradient())
                - (steady_state ? 0 : c022.get_gradient()) - r030.get_gradient();
    auto jac2_3 = 0 + (steady_state ? 0 : c024.get_gradient());
    auto jac2_4 = 0;
    auto jac2_5 = 0 + (steady_state ? 0 : c020.get_gradient());
    auto jac2_6 = 0;
    auto jac2_7 = 0 + (steady_state ? 0 : c022.get_gradient()) + r030.get_gradient();
    auto jac3_0 = 0 + (steady_state ? 0 : c025.get_gradient());
    auto jac3_1 = 0 + r034.get_gradient();
    auto jac3_2 = 0 + (steady_state ? 0 : c024.get_gradient());
    auto jac3_3
        = 0 - (steady_state ? 0 : c024.get_gradient()) - (steady_state ? 0 : c025.get_gradient()) - r034.get_gradient();
    auto jac3_4 = 0;
    auto jac3_5 = 0;
    auto jac3_6 = 0;
    auto jac3_7 = 0;
    auto jac4_0 = 0;
    auto jac4_1 = 0;
    auto jac4_2 = 0;
    auto jac4_3 = 0;
    auto jac4_4 = 0 - r025.get_gradient() - (steady_state ? 0 : c017.get_gradient()) - q007.ib_Vbc() - q007.ib_Vbe();
    auto jac4_5 = 0 + (steady_state ? 0 : c017.get_gradient());
    auto jac4_6 = 0 + q007.ib_Vbe();
    auto jac4_7 = 0;
    auto jac5_0 = 0;
    auto jac5_1 = 0;
    auto jac5_2 = 0 + (steady_state ? 0 : c020.get_gradient());
    auto jac5_3 = 0;
    auto jac5_4 = 0 + (steady_state ? 0 : c017.get_gradient());
    auto jac5_5
        = 0 - (steady_state ? 0 : c017.get_gradient()) - r027.get_gradient() - (steady_state ? 0 : c020.get_gradient());
    auto jac5_6 = 0 + r027.get_gradient();
    auto jac5_7 = 0;
    auto jac6_0 = 0;
    auto jac6_1 = 0;
    auto jac6_2 = 0;
    auto jac6_3 = 0;
    auto jac6_4 = 0 + q007.ib_Vbc() + q007.ib_Vbe() + q007.ic_Vbc() + q007.ic_Vbe();
    auto jac6_5 = 0 + r027.get_gradient();
    auto jac6_6 = 0 - r024.get_gradient() - q007.ib_Vbe() - q007.ic_Vbe() - r027.get_gradient();
    auto jac6_7 = 0;
    auto jac7_0 = 0;
    auto jac7_1 = 0;
    auto jac7_2 = 0 + -1;
    auto jac7_3 = 0;
    auto jac7_4 = 0;
    auto jac7_5 = 0;
    auto jac7_6 = 0;
    auto jac7_7 = 0;
    auto det
        = (1 * jac0_0
                * (1 * jac1_1
                        * (-1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac4_4
                                        * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                                    + 1 * jac4_5
                                          * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                                    + -1 * jac4_6
                                          * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2)
                                              + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
                    + 1 * jac1_3
                          * (-1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4
                                          * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                                      + 1 * jac4_5
                                            * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2)
                                                + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                                      + -1 * jac4_6
                                            * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2)
                                                + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (-1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac4_4
                                          * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                                      + 1 * jac4_5
                                            * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2)
                                                + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                                      + -1 * jac4_6
                                            * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2)
                                                + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
                      + 1 * jac1_3
                            * (-1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4
                                            * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2)
                                                + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                                        + 1 * jac4_5
                                              * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2)
                                                  + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                                        + -1 * jac4_6
                                              * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2)
                                                  + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (-1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4
                                          * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                                      + 1 * jac4_5
                                            * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2)
                                                + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                                      + -1 * jac4_6
                                            * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2)
                                                + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
                      + -1 * jac1_1
                            * (-1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4
                                            * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2)
                                                + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                                        + 1 * jac4_5
                                              * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2)
                                                  + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                                        + -1 * jac4_6
                                              * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2)
                                                  + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))));
    auto invdet = 1 / det;
    auto com0_0
        = (1 * jac1_1
                * (-1 * jac2_7
                    * (-1 * jac3_3
                        * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                            + 1 * jac4_5 * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                            + -1 * jac4_6
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + 1 * jac1_3
                  * (-1 * jac2_7
                      * (1 * jac3_1
                          * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                              + 1 * jac4_5
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                              + -1 * jac4_6
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com1_0
        = -1
        * (1 * jac1_0
                * (-1 * jac2_7
                    * (-1 * jac3_3
                        * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                            + 1 * jac4_5 * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                            + -1 * jac4_6
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + 1 * jac1_3
                  * (-1 * jac2_7
                      * (1 * jac3_0
                          * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                              + 1 * jac4_5
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                              + -1 * jac4_6
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com2_0 = 0;
    auto com3_0
        = -1
        * (1 * jac1_0
                * (-1 * jac2_7
                    * (1 * jac3_1
                        * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                            + 1 * jac4_5 * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                            + -1 * jac4_6
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac1_1
                  * (-1 * jac2_7
                      * (1 * jac3_0
                          * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                              + 1 * jac4_5
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                              + -1 * jac4_6
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com4_0 = 0;
    auto com5_0 = -1 * 0;
    auto com6_0 = 0;
    auto com7_0
        = -1
        * (1 * jac1_0
                * (1 * jac2_3
                    * (1 * jac3_1
                        * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                            + 1 * jac4_5 * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                            + -1 * jac4_6
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac1_1
                  * (1 * jac2_3
                      * (1 * jac3_0
                          * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                              + 1 * jac4_5
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                              + -1 * jac4_6
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com0_1
        = -1
        * (1 * jac0_1
                * (-1 * jac2_7
                    * (-1 * jac3_3
                        * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                            + 1 * jac4_5 * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                            + -1 * jac4_6
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + 1 * jac0_3
                  * (-1 * jac2_7
                      * (1 * jac3_1
                          * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                              + 1 * jac4_5
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                              + -1 * jac4_6
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com1_1
        = (1 * jac0_0
                * (-1 * jac2_7
                    * (-1 * jac3_3
                        * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                            + 1 * jac4_5 * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                            + -1 * jac4_6
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + 1 * jac0_3
                  * (-1 * jac2_7
                      * (1 * jac3_0
                          * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                              + 1 * jac4_5
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                              + -1 * jac4_6
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com2_1 = -1 * 0;
    auto com3_1
        = (1 * jac0_0
                * (-1 * jac2_7
                    * (1 * jac3_1
                        * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                            + 1 * jac4_5 * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                            + -1 * jac4_6
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_1
                  * (-1 * jac2_7
                      * (1 * jac3_0
                          * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                              + 1 * jac4_5
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                              + -1 * jac4_6
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com4_1 = -1 * 0;
    auto com5_1 = 0;
    auto com6_1 = -1 * 0;
    auto com7_1
        = (1 * jac0_0
                * (1 * jac2_3
                    * (1 * jac3_1
                        * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                            + 1 * jac4_5 * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                            + -1 * jac4_6
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac2_3
                      * (1 * jac3_0
                          * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                              + 1 * jac4_5
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                              + -1 * jac4_6
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com0_2 = 0;
    auto com1_2 = -1 * 0;
    auto com2_2 = 0;
    auto com3_2 = -1 * 0;
    auto com4_2 = 0;
    auto com5_2 = -1 * 0;
    auto com6_2 = 0;
    auto com7_2
        = -1
        * (1 * jac0_0
                * (1 * jac1_1
                        * (-1 * jac3_3
                            * (-1 * jac4_4
                                    * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                                + 1 * jac4_5
                                      * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                                + -1 * jac4_6
                                      * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))
                    + 1 * jac1_3
                          * (1 * jac3_1
                              * (-1 * jac4_4
                                      * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                                  + 1 * jac4_5
                                        * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                                  + -1 * jac4_6
                                        * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2)
                                            + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (-1 * jac3_3
                              * (-1 * jac4_4
                                      * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                                  + 1 * jac4_5
                                        * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                                  + -1 * jac4_6
                                        * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))
                      + 1 * jac1_3
                            * (1 * jac3_0
                                * (-1 * jac4_4
                                        * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                                    + 1 * jac4_5
                                          * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                                    + -1 * jac4_6
                                          * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2)
                                              + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac3_1
                              * (-1 * jac4_4
                                      * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                                  + 1 * jac4_5
                                        * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                                  + -1 * jac4_6
                                        * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))
                      + -1 * jac1_1
                            * (1 * jac3_0
                                * (-1 * jac4_4
                                        * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                                    + 1 * jac4_5
                                          * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                                    + -1 * jac4_6
                                          * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2)
                                              + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com0_3
        = -1
        * (1 * jac0_1
                * (-1 * jac1_3
                    * (1 * jac2_7
                        * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                            + 1 * jac4_5 * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                            + -1 * jac4_6
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + 1 * jac0_3
                  * (1 * jac1_1
                      * (1 * jac2_7
                          * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                              + 1 * jac4_5
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                              + -1 * jac4_6
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com1_3
        = (1 * jac0_0
                * (-1 * jac1_3
                    * (1 * jac2_7
                        * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                            + 1 * jac4_5 * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                            + -1 * jac4_6
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + 1 * jac0_3
                  * (1 * jac1_0
                      * (1 * jac2_7
                          * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                              + 1 * jac4_5
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                              + -1 * jac4_6
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com2_3 = -1 * 0;
    auto com3_3
        = (1 * jac0_0
                * (1 * jac1_1
                    * (1 * jac2_7
                        * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                            + 1 * jac4_5 * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                            + -1 * jac4_6
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                      * (1 * jac2_7
                          * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                              + 1 * jac4_5
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                              + -1 * jac4_6
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com4_3 = -1 * 0;
    auto com5_3 = 0;
    auto com6_3 = -1 * 0;
    auto com7_3
        = (1 * jac0_0
                * (1 * jac1_1
                    * (-1 * jac2_3
                        * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                            + 1 * jac4_5 * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                            + -1 * jac4_6
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                      * (-1 * jac2_3
                          * (-1 * jac4_4 * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))
                              + 1 * jac4_5
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))
                              + -1 * jac4_6
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com0_4 = 0;
    auto com1_4 = -1 * 0;
    auto com2_4 = 0;
    auto com3_4 = -1 * 0;
    auto com4_4
        = (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))))
                    + 1 * jac1_3
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))))
                      + 1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))))
                      + -1 * jac1_1
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac5_5 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_5 * jac7_2))))));
    auto com5_4
        = -1
        * (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))))
                    + 1 * jac1_3
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))))
                      + 1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))))
                      + -1 * jac1_1
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))))));
    auto com6_4
        = (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))
                    + 1 * jac1_3
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))
                      + 1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))
                      + -1 * jac1_1
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac5_4 * (-1 * jac6_5 * jac7_2) + 1 * jac5_5 * (-1 * jac6_4 * jac7_2))))));
    auto com7_4
        = -1
        * (1 * jac0_0
                * (1 * jac1_1
                        * (-1 * jac2_5
                            * (-1 * jac3_3
                                * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))))
                    + 1 * jac1_3
                          * (-1 * jac2_5
                              * (1 * jac3_1
                                  * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (-1 * jac2_5
                              * (-1 * jac3_3
                                  * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))))
                      + 1 * jac1_3
                            * (-1 * jac2_5
                                * (1 * jac3_0
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (-1 * jac2_5
                              * (1 * jac3_1
                                  * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))))
                      + -1 * jac1_1
                            * (-1 * jac2_5
                                * (1 * jac3_0
                                    * (-1 * jac5_4 * (-1 * jac6_6 * jac7_2) + 1 * jac5_6 * (-1 * jac6_4 * jac7_2))))));
    auto com0_5 = -1 * 0;
    auto com1_5 = 0;
    auto com2_5 = -1 * 0;
    auto com3_5 = 0;
    auto com4_5
        = -1
        * (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac4_5 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_5 * jac7_2))))
                    + 1 * jac1_3
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_5 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_5 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac4_5 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_5 * jac7_2))))
                      + 1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_5 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_5 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_5 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_5 * jac7_2))))
                      + -1 * jac1_1
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_5 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_5 * jac7_2))))));
    auto com5_5
        = (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac4_4 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_4 * jac7_2))))
                    + 1 * jac1_3
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac4_4 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_4 * jac7_2))))
                      + 1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_4 * jac7_2))))
                      + -1 * jac1_1
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_4 * jac7_2))))));
    auto com6_5
        = -1
        * (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac4_4 * (-1 * jac6_5 * jac7_2) + 1 * jac4_5 * (-1 * jac6_4 * jac7_2))))
                    + 1 * jac1_3
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (-1 * jac6_5 * jac7_2) + 1 * jac4_5 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac4_4 * (-1 * jac6_5 * jac7_2) + 1 * jac4_5 * (-1 * jac6_4 * jac7_2))))
                      + 1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (-1 * jac6_5 * jac7_2) + 1 * jac4_5 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (-1 * jac6_5 * jac7_2) + 1 * jac4_5 * (-1 * jac6_4 * jac7_2))))
                      + -1 * jac1_1
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (-1 * jac6_5 * jac7_2) + 1 * jac4_5 * (-1 * jac6_4 * jac7_2))))));
    auto com7_5
        = (1 * jac0_0
                * (1 * jac1_1
                        * (-1 * jac2_5
                            * (-1 * jac3_3
                                * (-1 * jac4_4 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_4 * jac7_2))))
                    + 1 * jac1_3
                          * (-1 * jac2_5
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (-1 * jac2_5
                              * (-1 * jac3_3
                                  * (-1 * jac4_4 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_4 * jac7_2))))
                      + 1 * jac1_3
                            * (-1 * jac2_5
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_4 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (-1 * jac2_5
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_4 * jac7_2))))
                      + -1 * jac1_1
                            * (-1 * jac2_5
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (-1 * jac6_6 * jac7_2) + 1 * jac4_6 * (-1 * jac6_4 * jac7_2))))));
    auto com0_6 = 0;
    auto com1_6 = -1 * 0;
    auto com2_6 = 0;
    auto com3_6 = -1 * 0;
    auto com4_6
        = (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac4_5 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_5 * jac7_2))))
                    + 1 * jac1_3
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_5 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_5 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac4_5 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_5 * jac7_2))))
                      + 1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_5 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_5 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_5 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_5 * jac7_2))))
                      + -1 * jac1_1
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_5 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_5 * jac7_2))))));
    auto com5_6
        = -1
        * (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac4_4 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_4 * jac7_2))))
                    + 1 * jac1_3
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac4_4 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_4 * jac7_2))))
                      + 1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_4 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_4 * jac7_2))))
                      + -1 * jac1_1
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_4 * jac7_2))))));
    auto com6_6
        = (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac4_4 * (-1 * jac5_5 * jac7_2) + 1 * jac4_5 * (-1 * jac5_4 * jac7_2))))
                    + 1 * jac1_3
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (-1 * jac5_5 * jac7_2) + 1 * jac4_5 * (-1 * jac5_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac4_4 * (-1 * jac5_5 * jac7_2) + 1 * jac4_5 * (-1 * jac5_4 * jac7_2))))
                      + 1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (-1 * jac5_5 * jac7_2) + 1 * jac4_5 * (-1 * jac5_4 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (-1 * jac5_5 * jac7_2) + 1 * jac4_5 * (-1 * jac5_4 * jac7_2))))
                      + -1 * jac1_1
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (-1 * jac5_5 * jac7_2) + 1 * jac4_5 * (-1 * jac5_4 * jac7_2))))));
    auto com7_6
        = -1
        * (1 * jac0_0
                * (1 * jac1_1
                        * (-1 * jac2_5
                            * (-1 * jac3_3
                                * (-1 * jac4_4 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_4 * jac7_2))))
                    + 1 * jac1_3
                          * (-1 * jac2_5
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_4 * jac7_2)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (-1 * jac2_5
                              * (-1 * jac3_3
                                  * (-1 * jac4_4 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_4 * jac7_2))))
                      + 1 * jac1_3
                            * (-1 * jac2_5
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_4 * jac7_2)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (-1 * jac2_5
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_4 * jac7_2))))
                      + -1 * jac1_1
                            * (-1 * jac2_5
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (-1 * jac5_6 * jac7_2) + 1 * jac4_6 * (-1 * jac5_4 * jac7_2))))));
    auto com0_7 = -1
                * (1 * jac0_1
                        * (-1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_2
                                    * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                        + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                        + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))))
                    + 1 * jac0_3
                          * (1 * jac1_1
                              * (1 * jac2_7
                                  * (1 * jac3_2
                                      * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                          + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                          + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4))))));
    auto com1_7 = (1 * jac0_0
                       * (-1 * jac1_3
                           * (1 * jac2_7
                               * (1 * jac3_2
                                   * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                       + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                       + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))))
                   + 1 * jac0_3
                         * (1 * jac1_0
                             * (1 * jac2_7
                                 * (1 * jac3_2
                                     * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                         + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                         + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4))))));
    auto com2_7 = -1
                * (1 * jac0_0
                        * (1 * jac1_1
                                * (1 * jac2_7
                                    * (1 * jac3_3
                                        * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                            + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                            + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4))))
                            + -1 * jac1_3
                                  * (1 * jac2_7
                                      * (1 * jac3_1
                                          * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                              + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                              + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))))
                    + -1 * jac0_1
                          * (1 * jac1_0
                                  * (1 * jac2_7
                                      * (1 * jac3_3
                                          * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                              + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                              + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4))))
                              + -1 * jac1_3
                                    * (1 * jac2_7
                                        * (1 * jac3_0
                                            * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                                + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                                + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))))
                    + 1 * jac0_3
                          * (1 * jac1_0
                                  * (1 * jac2_7
                                      * (1 * jac3_1
                                          * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                              + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                              + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4))))
                              + -1 * jac1_1
                                    * (1 * jac2_7
                                        * (1 * jac3_0
                                            * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                                + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                                + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4))))));
    auto com3_7 = (1 * jac0_0
                       * (1 * jac1_1
                           * (1 * jac2_7
                               * (1 * jac3_2
                                   * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                       + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                       + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))))
                   + -1 * jac0_1
                         * (1 * jac1_0
                             * (1 * jac2_7
                                 * (1 * jac3_2
                                     * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                         + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                         + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4))))));
    auto com4_7
        = -1
        * (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac4_5 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_5))))
                    + 1 * jac1_3
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_5 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_5)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac4_5 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_5))))
                      + 1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_5 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_5)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_5 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_5))))
                      + -1 * jac1_1
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_5 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_5))))));
    auto com5_7
        = (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac4_4 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_4))))
                    + 1 * jac1_3
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_4)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac4_4 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_4))))
                      + 1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_4)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_4))))
                      + -1 * jac1_1
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_4))))));
    auto com6_7
        = -1
        * (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_7
                            * (-1 * jac3_3
                                * (-1 * jac4_4 * (1 * jac5_2 * jac6_5) + 1 * jac4_5 * (1 * jac5_2 * jac6_4))))
                    + 1 * jac1_3
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (1 * jac5_2 * jac6_5) + 1 * jac4_5 * (1 * jac5_2 * jac6_4)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (-1 * jac3_3
                                  * (-1 * jac4_4 * (1 * jac5_2 * jac6_5) + 1 * jac4_5 * (1 * jac5_2 * jac6_4))))
                      + 1 * jac1_3
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (1 * jac5_2 * jac6_5) + 1 * jac4_5 * (1 * jac5_2 * jac6_4)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (1 * jac2_7
                              * (1 * jac3_1
                                  * (-1 * jac4_4 * (1 * jac5_2 * jac6_5) + 1 * jac4_5 * (1 * jac5_2 * jac6_4))))
                      + -1 * jac1_1
                            * (1 * jac2_7
                                * (1 * jac3_0
                                    * (-1 * jac4_4 * (1 * jac5_2 * jac6_5) + 1 * jac4_5 * (1 * jac5_2 * jac6_4))))));
    auto com7_7
        = (1 * jac0_0
                * (1 * jac1_1
                        * (1 * jac2_2
                                * (1 * jac3_3
                                    * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                        + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                        + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))
                            + -1 * jac2_3
                                  * (1 * jac3_2
                                      * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                          + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                          + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))
                            + -1 * jac2_5
                                  * (-1 * jac3_3
                                      * (-1 * jac4_4 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_4))))
                    + 1 * jac1_3
                          * (-1 * jac2_2
                                  * (1 * jac3_1
                                      * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                          + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                          + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))
                              + -1 * jac2_5
                                    * (1 * jac3_1
                                        * (-1 * jac4_4 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_4)))))
            + -1 * jac0_1
                  * (1 * jac1_0
                          * (1 * jac2_2
                                  * (1 * jac3_3
                                      * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                          + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                          + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))
                              + -1 * jac2_3
                                    * (1 * jac3_2
                                        * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                            + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                            + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))
                              + -1 * jac2_5
                                    * (-1 * jac3_3
                                        * (-1 * jac4_4 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_4))))
                      + 1 * jac1_3
                            * (-1 * jac2_2
                                    * (1 * jac3_0
                                        * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                            + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                            + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))
                                + -1 * jac2_5
                                      * (1 * jac3_0
                                          * (-1 * jac4_4 * (1 * jac5_2 * jac6_6)
                                              + 1 * jac4_6 * (1 * jac5_2 * jac6_4)))))
            + -1 * jac0_3
                  * (1 * jac1_0
                          * (-1 * jac2_2
                                  * (1 * jac3_1
                                      * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                          + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                          + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))
                              + -1 * jac2_5
                                    * (1 * jac3_1
                                        * (-1 * jac4_4 * (1 * jac5_2 * jac6_6) + 1 * jac4_6 * (1 * jac5_2 * jac6_4))))
                      + -1 * jac1_1
                            * (-1 * jac2_2
                                    * (1 * jac3_0
                                        * (1 * jac4_4 * (1 * jac5_5 * jac6_6 + -1 * jac5_6 * jac6_5)
                                            + -1 * jac4_5 * (1 * jac5_4 * jac6_6 + -1 * jac5_6 * jac6_4)
                                            + 1 * jac4_6 * (1 * jac5_4 * jac6_5 + -1 * jac5_5 * jac6_4)))
                                + -1 * jac2_5
                                      * (1 * jac3_0
                                          * (-1 * jac4_4 * (1 * jac5_2 * jac6_6)
                                              + 1 * jac4_6 * (1 * jac5_2 * jac6_4))))));
    Eigen::Matrix<DataType, 8, 8> cojacobian(Eigen::Matrix<DataType, 8, 8>::Zero());

    cojacobian << com0_0, com0_1, com0_2, com0_3, com0_4, com0_5, com0_6, com0_7, com1_0, com1_1, com1_2, com1_3,
        com1_4, com1_5, com1_6, com1_7, com2_0, com2_1, com2_2, com2_3, com2_4, com2_5, com2_6, com2_7, com3_0, com3_1,
        com3_2, com3_3, com3_4, com3_5, com3_6, com3_7, com4_0, com4_1, com4_2, com4_3, com4_4, com4_5, com4_6, com4_7,
        com5_0, com5_1, com5_2, com5_3, com5_4, com5_5, com5_6, com5_7, com6_0, com6_1, com6_2, com6_3, com6_4, com6_5,
        com6_6, com6_7, com7_0, com7_1, com7_2, com7_3, com7_4, com7_5, com7_6, com7_7;
    Eigen::Matrix<DataType, 8, 1> delta = cojacobian * eqs * invdet;

    // Check if the update is big enough
    if(delta.hasNaN() || (delta.array().abs() < EPS).all())
    {
      return true;
    }

    // Big variations are only in steady state mode
    if constexpr(steady_state)
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
};
} // namespace

extern "C"
{
  std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter()
  {
    return std::make_unique<StaticFilter>();
  }
} // namespace
