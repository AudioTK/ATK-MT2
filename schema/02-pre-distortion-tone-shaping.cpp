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
constexpr gsl::index MAX_ITERATION = 10;
constexpr gsl::index MAX_ITERATION_STEADY_STATE{200};

constexpr gsl::index INIT_WARMUP{10};
constexpr double EPS{1e-8};
constexpr double MAX_DELTA{1e-1};

class StaticFilter: public ATK::ModellerFilter<double>
{
  using typename ATK::TypedBaseFilter<double>::DataType;
  bool initialized{false};

  Eigen::Matrix<DataType, 3, 1> static_state{Eigen::Matrix<DataType, 3, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 1, 1> input_state{Eigen::Matrix<DataType, 1, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 5, 1> dynamic_state{Eigen::Matrix<DataType, 5, 1>::Zero()};
  ATK::StaticResistor<DataType> r044{220000};
  ATK::StaticCapacitor<DataType> c032{1e-10};
  ATK::StaticCapacitor<DataType> c034{2.7e-08};
  ATK::StaticResistor<DataType> r046{2200};
  ATK::StaticResistor<DataType> r054{10000};
  ATK::StaticCapacitor<DataType> c035{1e-08};
  ATK::StaticResistor<DataType> r053{47000};
  ATK::StaticNPN<DataType> q010{1e-12, 0.026, 1, 1, 100};

public:
  StaticFilter(): ModellerFilter<DataType>(5, 1)
  {
    static_state << 0.000000, -4.500000, 4.500000;
  }

  gsl::index get_nb_dynamic_pins() const override
  {
    return 5;
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
    return 9;
  }

  std::string get_dynamic_pin_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 4:
      return "5";
    case 1:
      return "vout";
    case 3:
      return "4";
    case 2:
      return "3";
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
    case 1:
      return "vdd";
    case 2:
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
    c032.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[1]);
    c034.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[2]);
    c035.update_steady_state(1. / input_sampling_rate, dynamic_state[2], dynamic_state[4]);

    solve<true>();

    // update_steady_state
    c032.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[1]);
    c034.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[2]);
    c035.update_steady_state(1. / input_sampling_rate, dynamic_state[2], dynamic_state[4]);

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
      c032.update_state(dynamic_state[0], dynamic_state[1]);
      c034.update_state(dynamic_state[0], dynamic_state[2]);
      c035.update_state(dynamic_state[2], dynamic_state[4]);
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

    // Precomputes
    q010.precompute(dynamic_state[4], static_state[2], dynamic_state[3]);

    Eigen::Matrix<DataType, 5, 1> eqs(Eigen::Matrix<DataType, 5, 1>::Zero());
    auto eq0 = +r044.get_current(d0_, d1_) + (steady_state ? 0 : c032.get_current(d0_, d1_))
             + (steady_state ? 0 : c034.get_current(d0_, d2_));
    auto eq1 = input_state[0] - dynamic_state[0];
    auto eq2 = -(steady_state ? 0 : c034.get_current(d0_, d2_)) + r046.get_current(d2_, d3_)
             + (steady_state ? 0 : c035.get_current(d2_, d4_));
    auto eq3 = -r046.get_current(d2_, d3_) + r054.get_current(d3_, s1_) + q010.ib() + q010.ic();
    auto eq4 = -(steady_state ? 0 : c035.get_current(d2_, d4_)) - r053.get_current(s0_, d4_) - q010.ib();
    eqs << eq0, eq1, eq2, eq3, eq4;

    // Check if the equations have converged
    if((eqs.array().abs() < EPS).all())
    {
      return true;
    }

    auto jac0_0
        = 0 - r044.get_gradient() - (steady_state ? 0 : c032.get_gradient()) - (steady_state ? 0 : c034.get_gradient());
    auto jac0_1 = 0 + r044.get_gradient() + (steady_state ? 0 : c032.get_gradient());
    auto jac0_2 = 0 + (steady_state ? 0 : c034.get_gradient());
    auto jac0_3 = 0;
    auto jac0_4 = 0;
    auto jac1_0 = 0 + -1;
    auto jac1_1 = 0;
    auto jac1_2 = 0;
    auto jac1_3 = 0;
    auto jac1_4 = 0;
    auto jac2_0 = 0 + (steady_state ? 0 : c034.get_gradient());
    auto jac2_1 = 0;
    auto jac2_2
        = 0 - (steady_state ? 0 : c034.get_gradient()) - r046.get_gradient() - (steady_state ? 0 : c035.get_gradient());
    auto jac2_3 = 0 + r046.get_gradient();
    auto jac2_4 = 0 + (steady_state ? 0 : c035.get_gradient());
    auto jac3_0 = 0;
    auto jac3_1 = 0;
    auto jac3_2 = 0 + r046.get_gradient();
    auto jac3_3 = 0 - r046.get_gradient() - r054.get_gradient() - q010.ib_Vbe() - q010.ic_Vbe();
    auto jac3_4 = 0 + q010.ib_Vbc() + q010.ib_Vbe() + q010.ic_Vbc() + q010.ic_Vbe();
    auto jac4_0 = 0;
    auto jac4_1 = 0;
    auto jac4_2 = 0 + (steady_state ? 0 : c035.get_gradient());
    auto jac4_3 = 0 + q010.ib_Vbe();
    auto jac4_4 = 0 - (steady_state ? 0 : c035.get_gradient()) - r053.get_gradient() - q010.ib_Vbc() - q010.ib_Vbe();
    auto det = (-1 * jac0_1
                * (1 * jac1_0
                    * (1 * jac2_2 * (1 * jac3_3 * jac4_4 + -1 * jac3_4 * jac4_3)
                        + -1 * jac2_3 * (1 * jac3_2 * jac4_4 + -1 * jac3_4 * jac4_2)
                        + 1 * jac2_4 * (1 * jac3_2 * jac4_3 + -1 * jac3_3 * jac4_2))));
    auto invdet = 1 / det;
    auto com0_0 = 0;
    auto com1_0 = -1
                * (1 * jac1_0
                    * (1 * jac2_2 * (1 * jac3_3 * jac4_4 + -1 * jac3_4 * jac4_3)
                        + -1 * jac2_3 * (1 * jac3_2 * jac4_4 + -1 * jac3_4 * jac4_2)
                        + 1 * jac2_4 * (1 * jac3_2 * jac4_3 + -1 * jac3_3 * jac4_2)));
    auto com2_0 = 0;
    auto com3_0 = -1 * 0;
    auto com4_0 = 0;
    auto com0_1 = -1
                * (1 * jac0_1
                    * (1 * jac2_2 * (1 * jac3_3 * jac4_4 + -1 * jac3_4 * jac4_3)
                        + -1 * jac2_3 * (1 * jac3_2 * jac4_4 + -1 * jac3_4 * jac4_2)
                        + 1 * jac2_4 * (1 * jac3_2 * jac4_3 + -1 * jac3_3 * jac4_2)));
    auto com1_1 = (1 * jac0_0
                       * (1 * jac2_2 * (1 * jac3_3 * jac4_4 + -1 * jac3_4 * jac4_3)
                           + -1 * jac2_3 * (1 * jac3_2 * jac4_4 + -1 * jac3_4 * jac4_2)
                           + 1 * jac2_4 * (1 * jac3_2 * jac4_3 + -1 * jac3_3 * jac4_2))
                   + -1 * jac0_2 * (1 * jac2_0 * (1 * jac3_3 * jac4_4 + -1 * jac3_4 * jac4_3)));
    auto com2_1 = -1 * (-1 * jac0_1 * (1 * jac2_0 * (1 * jac3_3 * jac4_4 + -1 * jac3_4 * jac4_3)));
    auto com3_1 = (-1 * jac0_1 * (1 * jac2_0 * (1 * jac3_2 * jac4_4 + -1 * jac3_4 * jac4_2)));
    auto com4_1 = -1 * (-1 * jac0_1 * (1 * jac2_0 * (1 * jac3_2 * jac4_3 + -1 * jac3_3 * jac4_2)));
    auto com0_2 = 0;
    auto com1_2 = -1 * (-1 * jac0_2 * (1 * jac1_0 * (1 * jac3_3 * jac4_4 + -1 * jac3_4 * jac4_3)));
    auto com2_2 = (-1 * jac0_1 * (1 * jac1_0 * (1 * jac3_3 * jac4_4 + -1 * jac3_4 * jac4_3)));
    auto com3_2 = -1 * (-1 * jac0_1 * (1 * jac1_0 * (1 * jac3_2 * jac4_4 + -1 * jac3_4 * jac4_2)));
    auto com4_2 = (-1 * jac0_1 * (1 * jac1_0 * (1 * jac3_2 * jac4_3 + -1 * jac3_3 * jac4_2)));
    auto com0_3 = -1 * 0;
    auto com1_3 = (-1 * jac0_2 * (1 * jac1_0 * (1 * jac2_3 * jac4_4 + -1 * jac2_4 * jac4_3)));
    auto com2_3 = -1 * (-1 * jac0_1 * (1 * jac1_0 * (1 * jac2_3 * jac4_4 + -1 * jac2_4 * jac4_3)));
    auto com3_3 = (-1 * jac0_1 * (1 * jac1_0 * (1 * jac2_2 * jac4_4 + -1 * jac2_4 * jac4_2)));
    auto com4_3 = -1 * (-1 * jac0_1 * (1 * jac1_0 * (1 * jac2_2 * jac4_3 + -1 * jac2_3 * jac4_2)));
    auto com0_4 = 0;
    auto com1_4 = -1 * (-1 * jac0_2 * (1 * jac1_0 * (1 * jac2_3 * jac3_4 + -1 * jac2_4 * jac3_3)));
    auto com2_4 = (-1 * jac0_1 * (1 * jac1_0 * (1 * jac2_3 * jac3_4 + -1 * jac2_4 * jac3_3)));
    auto com3_4 = -1 * (-1 * jac0_1 * (1 * jac1_0 * (1 * jac2_2 * jac3_4 + -1 * jac2_4 * jac3_2)));
    auto com4_4 = (-1 * jac0_1 * (1 * jac1_0 * (1 * jac2_2 * jac3_3 + -1 * jac2_3 * jac3_2)));
    Eigen::Matrix<DataType, 5, 5> cojacobian(Eigen::Matrix<DataType, 5, 5>::Zero());

    cojacobian << com0_0, com0_1, com0_2, com0_3, com0_4, com1_0, com1_1, com1_2, com1_3, com1_4, com2_0, com2_1,
        com2_2, com2_3, com2_4, com3_0, com3_1, com3_2, com3_3, com3_4, com4_0, com4_1, com4_2, com4_3, com4_4;
    Eigen::Matrix<DataType, 5, 1> delta = cojacobian * eqs * invdet;

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
