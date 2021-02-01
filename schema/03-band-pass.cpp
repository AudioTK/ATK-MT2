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
  mutable Eigen::Matrix<DataType, 2, 1> dynamic_state{Eigen::Matrix<DataType, 2, 1>::Zero()};
  Eigen::Matrix<DataType, 2, 2> inverse;
  ATK::StaticResistor<DataType> r045{10000};
  ATK::StaticResistor<DataType> r042{10000};
  ATK::StaticCapacitor<DataType> c031{4.7e-08};
  ATK::StaticCapacitor<DataType> c029{3.3e-08};
  ATK::StaticResistor<DataType> r040{100000};

public:
  StaticFilter(): ModellerFilter<DataType>(2, 1), inverse(2, 2)
  {
    static_state << 0.000000;
  }

  gsl::index get_nb_dynamic_pins() const override
  {
    return 2;
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
    return 5;
  }

  std::string get_dynamic_pin_name(gsl::index identifier) const override
  {
    switch(identifier)
    {
    case 1:
      return "vout";
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
    Eigen::Matrix<DataType, 2, 2> jacobian(Eigen::Matrix<DataType, 2, 2>::Zero());
    auto jac0_0 = 0 - r045.get_gradient() - r042.get_gradient() - (steady_state ? 0 : c031.get_gradient())
                - (steady_state ? 0 : c029.get_gradient());
    auto jac0_1 = 0 + (steady_state ? 0 : c029.get_gradient());
    auto jac1_0 = 0 + (steady_state ? 0 : c029.get_gradient());
    auto jac1_1 = 0 - (steady_state ? 0 : c029.get_gradient()) - r040.get_gradient();
    jacobian << jac0_0, jac0_1, jac1_0, jac1_1;
    inverse = jacobian.inverse();
  }

  void init()
  {
    // update_steady_state
    c031.update_steady_state(1. / input_sampling_rate, dynamic_state[0], static_state[0]);
    c029.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[1]);

    solve<true>();

    // update_steady_state
    c031.update_steady_state(1. / input_sampling_rate, dynamic_state[0], static_state[0]);
    c029.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[1]);

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
      c031.update_state(dynamic_state[0], static_state[0]);
      c029.update_state(dynamic_state[0], dynamic_state[1]);
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

    // Precomputes

    Eigen::Matrix<DataType, 2, 1> eqs(Eigen::Matrix<DataType, 2, 1>::Zero());
    auto eq0 = -r045.get_current(i0_, d0_) + r042.get_current(d0_, s0_)
             + (steady_state ? 0 : c031.get_current(d0_, s0_)) + (steady_state ? 0 : c029.get_current(d0_, d1_));
    auto eq1 = -(steady_state ? 0 : c029.get_current(d0_, d1_)) + r040.get_current(d1_, s0_);
    eqs << eq0, eq1;

    // Check if the equations have converged
    if((eqs.array().abs() < EPS).all())
    {
      return true;
    }

    Eigen::Matrix<DataType, 2, 1> delta = inverse * eqs;

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

extern "C"
{
  std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter()
  {
    return std::make_unique<StaticFilter>();
  }
} // namespace
