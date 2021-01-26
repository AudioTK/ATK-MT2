#include <cstdlib>
#include <memory>

#include <ATK/Core/Utilities.h>

#include <ATK/Modelling/ModellerFilter.h>
#include <ATK/Modelling/StaticCapacitor.h>
#include <ATK/Modelling/StaticCoil.h>
#include <ATK/Modelling/StaticCurrent.h>
#include <ATK/Modelling/StaticDiode.h>
#include <ATK/Modelling/StaticResistor.h>
#include <ATK/Modelling/StaticTransistor.h>

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

  Eigen::Matrix<DataType, 1, 1> static_state{Eigen::Matrix<DataType, 1, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 1, 1> input_state{Eigen::Matrix<DataType, 1, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 2, 1> dynamic_state{Eigen::Matrix<DataType, 2, 1>::Zero()};
  StaticResistor<DataType> r1{10000};
  StaticResistor<DataType> r2{10000};
  StaticCapacitor<DataType> c1{4.7e-08};
  StaticCapacitor<DataType> c2{3.3e-08};
  StaticResistor<DataType> r2{100000};

public:
  StaticFilter(): ModellerFilter<DataType>(2, 1)
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
      return "1";
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
      auto target_static_state = static_state;

      for(gsl::index i = 0; i < INIT_WARMUP; ++i)
      {
        static_state = target_static_state * ((i + 1.) / INIT_WARMUP);
        init();
      }
      static_state = target_static_state;
    }
  }

  void init()
  {
    // update_steady_state
    c1.update_steady_state(1. / input_sampling_rate, dynamic_state[0], static_state[0]);
    c2.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[1]);

    solve<true>();

    // update_steady_state
    c1.update_steady_state(1. / input_sampling_rate, dynamic_state[0], static_state[0]);
    c2.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[1]);

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
      c1.update_state(dynamic_state[0], static_state[0]);
      c2.update_state(dynamic_state[0], dynamic_state[1]);
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

    Eigen::Matrix<DataType, 2, 1> eqs(Eigen::Matrix<DataType, 2, 1>::Zero());
    auto eq0 = -r1.get_current(i0_, d0_) + r2.get_current(d0_, s0_) + (steady_state ? 0 : c1.get_current(d0_, s0_))
             + (steady_state ? 0 : c2.get_current(d0_, d1_));
    auto eq1 = -(steady_state ? 0 : c2.get_current(d0_, d1_)) + r2.get_current(d1_, s0_);
    eqs << eq0, eq1;

    // Check if the equations have converged
    if((eqs.array().abs() < EPS).all())
    {
      return true;
    }

    auto jac0_0 = 0 - r1.get_gradient() - r2.get_gradient() - (steady_state ? 0 : c1.get_gradient())
                - (steady_state ? 0 : c2.get_gradient());
    auto jac0_1 = 0 + (steady_state ? 0 : c2.get_gradient());
    auto jac1_0 = 0 + (steady_state ? 0 : c2.get_gradient());
    auto jac1_1 = 0 - (steady_state ? 0 : c2.get_gradient()) - r2.get_gradient();
    auto det = (jac0_0 * jac1_1 + jac0_1 * jac1_0);
    auto invdet = 1 / det;
    auto com0_0 = jac1_1;
    auto com1_0 = -1 * jac1_0;
    auto com0_1 = -1 * jac0_1;
    auto com1_1 = jac0_0;
    Eigen::Matrix<DataType, 2, 2> cojacobian(Eigen::Matrix<DataType, 2, 2>::Zero());

    cojacobian << com0_0, com0_1, com1_0, com1_1;
    Eigen::Matrix<DataType, 2, 1> delta = cojacobian * eqs * invdet;

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
};
} // namespace

extern "C"
{
  std::unique_ptr<ATK::ModellerFilter<double>> createStaticFilter()
  {
    return std::make_unique<StaticFilter>();
  }
}
