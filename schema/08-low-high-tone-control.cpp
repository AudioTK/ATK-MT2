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

  Eigen::Matrix<DataType, 1, 1> static_state{Eigen::Matrix<DataType, 1, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 1, 1> input_state{Eigen::Matrix<DataType, 1, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 9, 1> dynamic_state{Eigen::Matrix<DataType, 9, 1>::Zero()};
  StaticResistor<DataType> r014{22000};
  StaticCapacitor<DataType> c010{1e-11};
  StaticResistor<DataType> r015{22000};
  DataType pr03a{100000};
  DataType pr03a_trimmer{0};
  StaticResistor<DataType> r061{2200};
  StaticCapacitor<DataType> c044{1e-08};
  DataType pr03b{100000};
  DataType pr03b_trimmer{0};
  StaticCapacitor<DataType> c008{2.2e-07};
  StaticResistor<DataType> r012{2200};
  StaticCapacitor<DataType> c009{4.7e-08};
  StaticResistor<DataType> r013{1e+06};

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
    return 0;
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

    Eigen::Matrix<DataType, 9, 9> jacobian(Eigen::Matrix<DataType, 9, 9>::Zero());
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
    jacobian << jac0_0, jac0_1, jac0_2, jac0_3, jac0_4, jac0_5, jac0_6, jac0_7, jac0_8, jac1_0, jac1_1, jac1_2, jac1_3,
        jac1_4, jac1_5, jac1_6, jac1_7, jac1_8, jac2_0, jac2_1, jac2_2, jac2_3, jac2_4, jac2_5, jac2_6, jac2_7, jac2_8,
        jac3_0, jac3_1, jac3_2, jac3_3, jac3_4, jac3_5, jac3_6, jac3_7, jac3_8, jac4_0, jac4_1, jac4_2, jac4_3, jac4_4,
        jac4_5, jac4_6, jac4_7, jac4_8, jac5_0, jac5_1, jac5_2, jac5_3, jac5_4, jac5_5, jac5_6, jac5_7, jac5_8, jac6_0,
        jac6_1, jac6_2, jac6_3, jac6_4, jac6_5, jac6_6, jac6_7, jac6_8, jac7_0, jac7_1, jac7_2, jac7_3, jac7_4, jac7_5,
        jac7_6, jac7_7, jac7_8, jac8_0, jac8_1, jac8_2, jac8_3, jac8_4, jac8_5, jac8_6, jac8_7, jac8_8;
    Eigen::Matrix<DataType, 9, 1> delta = jacobian.colPivHouseholderQr().solve(eqs);

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
        {0, 3},
        {0, 5},
        {1, 1},
        {1, 2},
        {1, 3},
        {1, 5},
        {2, 0},
        {2, 1},
        {3, 0},
        {3, 1},
        {3, 3},
        {3, 4},
        {4, 3},
        {4, 4},
        {5, 0},
        {5, 1},
        {5, 5},
        {5, 6},
        {6, 5},
        {6, 6},
        {6, 7},
        {6, 8},
        {7, 7},
        {7, 8},
        {8, 6},
        {8, 8}};
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
