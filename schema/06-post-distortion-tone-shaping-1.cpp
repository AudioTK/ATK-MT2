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

  Eigen::Matrix<DataType, 3, 1> static_state{Eigen::Matrix<DataType, 3, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 1, 1> input_state{Eigen::Matrix<DataType, 1, 1>::Zero()};
  mutable Eigen::Matrix<DataType, 8, 1> dynamic_state{Eigen::Matrix<DataType, 8, 1>::Zero()};
  StaticResistor<DataType> r030{3300};
  StaticCapacitor<DataType> c022{4.7e-11};
  StaticCapacitor<DataType> c020{2.2e-08};
  StaticResistor<DataType> r027{470};
  StaticResistor<DataType> r024{10000};
  StaticCapacitor<DataType> c017{4.7e-08};
  StaticResistor<DataType> r025{470000};
  StaticNPN<DataType> q007{1e-12, 0.026, 1, 1, 100};
  StaticCapacitor<DataType> c024{1.5e-08};
  StaticResistor<DataType> r034{1000};
  StaticResistor<DataType> r037{10000};
  StaticCapacitor<DataType> c025{1.5e-09};
  StaticResistor<DataType> r036{47000};
  StaticNPN<DataType> q008{1e-12, 0.026, 1, 1, 100};

public:
  StaticFilter(): ModellerFilter<DataType>(8, 1)
  {
    static_state << 0.000000, -4.500000, 4.500000;
  }

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
    case 5:
      return "6";
    case 6:
      return "7";
    case 4:
      return "5";
    case 1:
      return "vout";
    case 7:
      return "8";
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
    c022.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[1]);
    c020.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[2]);
    c017.update_steady_state(1. / input_sampling_rate, dynamic_state[2], dynamic_state[4]);
    c024.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[5]);
    c025.update_steady_state(1. / input_sampling_rate, dynamic_state[5], dynamic_state[7]);

    solve<true>();

    // update_steady_state
    c022.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[1]);
    c020.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[2]);
    c017.update_steady_state(1. / input_sampling_rate, dynamic_state[2], dynamic_state[4]);
    c024.update_steady_state(1. / input_sampling_rate, dynamic_state[0], dynamic_state[5]);
    c025.update_steady_state(1. / input_sampling_rate, dynamic_state[5], dynamic_state[7]);

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
      c022.update_state(dynamic_state[0], dynamic_state[1]);
      c020.update_state(dynamic_state[0], dynamic_state[2]);
      c017.update_state(dynamic_state[2], dynamic_state[4]);
      c024.update_state(dynamic_state[0], dynamic_state[5]);
      c025.update_state(dynamic_state[5], dynamic_state[7]);
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
    q007.precompute(dynamic_state[4], static_state[2], dynamic_state[3]);
    q008.precompute(dynamic_state[7], static_state[2], dynamic_state[6]);

    Eigen::Matrix<DataType, 8, 1> eqs(Eigen::Matrix<DataType, 8, 1>::Zero());
    auto eq0 = +r030.get_current(d0_, d1_) + (steady_state ? 0 : c022.get_current(d0_, d1_))
             + (steady_state ? 0 : c020.get_current(d0_, d2_)) + (steady_state ? 0 : c024.get_current(d0_, d5_));
    auto eq1 = input_state[0] - dynamic_state[0];
    auto eq2 = -(steady_state ? 0 : c020.get_current(d0_, d2_)) + r027.get_current(d2_, d3_)
             + (steady_state ? 0 : c017.get_current(d2_, d4_));
    auto eq3 = -r027.get_current(d2_, d3_) + r024.get_current(d3_, s1_) + q007.ib() + q007.ic();
    auto eq4 = -(steady_state ? 0 : c017.get_current(d2_, d4_)) - r025.get_current(s0_, d4_) - q007.ib();
    auto eq5 = -(steady_state ? 0 : c024.get_current(d0_, d5_)) + r034.get_current(d5_, d6_)
             + (steady_state ? 0 : c025.get_current(d5_, d7_));
    auto eq6 = -r034.get_current(d5_, d6_) + r037.get_current(d6_, s1_) + q008.ib() + q008.ic();
    auto eq7 = -(steady_state ? 0 : c025.get_current(d5_, d7_)) - r036.get_current(s0_, d7_) - q008.ib();
    eqs << eq0, eq1, eq2, eq3, eq4, eq5, eq6, eq7;

    // Check if the equations have converged
    if((eqs.array().abs() < EPS).all())
    {
      return true;
    }

    Eigen::Matrix<DataType, 8, 8> jacobian(Eigen::Matrix<DataType, 8, 8>::Zero());
    auto jac0_0 = 0 - r030.get_gradient() - (steady_state ? 0 : c022.get_gradient())
                - (steady_state ? 0 : c020.get_gradient()) - (steady_state ? 0 : c024.get_gradient());
    auto jac0_1 = 0 + r030.get_gradient() + (steady_state ? 0 : c022.get_gradient());
    auto jac0_2 = 0 + (steady_state ? 0 : c020.get_gradient());
    auto jac0_3 = 0;
    auto jac0_4 = 0;
    auto jac0_5 = 0 + (steady_state ? 0 : c024.get_gradient());
    auto jac0_6 = 0;
    auto jac0_7 = 0;
    auto jac1_0 = 0 + -1;
    auto jac1_1 = 0;
    auto jac1_2 = 0;
    auto jac1_3 = 0;
    auto jac1_4 = 0;
    auto jac1_5 = 0;
    auto jac1_6 = 0;
    auto jac1_7 = 0;
    auto jac2_0 = 0 + (steady_state ? 0 : c020.get_gradient());
    auto jac2_1 = 0;
    auto jac2_2
        = 0 - (steady_state ? 0 : c020.get_gradient()) - r027.get_gradient() - (steady_state ? 0 : c017.get_gradient());
    auto jac2_3 = 0 + r027.get_gradient();
    auto jac2_4 = 0 + (steady_state ? 0 : c017.get_gradient());
    auto jac2_5 = 0;
    auto jac2_6 = 0;
    auto jac2_7 = 0;
    auto jac3_0 = 0;
    auto jac3_1 = 0;
    auto jac3_2 = 0 + r027.get_gradient();
    auto jac3_3 = 0 - r027.get_gradient() - r024.get_gradient() - q007.ib_Vbe() - q007.ic_Vbe();
    auto jac3_4 = 0 + q007.ib_Vbc() + q007.ib_Vbe() + q007.ic_Vbc() + q007.ic_Vbe();
    auto jac3_5 = 0;
    auto jac3_6 = 0;
    auto jac3_7 = 0;
    auto jac4_0 = 0;
    auto jac4_1 = 0;
    auto jac4_2 = 0 + (steady_state ? 0 : c017.get_gradient());
    auto jac4_3 = 0 + q007.ib_Vbe();
    auto jac4_4 = 0 - (steady_state ? 0 : c017.get_gradient()) - r025.get_gradient() - q007.ib_Vbc() - q007.ib_Vbe();
    auto jac4_5 = 0;
    auto jac4_6 = 0;
    auto jac4_7 = 0;
    auto jac5_0 = 0 + (steady_state ? 0 : c024.get_gradient());
    auto jac5_1 = 0;
    auto jac5_2 = 0;
    auto jac5_3 = 0;
    auto jac5_4 = 0;
    auto jac5_5
        = 0 - (steady_state ? 0 : c024.get_gradient()) - r034.get_gradient() - (steady_state ? 0 : c025.get_gradient());
    auto jac5_6 = 0 + r034.get_gradient();
    auto jac5_7 = 0 + (steady_state ? 0 : c025.get_gradient());
    auto jac6_0 = 0;
    auto jac6_1 = 0;
    auto jac6_2 = 0;
    auto jac6_3 = 0;
    auto jac6_4 = 0;
    auto jac6_5 = 0 + r034.get_gradient();
    auto jac6_6 = 0 - r034.get_gradient() - r037.get_gradient() - q008.ib_Vbe() - q008.ic_Vbe();
    auto jac6_7 = 0 + q008.ib_Vbc() + q008.ib_Vbe() + q008.ic_Vbc() + q008.ic_Vbe();
    auto jac7_0 = 0;
    auto jac7_1 = 0;
    auto jac7_2 = 0;
    auto jac7_3 = 0;
    auto jac7_4 = 0;
    auto jac7_5 = 0 + (steady_state ? 0 : c025.get_gradient());
    auto jac7_6 = 0 + q008.ib_Vbe();
    auto jac7_7 = 0 - (steady_state ? 0 : c025.get_gradient()) - r036.get_gradient() - q008.ib_Vbc() - q008.ib_Vbe();
    jacobian << jac0_0, jac0_1, jac0_2, jac0_3, jac0_4, jac0_5, jac0_6, jac0_7, jac1_0, jac1_1, jac1_2, jac1_3, jac1_4,
        jac1_5, jac1_6, jac1_7, jac2_0, jac2_1, jac2_2, jac2_3, jac2_4, jac2_5, jac2_6, jac2_7, jac3_0, jac3_1, jac3_2,
        jac3_3, jac3_4, jac3_5, jac3_6, jac3_7, jac4_0, jac4_1, jac4_2, jac4_3, jac4_4, jac4_5, jac4_6, jac4_7, jac5_0,
        jac5_1, jac5_2, jac5_3, jac5_4, jac5_5, jac5_6, jac5_7, jac6_0, jac6_1, jac6_2, jac6_3, jac6_4, jac6_5, jac6_6,
        jac6_7, jac7_0, jac7_1, jac7_2, jac7_3, jac7_4, jac7_5, jac7_6, jac7_7;
    Eigen::Matrix<DataType, 8, 1> delta = jacobian.colPivHouseholderQr().solve(eqs);

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
        {0, 2},
        {0, 5},
        {1, 0},
        {2, 0},
        {2, 2},
        {2, 3},
        {2, 4},
        {3, 2},
        {3, 3},
        {3, 4},
        {4, 2},
        {4, 3},
        {4, 4},
        {5, 0},
        {5, 5},
        {5, 6},
        {5, 7},
        {6, 5},
        {6, 6},
        {6, 7},
        {7, 5},
        {7, 6},
        {7, 7}};
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
