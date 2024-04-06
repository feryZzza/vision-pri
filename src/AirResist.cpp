#include "AirResist.hpp"

// 微分方程模型
void AirResist::model(const std::vector<double> &z, std::vector<double> &dzdt, double t,
           double k1, double g) {
  double vx = z[1];
  double vy = z[3];
  dzdt[0] = vx;
  dzdt[1] = -k1 * vx * std::abs(vx);                  // x方向的阻力
  dzdt[2] = vy;
  dzdt[3] = g - k1 * vy * std::abs(vy);              // y方向的阻力和重力
}

// Observer结构记录轨迹数据
void AirResist::StateObserver::operator()(const std::vector<double> &z, double t) {
  states.push_back(z);
}

// 目标函数
double AirResist::objective(const std::vector<double> &x, std::vector<double> &grad, void *data) {
  auto params = reinterpret_cast<
      std::tuple<double, std::pair<double, double>, double, double, double> *>(
      data);
  double k1 = std::get<0>(*params);
  auto point_a = std::get<1>(*params);
  double t_start = std::get<2>(*params);
  double t_end = std::get<3>(*params);
  double kv = std::get<4>(*params);  // 获取传入的 kv 值

  double theta = x[0] * M_PI / 180.0;  // 角度转弧度

  // 初始状态
  std::vector<double> z0 = {0, kv * std::cos(theta), 0, kv * std::sin(theta)};

  // 积分器
  using namespace boost::numeric::odeint;
  typedef runge_kutta4<std::vector<double>> stepper_type;  // rk4
  StateObserver observer;

  // integrate_const(stepper_type(),
  //                std::bind(AirResist::model, std::placeholders::_1, std::placeholders::_2,
  //                          std::placeholders::_3, k1, 20),
  //                z0, t_start, t_end, 0.01, std::ref(observer));
  // 在 C++ 中，绑定类的成员函数比绑定普通函数或静态成员函数要复杂一些，因为成员函数需要一个对象上下文来执行
  // AirResist::model 是一个非静态成员函数，所以它需要一个 AirResist 类型的对象来调用，但是在 std::bind 中，没有提供这样的对象。
  // 可以使用 this 指针来引用当前对象实例
  integrate_const(stepper_type(),
                  std::bind(&AirResist::model,this, std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3, k1, 9.788),
                  z0, t_start, t_end, 0.005, std::ref(observer));
  // 可以调整积分步长
  
  // 应对段错误的处理
  if (grad.size() < 1) {
    grad.resize(1);
  }

  double min_distance = std::numeric_limits<double>::max();
  for (const auto &state : observer.states) {
    double distance_x = std::abs(state[0] - point_a.first);
    if (distance_x < min_distance) {
      min_distance = distance_x;
      grad[0] = 2 * (state[2] - point_a.second);
    }
  }

  return grad[0] * grad[0];
}

// 静态目标函数包装器
double AirResist::ObjectiveWrapper(const std::vector<double> &x, std::vector<double> &grad, void *data) {
    // 解包数据以获取 AirResist 实例和其他参数
    auto unpackedData = reinterpret_cast<std::tuple<AirResist*, std::tuple<double, std::pair<double, double>, double, double, double>>*>(data);
    AirResist* instance = std::get<0>(*unpackedData);
    auto& params = std::get<1>(*unpackedData);

    // 调用实际的成员函数
    return instance->objective(x, grad, &params);
}

// 优化函数
cv::Vec2f AirResist::AirResistSolve(cv::Point2f point_a, double kv) {
  double k1 = 0.0001949;                      // 阻力系数(m)
  double t_start = 0.0;                       // 积分开始时间
  double t_end = 4.0;                         // 积分结束时间 视具体情况定，时间越长，计算开销越大
  nlopt::opt optimizer(nlopt::LN_COBYLA, 1);  
  std::vector<double> lb(1, -90);             // 下界
  std::vector<double> ub(1,90);               // 上界
  optimizer.set_lower_bounds(lb);
  optimizer.set_upper_bounds(ub);
  optimizer.set_xtol_rel(1e-6);

  std::vector<double> x(1, 5);  // 初始猜测值
  std::tuple<double, std::pair<double, double>, double, double, double> data(
      k1, {point_a.x, point_a.y}, t_start, t_end, kv);

  // optimizer.set_min_objective(AirResist::objective,&data);
  // 这行代码的使用方式:这里试图将非静态成员函数 AirResist::objective 直接作为函数指针传递给 set_min_objective，
  // 这是不允许的，因为非静态成员函数需要一个类的实例来调用
  
  // 解决这个问题的一种方法是使用 std::bind，就像之前提到的 
  // 但是，这里有一个陷阱：std::bind 生成的可调用对象不能直接转换为C函数指针，而 nlopt 库需要一个C函数指针作为目标函数。
  // 因此，这里需要一个静态函数或非成员函数来充当桥梁,所以创建ObjectiveWrapper作为静态目标函数包装器

  // 也可以用同样的方式来处理data
  auto wrappedData = std::make_tuple(this, data);
  optimizer.set_min_objective(AirResist::ObjectiveWrapper, &wrappedData);


  double minf;  // 存储最优化结果的变量
  try {
    nlopt::result result = optimizer.optimize(x, minf);
    std::cout << "Found minimum at theta = " << x[0]
              << " degrees with deviation = " << minf << std::endl;
  } catch (std::exception &e) {
    std::cerr << "nlopt failed: " << e.what() << std::endl;
  }

  return x[0]*(3.1415976/180);
}

cv::Vec2f AirResist::ParabolSolve(cv::Point2f point_a, float kv) {
  // x1=v*cos@*t
  // y1=v*sin@t-g*t^2/2
  // 联立方程消去t,得关于出射角tan@的方程kg*x1*x1/(2*kv*kv)*tan@^2+x1*tan@+kg*x1*x1/(2*kv*kv)-y1=0
  kv *= 100;
  float kg = 978.8f;
  float x1 = point_a.x*100, y1 = point_a.y*100;
  float a = kg * x1 * x1 / (2 * kv * kv), b = -x1,
        c = kg * x1 * x1 / (2 * kv * kv) - y1;
  if (a == 0) {
    std::cout << "a = 0" << std::endl;
  }
  float tan_phi0 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a),
        tan_phi1 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);

  if (b * b - 4 * a * c <= 0) {
    return {-1, -1};
  }
  if (kv == 0) {
    kv = 700;
  }             // 这又是解决什么bug
  float phi0 = atan(tan_phi0), phi1 = atan(tan_phi1);
  if (isnan(phi0) || isnan(phi1)) {
    return {-1, -1};
  }
  cv::Vec2f ret = {-phi0, -phi1};
  //std::cout << ret << std::endl;
  return ret;
}

// TODO: 解决k1=0时和抛物线模型数据出入过大的问题
// TODO：解决在5m到7m这段距离内模型结果比抛物线还小的问题
// TODO：模型改进，可以考虑在垂直方向的升力作用