/*
 * orca::ThrustCurve：把 ArduPilotPlugin 送出的 PWM 查表換成推力，轉給 gz 的 Thruster 外掛。
 *
 * 為什麼要這一層：ArduPilotPlugin 的 COMMAND 只能把 PWM 線性換算成指令，但 T200 有死區
 * （約 1472～1528 µs 推力為 0），小推力也遠比線性小。實機的 PWM 大多只偏離中位幾十 µs，
 * 線性換算會把這一段高估好幾倍（實機紀錄的分析見 tools/sysid/analyze.py）。
 *
 * 每顆推進器的資料流：
 *   ArduPilotPlugin ──/model/<model>/joint/<joint>/cmd_pwm──▶ ThrustCurve ──/model/<model>/joint/<joint>/cmd_thrust──▶ Thruster
 *   cmd_pwm：gz.msgs.Double，PWM - 1500 [µs]，已經乘上方向的正負號（ArduPilotPlugin 的
 *            multiplier = ±800、offset = -0.5、servo 1100～1900 就是這個值，範圍 ±400）。
 *   cmd_thrust：gz.msgs.Double，推力 [N]，Thruster 外掛的預設 topic。
 *
 * SDF 參數（掛在 model 底下）：
 *   <table>       推力表 CSV：model://<模型>/<檔案>（在 GZ_SIM_RESOURCE_PATH 裡找），或絕對路徑。
 *                 # 開頭是註解；第一列是
 *                 pwm,10V,12V,…（欄名的數字是電壓）；之後每列是 PWM [µs] 與各電壓的推力 [N]。
 *   <voltage>     供電電壓 [V]，在表的電壓欄之間線性內插，超出範圍就報錯不載入。
 *   <joint_name>  推進器的 joint，可以重複。
 *
 * 正反轉取平均：f(d) = (T(1500 + d) - T(1500 - d)) / 2，所以 f(-d) = -f(d)。實機各顆推進器的
 * 高效率方向對應哪一側 PWM 不確定，而且 MOT_n_DIRECTION 正負各半，同一個動作裡兩側成對出現，
 * 取平均的誤差會抵銷（用實機紀錄驗證過，差異可以忽略）。
 */
#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace orca
{
namespace
{
std::vector<std::string> SplitCsv(const std::string &_line)
{
  std::vector<std::string> out;
  std::stringstream ss(_line);
  std::string cell;
  while (std::getline(ss, cell, ','))
    out.push_back(cell);
  return out;
}

// 分段線性內插；超出範圍取端點
double Interp(const std::vector<double> &_x, const std::vector<double> &_y, double _q)
{
  if (_q <= _x.front())
    return _y.front();
  if (_q >= _x.back())
    return _y.back();
  const auto it = std::upper_bound(_x.begin(), _x.end(), _q);
  const size_t i = static_cast<size_t>(it - _x.begin());
  const double a = (_q - _x[i - 1]) / (_x[i] - _x[i - 1]);
  return (1 - a) * _y[i - 1] + a * _y[i];
}

// model://<模型>/<檔案> 在 GZ_SIM_RESOURCE_PATH 裡找；其他照 SDF 的規則（相對路徑以 SDF 所在的資料夾為準）。
// 用 <include merge="true"> 併進別的模型時，外掛元素沒有 SDF 的檔案路徑，相對路徑會找不到，所以要寫 model://。
std::string ResolvePath(const std::string &_uri, const std::string &_sdfFile)
{
  const std::string scheme = "model://";
  if (_uri.rfind(scheme, 0) != 0)
    return gz::sim::asFullPath(_uri, _sdfFile);
  const std::string rel = _uri.substr(scheme.size());
  for (const auto &dir : gz::sim::resourcePaths())
  {
    const std::string p = dir + "/" + rel;
    if (std::ifstream(p).good())
      return p;
  }
  return _uri;
}
}  // namespace

class ThrustCurve : public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    const std::string model = gz::sim::Model(_entity).Name(_ecm);
    const std::string tag = "[orca::ThrustCurve " + model + "] ";
    auto sdf = _sdf->Clone();

    if (!sdf->HasElement("table") || !sdf->HasElement("voltage") || !sdf->HasElement("joint_name"))
    {
      gzerr << tag << "需要 <table>、<voltage> 與至少一個 <joint_name>，不載入\n";
      return;
    }
    const std::string path = ResolvePath(sdf->Get<std::string>("table"), _sdf->FilePath());
    if (!this->Load(path, sdf->Get<double>("voltage"), tag))
      return;

    for (auto e = sdf->GetElement("joint_name"); e; e = e->GetNextElement("joint_name"))
    {
      const std::string joint = e->Get<std::string>();
      const std::string base = "/model/" + model + "/joint/" + joint;
      auto pub = std::make_shared<gz::transport::Node::Publisher>(
          this->node.Advertise<gz::msgs::Double>(base + "/cmd_thrust"));
      std::function<void(const gz::msgs::Double &)> cb = [this, pub](const gz::msgs::Double &_in)
      {
        gz::msgs::Double out;
        out.set_data(this->Thrust(_in.data()));
        pub->Publish(out);
      };
      if (!this->node.Subscribe(base + "/cmd_pwm", cb))
        gzerr << tag << "訂閱 " << base << "/cmd_pwm 失敗\n";
      this->pubs.push_back(pub);
    }
    gzmsg << tag << path << " @ " << sdf->Get<double>("voltage") << " V，" << this->pubs.size()
          << " 顆；PWM 1550／1600／1700／1900 → " << this->Thrust(50) << "／" << this->Thrust(100)
          << "／" << this->Thrust(200) << "／" << this->Thrust(400) << " N\n";
  }

private:
  // PWM - 1500 [µs] → 推力 [N]（奇函數）
  double Thrust(double _d) const
  {
    const double f = Interp(this->d, this->f, std::abs(_d));
    return _d < 0 ? -f : f;
  }

  bool Load(const std::string &_path, double _volt, const std::string &_tag)
  {
    std::ifstream in(_path);
    if (!in)
    {
      gzerr << _tag << "打不開推力表 " << _path << "\n";
      return false;
    }
    std::vector<double> volts, pwm;
    std::vector<std::vector<double>> cols;
    std::string line;
    while (std::getline(in, line))
    {
      if (line.empty() || line[0] == '#')
        continue;
      const auto cells = SplitCsv(line);
      if (volts.empty())   // 欄名：pwm,10V,12V,…
      {
        for (size_t i = 1; i < cells.size(); ++i)
          volts.push_back(std::stod(cells[i]));
        cols.resize(volts.size());
        continue;
      }
      if (cells.size() != volts.size() + 1)
      {
        gzerr << _tag << "推力表欄數不對：" << line << "\n";
        return false;
      }
      pwm.push_back(std::stod(cells[0]));
      for (size_t i = 0; i < volts.size(); ++i)
        cols[i].push_back(std::stod(cells[i + 1]));
    }
    if (volts.empty() || pwm.size() < 2 || !std::is_sorted(volts.begin(), volts.end()) ||
        !std::is_sorted(pwm.begin(), pwm.end()))
    {
      gzerr << _tag << "推力表格式不對（電壓、PWM 都要遞增）：" << _path << "\n";
      return false;
    }
    if (_volt < volts.front() || _volt > volts.back())
    {
      gzerr << _tag << "<voltage> " << _volt << " 超出推力表範圍 " << volts.front() << "～"
            << volts.back() << " V\n";
      return false;
    }

    // 電壓內插
    size_t k = 1;
    while (k + 1 < volts.size() && volts[k] < _volt)
      ++k;
    const double a = volts.size() == 1 ? 0 : (_volt - volts[k - 1]) / (volts[k] - volts[k - 1]);
    std::vector<double> t(pwm.size());
    for (size_t i = 0; i < pwm.size(); ++i)
      t[i] = volts.size() == 1 ? cols[0][i] : (1 - a) * cols[k - 1][i] + a * cols[k][i];

    // 正反轉取平均，只留 d >= 0 的一半
    for (size_t i = 0; i < pwm.size(); ++i)
    {
      if (pwm[i] < 1500)
        continue;
      const double dd = pwm[i] - 1500;
      this->d.push_back(dd);
      this->f.push_back((t[i] - Interp(pwm, t, 1500 - dd)) / 2);
    }
    if (this->d.size() < 2)
    {
      gzerr << _tag << "推力表沒有 PWM >= 1500 的資料\n";
      return false;
    }
    return true;
  }

  gz::transport::Node node;
  std::vector<std::shared_ptr<gz::transport::Node::Publisher>> pubs;
  std::vector<double> d, f;
};
}  // namespace orca

GZ_ADD_PLUGIN(orca::ThrustCurve, gz::sim::System, orca::ThrustCurve::ISystemConfigure)
