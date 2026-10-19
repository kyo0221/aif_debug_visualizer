#include "aif_debug_visualizer/manual_dashboard_panel.hpp"

#include <cmath>
#include <cstdio>
#include <vector>
#include <QFontDatabase>
#include <QGridLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace aif_debug_visualizer {
namespace {

const char * kGreen = "#2ecc71";
const char * kYellow = "#f1c40f";
const char * kRed = "#e74c3c";
const char * kGray = "#7f8c8d";
const char * kText = "#e8e8e8";

const std::vector<std::pair<std::string, std::string>> kRates = {
  {"/cmd_vel", "cmd_vel"}, {"/joy", "joy"}, {"0x210 tx", "drive_cmd"}, {"0x711 rx", "drive_meas"},
  {"/caster_data", "caster_data"}, {"0x012", "caster_angle"}, {"velocity_body", "velocity_body"},
  {"ins_status", "ins_status"}};

const std::vector<std::string> kRaw = {
  "0x012 raw", "0x013 rot", "vx / vy", "\xCF\x89 raw (NED)", "INS flags", "ODrive", "joy",
  "rpm\xE2\x86\x92m/s L/R"};

template <typename... A>
std::string fmt(const char * f, A... a)
{
  char b[160];
  std::snprintf(b, sizeof(b), f, a...);
  return b;
}

}  // namespace

ManualDashboardPanel::ManualDashboardPanel(QWidget * parent)
: rviz_common::Panel(parent), timer_(new QTimer(this))
{
  auto * grid = new QGridLayout(this);
  const QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  auto row = [&](int r, const std::string & name, const std::string & key) {
    auto * n = new QLabel(QString::fromStdString(name));
    auto * v = new QLabel("\xE2\x80\x94");
    n->setFont(mono);
    v->setFont(mono);
    grid->addWidget(n, r, 0);
    grid->addWidget(v, r, 1);
    values_[key] = v;
  };
  int r = 1;
  for (const auto & [name, key] : kRates) row(r++, name, key);
  for (const auto & name : kRaw) row(r++, name, name);
  grid->setRowStretch(r, 1);
  grid->setColumnStretch(1, 1);

  auto * err = new QLabel();
  err->setFont(mono);
  err->setStyleSheet(QString("color: %1").arg(kRed));
  grid->addWidget(err, 0, 0, 1, 2);
  values_["error"] = err;

  connect(timer_, &QTimer::timeout, this, [this]() { refresh(); });
}

void ManualDashboardPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  timer_->start(200);
}

void ManualDashboardPanel::set(const std::string & key, const std::string & text, const char * color)
{
  auto * l = values_.at(key);
  l->setText(QString::fromStdString(text));
  l->setStyleSheet(QString("color: %1").arg(color));
}

void ManualDashboardPanel::refresh()
{
  values_.at("error")->clear();
  state_ = VehicleState::latest();
  if (!state_ && node_) {
    try {
      state_ = VehicleState::get(node_, DecodeConfig::defaultPath());
    } catch (const std::exception & e) {
      values_.at("error")->setText(e.what());
    }
  }
  if (!state_) return;

  const auto s = state_->snapshot(VehicleState::Clock::now());
  const auto & cfg = state_->config();
  const std::string none = "\xE2\x80\x94";

  for (const auto & [name, key] : kRates) {
    auto it = s.freshness.find(key);
    if (it == s.freshness.end() || it->second.second == Freshness::Level::Stale) {
      set(key, none, kGray);
    } else {
      set(key, fmt("%.0f Hz", it->second.first),
        it->second.second == Freshness::Level::Good ? kGreen : kYellow);
    }
  }

  auto put = [&](const std::string & key, bool ok, const std::string & text) {
    if (ok) set(key, text, kText);
    else set(key, none, kGray);
  };

  put("0x012 raw", s.caster_angle_raw && s.caster_angle,
    s.caster_angle_raw && s.caster_angle ?
    fmt("%d  %+.2f\xC2\xB0", static_cast<int>(*s.caster_angle_raw), *s.caster_angle * 180.0 / M_PI) : "");
  put("0x013 rot", s.caster_rot.has_value(), s.caster_rot ? fmt("%.3f rad", *s.caster_rot) : "");
  put("vx / vy", s.vbody_linear.has_value(),
    s.vbody_linear ? fmt("%.2f / %.2f", (*s.vbody_linear)[0], (*s.vbody_linear)[1]) : "");
  put("\xCF\x89 raw (NED)", s.yaw_rate_raw.has_value(),
    s.yaw_rate_raw ? fmt("%+.3f", *s.yaw_rate_raw) : "");
  put("INS flags", s.ins.has_value(),
    s.ins ? fmt("mode=%d fix=%d imu_err=%d gps_err=%d time_err=%d mag_err=%d",
      static_cast<int>(s.ins->mode), static_cast<int>(s.ins->gps_fix),
      static_cast<int>(s.ins->imu_error), static_cast<int>(s.ins->gps_error),
      static_cast<int>(s.ins->time_error), static_cast<int>(s.ins->mag_pres_error)) : "");
  const bool od = s.odrive_axis_state && s.odrive_encoder && s.odrive_bus;
  put("ODrive", od,
    od ? fmt("state=%d pos=%.2f vbus=%.1f ibus=%.2f", static_cast<int>(*s.odrive_axis_state),
      s.odrive_encoder->first, s.odrive_bus->first, s.odrive_bus->second) : "");
  put("joy", s.joy.has_value(), s.joy ? fmt("%.2f / %.2f", (*s.joy)[0], (*s.joy)[1]) : "");
  put("rpm\xE2\x86\x92m/s L/R", s.drive_meas.has_value(),
    s.drive_meas ? fmt("%.2f / %.2f", rpmToMps(s.drive_meas->left, cfg.wheel_radius),
      rpmToMps(s.drive_meas->right, cfg.wheel_radius)) : "");
}

}  // namespace aif_debug_visualizer

PLUGINLIB_EXPORT_CLASS(aif_debug_visualizer::ManualDashboardPanel, rviz_common::Panel)
