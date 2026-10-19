#include "aif_debug_visualizer/manual_dashboard_overlay.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
#include <QFont>
#include <QFontDatabase>
#include <QFontMetricsF>
#include <QImage>
#include <QPainter>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <Overlay/OgreOverlayManager.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_system.hpp>
#include <rviz_rendering/render_window.hpp>

namespace aif_debug_visualizer {
namespace {

const QColor kGreen(46, 204, 113);
const QColor kYellow(241, 196, 15);
const QColor kRed(231, 76, 60);
const QColor kGray(127, 140, 141);
const QColor kText(232, 232, 232);
const QColor kBack(0, 0, 0, 97);

std::string fmt(const char * f, double v)
{
  char b[64];
  std::snprintf(b, sizeof(b), f, v);
  return b;
}

std::string fmtInt(const char * f, int a, int b)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), f, a, b);
  return buf;
}

const char * insMode(uint8_t m)
{
  static const char * names[] = {"NOT_TRACKING", "ALIGNING", "TRACKING", "NO_GPS"};
  return m < 4 ? names[m] : "?";
}

QColor chainColor(Color c)
{
  switch (c) {
    case Color::Ref: return kText;
    case Color::Good: return kGreen;
    case Color::Warn: return kYellow;
    case Color::Bad: return kRed;
    default: return kGray;
  }
}

const char * chainArrow(const StageView & v)
{
  if (v.color == Color::Off) return "\xE2\x80\x94";
  switch (v.dir) {
    case Dir::Left: return "\xE2\x97\x80";
    case Dir::Right: return "\xE2\x96\xB6";
    default: return "\xE2\x94\x80";
  }
}

const char * kDash = "\xE2\x80\x94";
const char * kStageNames[] = {"cmd", "rpm tx", "rpm rx", "\xCE\xB4 meas", "IMU \xCF\x89"};

}

ManualDashboardOverlay::ManualDashboardOverlay()
{
  config_ = new rviz_common::properties::StringProperty(
    "Decode Config", QString::fromStdString(DecodeConfig::defaultPath()),
    "Path of vehicle_decode.yaml", this, SLOT(reconnect()));
  scale_ = new rviz_common::properties::FloatProperty("Scale", 1.0f, "Overlay scale", this);
  show_speed_ = new rviz_common::properties::BoolProperty("Show Speed", true, "", this);
  show_steer_ = new rviz_common::properties::BoolProperty("Show Steering", true, "", this);
  show_drive_ = new rviz_common::properties::BoolProperty("Show Drive", true, "", this);
  show_chain_ = new rviz_common::properties::BoolProperty("Show Chain", true, "", this);
  show_status_ = new rviz_common::properties::BoolProperty("Show Status", true, "", this);
}

ManualDashboardOverlay::~ManualDashboardOverlay()
{
  if (overlay_) {
    destroyTexture();
    auto & manager = Ogre::OverlayManager::getSingleton();
    overlay_->remove2D(panel_);
    manager.destroyOverlayElement(panel_);
    manager.destroy(overlay_);
  }
}

void ManualDashboardOverlay::destroyTexture()
{
  if (material_) {
    Ogre::MaterialManager::getSingleton().remove(material_);
    material_.reset();
  }
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_);
    texture_.reset();
  }
  tex_w_ = 0;
  tex_h_ = 0;
}

void ManualDashboardOverlay::onInitialize()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  const std::string fonts =
    ament_index_cpp::get_package_share_directory("aif_debug_visualizer") + "/assets/font/Quicksand/";
  QFontDatabase::addApplicationFont(QString::fromStdString(fonts + "Quicksand-Regular.ttf"));
  QFontDatabase::addApplicationFont(QString::fromStdString(fonts + "Quicksand-Bold.ttf"));

  name_ = "aif_dashboard_" + std::to_string(reinterpret_cast<uintptr_t>(this));
  auto & manager = Ogre::OverlayManager::getSingleton();
  panel_ = static_cast<Ogre::PanelOverlayElement *>(manager.createOverlayElement("Panel", name_));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);
  panel_->setPosition(0, 0);
  overlay_ = manager.create(name_);
  overlay_->add2D(panel_);
  overlay_->setZOrder(400);
}

void ManualDashboardOverlay::reconnect()
{
  if (!isEnabled() || !node_) return;
  try {
    state_ = VehicleState::get(node_, config_->getStdString());
    setStatus(rviz_common::properties::StatusProperty::Ok, "Config", "OK");
  } catch (const std::exception & e) {
    state_.reset();
    setStatus(rviz_common::properties::StatusProperty::Error, "Config", e.what());
  }
}

void ManualDashboardOverlay::onEnable()
{
  shown_.clear();
  reconnect();
}

void ManualDashboardOverlay::onDisable()
{
  state_.reset();
  if (overlay_) overlay_->hide();
}

void ManualDashboardOverlay::ensureTexture(int w, int h)
{
  if (texture_ && tex_w_ == w && tex_h_ == h) return;
  destroyTexture();
  texture_ = Ogre::TextureManager::getSingleton().createManual(
    name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
    w, h, 0, Ogre::PF_A8R8G8B8, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  material_ = Ogre::MaterialManager::getSingleton().create(
    name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  auto * pass = material_->getTechnique(0)->getPass(0);
  pass->createTextureUnitState(name_);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthCheckEnabled(false);
  pass->setLightingEnabled(false);
  panel_->setDimensions(w, h);
  panel_->setMaterialName(name_);
  tex_w_ = w;
  tex_h_ = h;
  shown_.clear();
}

void ManualDashboardOverlay::update(float, float)
{
  if (!state_) {
    if (overlay_) overlay_->hide();
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(
    context_->getViewManager()->getRenderPanel()->getRenderWindow());
  const int w = static_cast<int>(viewport->getActualWidth());
  const int h = static_cast<int>(viewport->getActualHeight());
  if (w <= 0 || h <= 0) return;
  if (texture_ && now - last_ < std::chrono::milliseconds(100) && tex_w_ == w && tex_h_ == h) {
    return;
  }
  last_ = now;
  ensureTexture(w, h);

  const auto s = state_->snapshot(now);
  std::vector<std::string> parts;
  auto opt = [&](const std::optional<double> & v, const char * f) {
    return v ? fmt(f, *v) : std::string("-");
  };
  parts.push_back(opt(s.cmd_velocity, "%.1f"));
  parts.push_back(opt(s.cmd_steering, "%.3f"));
  parts.push_back(s.vbody_linear ? fmt("%.2f", (*s.vbody_linear)[0]) : "-");
  parts.push_back(s.caster ? fmt("%.3f", (*s.caster)[1]) : "-");
  parts.push_back(s.drive_cmd ? fmtInt("%d/%d", static_cast<int>(s.drive_cmd->left), static_cast<int>(s.drive_cmd->right)) : "-");
  parts.push_back(s.drive_meas ? fmtInt("%d/%d", static_cast<int>(s.drive_meas->left), static_cast<int>(s.drive_meas->right)) : "-");
  for (const auto & c : s.chain) {
    parts.push_back(fmtInt("%d/%d", static_cast<int>(c.dir), static_cast<int>(c.color)));
  }
  parts.push_back(s.autonomous ? std::to_string(*s.autonomous) : "-");
  parts.push_back(s.estop ? std::to_string(*s.estop) : "-");
  parts.push_back(s.ins ? fmtInt("%d/%d", s.ins->mode, s.ins->gps_fix) : "-");
  parts.push_back(s.odrive_axis_state ? std::to_string(*s.odrive_axis_state) : "-");
  parts.push_back(s.odrive_bus ? fmt("%.1f", s.odrive_bus->first) : "-");
  std::string sig;
  for (const auto & p : parts) sig += p + "|";
  sig += fmt("%.2f", scale_->getFloat());
  sig += std::to_string(show_speed_->getBool()) + std::to_string(show_steer_->getBool()) +
    std::to_string(show_drive_->getBool()) + std::to_string(show_chain_->getBool()) +
    std::to_string(show_status_->getBool());
  if (sig != shown_) {
    shown_ = sig;
    draw(s, w, h);
  }
  overlay_->show();
}

void ManualDashboardOverlay::draw(const VehicleState::Snapshot & s, int w, int h)
{
  QImage image(w, h, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  QPainter p(&image);
  p.setRenderHint(QPainter::Antialiasing, true);
  const double k = scale_->getFloat();
  const auto & cfg = state_->config();

  auto font = [&](double pt, bool bold = false) {
    QFont f("Quicksand");
    f.setPointSizeF(pt * k);
    f.setBold(bold);
    return f;
  };
  auto text = [&](const QString & t, QRectF r, double pt, const QColor & c, int align, bool bold = false) {
    p.setFont(font(pt, bold));
    p.setPen(c);
    p.drawText(r, align, t);
  };
  auto box = [&](QRectF r, double radius) {
    p.setPen(Qt::NoPen);
    p.setBrush(kBack);
    p.drawRoundedRect(r, radius * k, radius * k);
  };
  auto qs = [](const std::string & t) { return QString::fromUtf8(t.c_str()); };
  const int ac = Qt::AlignCenter;
  const int al = Qt::AlignLeft | Qt::AlignVCenter;
  const QString dash = qs(kDash);

  struct Line { std::string t; QColor c; };
  std::vector<Line> lines;
  lines.push_back(s.autonomous
    ? Line{*s.autonomous ? "AUTO" : "MANUAL", kText} : Line{"UNKNOWN", kGray});
  lines.push_back(!s.estop ? Line{std::string("E-STOP ") + kDash, kGray}
    : *s.estop ? Line{"E-STOP ACTIVE", kRed} : Line{"E-STOP clear", kGreen});
  if (s.ins) {
    const bool ok = s.ins->mode == 2 && s.ins->gps_fix;
    lines.push_back({std::string("INS ") + insMode(s.ins->mode) + " \xC2\xB7 " +
      (s.ins->gps_fix ? "fix" : "nofix"), ok ? kGreen : kYellow});
  } else {
    lines.push_back({std::string("INS ") + kDash, kGray});
  }
  if (!s.odrive_axis_state) {
    lines.push_back({std::string("ODrive ") + kDash, kGray});
  } else if (*s.odrive_axis_state == 8) {
    lines.push_back({std::string("ODrive CLOSED_LOOP") +
      (s.odrive_bus ? fmt(" %.1f V", s.odrive_bus->first) : std::string()), kGreen});
  } else {
    lines.push_back({"ODrive state " + std::to_string(*s.odrive_axis_state), kYellow});
  }
  const QFontMetricsF status_fm(font(12, true));
  double status_w = 0;
  for (const auto & l : lines) status_w = std::max(status_w, status_fm.horizontalAdvance(qs(l.t)));
  status_w = std::max(status_w + 24 * k, 160 * k);
  const double drive_w = 150 * k + QFontMetricsF(font(11)).horizontalAdvance("-0000 \xE2\x94\x82 -0000") + 14 * k;
  const double chain_w = 84 * k * 5 + 20 * k;
  const double gap = 10 * k;
  const double side = std::max(show_drive_->getBool() ? drive_w : 0.0, show_status_->getBool() ? status_w : 0.0);
  const bool chain_above = w / 2.0 - chain_w / 2.0 < 14 * k + side + gap;

  if (s.estop && *s.estop) {
    p.fillRect(QRectF(0, 0, w, h), QColor(231, 76, 60, 46));
    const QRectF r(w / 2.0 - 150 * k, h / 2.0 - 45 * k, 300 * k, 90 * k);
    p.setPen(Qt::NoPen);
    p.setBrush(kRed);
    p.drawRoundedRect(r, 10 * k, 10 * k);
    text("E-STOP", r, 44, Qt::white, ac, true);
  }

  if (show_speed_->getBool()) {
    const QRectF r(w / 2.0 - 110 * k, 10 * k, 220 * k, 110 * k);
    box(r, 55);
    const QColor c = s.cmd_velocity ? kText : kGray;
    text(s.cmd_velocity ? qs(fmt("%.1f", *s.cmd_velocity)) : dash,
      QRectF(r.x(), r.y() + 4 * k, r.width(), 52 * k), 38, c, ac, true);
    text("m/s CMD", QRectF(r.x(), r.y() + 54 * k, r.width(), 16 * k), 10, kGray, ac);
    const QColor ic = s.vbody_linear ? kText : kGray;
    text(s.vbody_linear ? qs(fmt("%.2f", (*s.vbody_linear)[0])) : dash,
      QRectF(r.x(), r.y() + 72 * k, r.width() / 2 + 10 * k, 30 * k), 22, ic, Qt::AlignRight | Qt::AlignVCenter);
    text("IMU", QRectF(r.center().x() + 18 * k, r.y() + 72 * k, 60 * k, 30 * k), 10, kGray, al);
  }

  if (show_steer_->getBool()) {
    const double deg = 180.0 / M_PI;
    const bool close = s.cmd_steering && s.caster &&
      std::abs((*s.cmd_steering - (*s.caster)[1]) * deg) <= 3.0;
    const struct { double x; const char * title; std::optional<double> v; QColor c; } items[] = {
      {0.14, "\xCE\xB4 CMD", s.cmd_steering, kText},
      {0.86, "\xCE\xB4 MEAS",
        s.caster ? std::optional<double>((*s.caster)[1]) : std::nullopt, close ? kGreen : kText},
    };
    for (const auto & it : items) {
      const QRectF r(w * it.x - 70 * k, 10 * k, 140 * k, 70 * k);
      box(r, 10);
      text(qs(it.title), QRectF(r.x(), r.y() + 4 * k, r.width(), 20 * k), 10, kGray, ac);
      text(it.v ? qs(fmt("%+.1f\xC2\xB0", *it.v * deg)) : dash,
        QRectF(r.x(), r.y() + 26 * k, r.width(), 38 * k), 22, it.v ? it.c : kGray, ac, true);
    }
  }

  if (show_drive_->getBool()) {
    const QRectF r(14 * k, h - 120 * k, drive_w, 106 * k);
    box(r, 10);
    text("DRIVE rpm  target \xE2\x94\x82 actual", QRectF(r.x() + 10 * k, r.y() + 6 * k, r.width(), 20 * k), 10, kGray, al);
    const double full = cfg.linear_max_vel / (2.0 * M_PI * cfg.wheel_radius / 60.0);
    for (int i = 0; i < 2; ++i) {
      const double y = r.y() + 36 * k + i * 32 * k;
      text(i == 0 ? "L" : "R", QRectF(r.x() + 10 * k, y - 8 * k, 16 * k, 24 * k), 12, kText, ac, true);
      const QRectF bar(r.x() + 30 * k, y, 110 * k, 8 * k);
      p.setPen(Qt::NoPen);
      p.setBrush(QColor(255, 255, 255, 40));
      p.drawRect(bar);
      auto frac = [&](double v) { return std::clamp(std::abs(v) / full, 0.0, 1.0); };
      auto pick = [&](const std::optional<DriveRpm> & d) {
        return d ? std::optional<double>(i == 0 ? d->left : d->right) : std::nullopt;
      };
      const auto meas = pick(s.drive_meas);
      const auto cmd = pick(s.drive_cmd);
      if (meas) {
        p.setBrush(kGreen);
        p.drawRect(QRectF(bar.x(), bar.y(), bar.width() * frac(*meas), bar.height()));
      }
      if (cmd) {
        p.setPen(QPen(Qt::white, std::max(1.0, 2 * k)));
        const double x = bar.x() + bar.width() * frac(*cmd);
        p.drawLine(QPointF(x, bar.y() - 3 * k), QPointF(x, bar.bottom() + 3 * k));
      }
      const std::string nums = cmd && meas
        ? fmtInt("%d \xE2\x94\x82 %d", static_cast<int>(std::lround(*cmd)), static_cast<int>(std::lround(*meas)))
        : std::string(kDash);
      text(qs(nums), QRectF(bar.right() + 8 * k, y - 8 * k, 130 * k, 24 * k), 11,
        cmd && meas ? kText : kGray, al);
    }
  }

  if (show_chain_->getBool()) {
    const double cw = 84 * k;
    const QRectF r(w / 2.0 - chain_w / 2.0, h - 120 * k - (chain_above ? 116 * k : 0), chain_w, 106 * k);
    box(r, 10);
    text("TURN CHAIN", QRectF(r.x(), r.y() + 4 * k, r.width(), 20 * k), 10, kGray, ac);
    for (int i = 0; i < 5; ++i) {
      const auto & v = s.chain[i];
      const QRectF cell(r.x() + 10 * k + i * cw, r.y() + 28 * k, cw, 72 * k);
      const QColor c = chainColor(v.color);
      if (v.color == Color::Bad) {
        p.setPen(QPen(kRed, std::max(1.0, 2 * k)));
        p.setBrush(Qt::NoBrush);
        p.drawRoundedRect(cell.adjusted(4 * k, 0, -4 * k, 0), 6 * k, 6 * k);
      }
      text(qs(chainArrow(v)), QRectF(cell.x(), cell.y() + 2 * k, cell.width(), 36 * k), 22, c, ac, true);
      text(qs(kStageNames[i]), QRectF(cell.x(), cell.y() + 38 * k, cell.width(), 16 * k), 9, kText, ac);
      if (v.color == Color::NoData) {
        text("no data", QRectF(cell.x(), cell.y() + 54 * k, cell.width(), 14 * k), 8, kGray, ac);
      }
    }
  }

  if (show_status_->getBool()) {
    const QRectF r(w - 14 * k - status_w, h - 120 * k, status_w, 106 * k);
    box(r, 10);
    for (size_t i = 0; i < lines.size(); ++i) {
      text(qs(lines[i].t), QRectF(r.x() + 12 * k, r.y() + 6 * k + i * 24 * k, r.width() - 24 * k, 24 * k),
        12, lines[i].c, al, true);
    }
  }

  p.end();
  texture_->getBuffer()->blitFromMemory(
    Ogre::PixelBox(w, h, 1, Ogre::PF_A8R8G8B8, image.bits()));
}

}

PLUGINLIB_EXPORT_CLASS(aif_debug_visualizer::ManualDashboardOverlay, rviz_common::Display)
