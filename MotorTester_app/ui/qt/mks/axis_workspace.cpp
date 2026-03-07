#include "mks/axis_workspace.h"

#include "mks/ScopeWidget.h"
#include "mks/axis_manager.h"

#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QAbstractItemView>
#include <QLabel>
#include <QDateTime>
#include <QMessageBox>
#include <QEvent>
#include <QMetaObject>
#include <QMetaType>
#include <QPushButton>
#include <QStringList>
#include <QThread>
#include <QSpinBox>
#include <QStyledItemDelegate>
#include <QTabWidget>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QSplitter>
#include <QTextEdit>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QTextStream>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QSlider>
#include <QLineEdit>
#include <QTimer>
#include <QRadioButton>
#include <QAbstractSpinBox>
#include <cmath>
#include "motion_core/parameter_id.h"
#include "mks/sequencer_widget.h"

namespace {

constexpr int kNameColumn = 0;
constexpr int kGroupColumn = 1;
constexpr int kUnitColumn = 2;
constexpr int kReadOnlyColumn = 3;
constexpr int kCurrentValueColumn = 4;
constexpr int kNewValueColumn = 5;

class NewValueColumnDelegate final : public QStyledItemDelegate {
public:
    explicit NewValueColumnDelegate(QObject* parent = nullptr)
        : QStyledItemDelegate(parent) {}

    QWidget* createEditor(QWidget* parent,
                          const QStyleOptionViewItem& option,
                          const QModelIndex& index) const override {
        if (index.column() != kNewValueColumn) {
            return nullptr;
        }

        const QString read_only = index.sibling(index.row(), kReadOnlyColumn).data(Qt::DisplayRole).toString();
        if (read_only.compare("Yes", Qt::CaseInsensitive) == 0 ||
            read_only.compare("N/A", Qt::CaseInsensitive) == 0) {
            return nullptr;
        }

        return QStyledItemDelegate::createEditor(parent, option, index);
    }
};

class WheelEventFilter : public QObject {
public:
    explicit WheelEventFilter(QObject* parent = nullptr) : QObject(parent) {}

protected:
    bool eventFilter(QObject* obj, QEvent* event) override {
        if (event->type() == QEvent::Wheel) {
            auto* widget = qobject_cast<QWidget*>(obj);
            if (widget && !widget->hasFocus()) {
                return true; // Ignore wheel event if not focused
            }
            // If focused, some users prefer it to still not scroll, but let's just completely disable wheel changes on spinboxes.
            if (qobject_cast<QAbstractSpinBox*>(obj)) {
                return true; // Ignore completely
            }
        }
        return QObject::eventFilter(obj, event);
    }
};

QString protocolDetailsForParameter(const int domain, const int value) {
    using motion_core::CommonParameter;
    using motion_core::MksParameter;
    using motion_core::ParameterDomain;

    if (domain == static_cast<int>(motion_core::ParameterDomain::Common)) {
        if (static_cast<motion_core::CommonParameter>(value) == motion_core::CommonParameter::HardwareGearRatio) {
            return QStringLiteral(
                "<b>Gear Ratio (Передаточное число редуктора)</b><br>"
                "<b>Use:</b> Number of motor turns per single output turn.<br>"
                "<b>Note:</b> 1.0 = direct drive. Имплементация для пересчета шагов в градусы и обратно.");
        }
        if (static_cast<motion_core::CommonParameter>(value) ==
            motion_core::CommonParameter::HardwareEncoderResolutionBits) {
            return QStringLiteral(
                "<b>Encoder Resolution (bits)</b><br>"
                "<b>Use:</b> Software-only encoder CPR setting for ticks↔degrees conversion.<br>"
                "<b>Formula:</b> ticks_per_degree = (2^bits * gear_ratio) / 360.<br>"
                "<b>Note:</b> Не читается с драйвера, меняется только явным patch из UI/API.");
        }
        return {};
    }

    if (domain != static_cast<int>(motion_core::ParameterDomain::Mks)) {
        return {};
    }

    switch (static_cast<MksParameter>(value)) {
        case MksParameter::WorkMode:
            return QStringLiteral(
                "<b>Command:</b> 0x82 SetWorkMode<br>"
                "<b>Values:</b><br>"
                "1 = CR_CLOSE / SR_CLOSE (Замкнутый контур, 1500 RPM)<br>"
                "2 = CR_vFOC / SR_vFOC (Векторное управление, 3000 RPM)<br>"
                "0 = CR_OPEN / SR_OPEN (Без энкодера, 400 RPM)<br>"
                "<b>Note:</b> Требуется SR_* для управления по шине CAN.");
        case MksParameter::WorkingCurrentMilliAmp:
            return QStringLiteral(
                "<b>Command:</b> 0x83 SetWorkingCurrent<br>"
                "<b>Range:</b> 0..5200 mA.<br>"
                "<b>Note:</b> Working current strongly affects torque and heating.");
        case MksParameter::Subdivision:
            return QStringLiteral(
                "<b>Command:</b> 0x84 SetSubdivision<br>"
                "<b>Range:</b> 1..255.<br>"
                "<b>Note:</b> Микрошаг. Чем больше значение, тем выше разрешение.");
        case MksParameter::EnPinActiveLevel:
            return QStringLiteral(
                "<b>Command:</b> 0x85 SetEnPinActiveLevel<br>"
                "<b>Values:</b> 0=Active Low, 1=Active High, 2=Always Enabled.");
        case MksParameter::MotorDirection:
            return QStringLiteral(
                "<b>Command:</b> 0x86 SetMotorDirection<br>"
                "<b>Values:</b> 0=Normal, 1=Reverse.");
        case MksParameter::AutoScreenOff:
            return QStringLiteral(
                "<b>Command:</b> 0x87 SetAutoTurnOffScreen<br>"
                "<b>Values:</b> 0=Off, 1=On.");
        case MksParameter::LockedRotorProtection:
            return QStringLiteral(
                "<b>Command:</b> 0x88 SetLockedRotorProtection<br>"
                "<b>Values:</b> 0=Disable, 1=Enable.<br>"
                "<b>Note:</b> Вызывает ошибку 10 (Wrong Protect) при блокировке вала.");
        case MksParameter::SubdivisionInterpolation:
            return QStringLiteral(
                "<b>Command:</b> 0x89 SetSubdivisionInterpolation<br>"
                "<b>Values:</b> 0=Disable, 1=Enable.");
        case MksParameter::CanBitrateIndex:
            return QStringLiteral(
                "<b>Command:</b> 0x8A SetCanBitrate (read-only here)<br>"
                "<b>Values:</b> 0=125k, 1=250k, 2=500k, 3=1M.");
        case MksParameter::CanId:
            return QStringLiteral(
                "<b>Command:</b> 0x8B SetCanId (read-only here)<br>"
                "<b>Range:</b> 1..2047.<br>"
                "<b>Note:</b> Меняется только через Provisioning.");
        case MksParameter::SlaveRespondMode:
            return QStringLiteral(
                "<b>Command:</b> 0x8C SetSlaveRespondActive (Respond flag)<br>"
                "<b>Safe value:</b> 1 (Обязательно для работы HexaKinetica).<br>"
                "<b>Note:</b> Флаг разрешения ответов мотора на команды.");
        case MksParameter::SlaveActiveReport:
            return QStringLiteral(
                "<b>Command:</b> 0x8C SetSlaveRespondActive (Active flag)<br>"
                "<b>Values:</b><br>"
                "0 = Пассивный ответ (только подтверждение приема)<br>"
                "1 = Активный ответ (подтверждение + уведомление о завершении движения).");
        case MksParameter::GroupId:
            return QStringLiteral(
                "<b>Command:</b> 0x8D SetGroupId<br>"
                "<b>Range:</b> 1..2047.<br>"
                "<b>Use:</b> Group addressing on CAN bus.");
        case MksParameter::KeyLock:
            return QStringLiteral(
                "<b>Command:</b> 0x8F SetKeyLock<br>"
                "<b>Values:</b> 0=Unlock keys, 1=Lock keys.");
        case MksParameter::HoldingCurrentIndex:
            return QStringLiteral(
                "<b>Command:</b> 0x9B SetHoldingCurrent<br>"
                "<b>Range:</b> 0..8 (от 10% до 90% рабочего тока).");
        case MksParameter::LimitPortRemap:
            return QStringLiteral(
                "<b>Command:</b> 0x9E SetLimitPortRemap<br>"
                "<b>Values:</b> 0=Disable, 1=Enable.");
        case MksParameter::AxisPositionRaw:
            return QStringLiteral(
                "<b>Source:</b> 0x31 ReadEncoderAddition.<br>"
                "<b>Type:</b> Read-only telemetry.<br>"
                "<b>Note:</b> Абсолютное значение энкодера.");
        case MksParameter::MotorSpeedRpm:
            return QStringLiteral(
                "<b>Source:</b> 0x32 ReadMotorSpeed.<br>"
                "<b>Type:</b> Read-only telemetry.<br>"
                "<b>Note:</b> Текущая скорость в RPM.");
        case MksParameter::ProtectionState:
            return QStringLiteral(
                "<b>Source:</b> 0x3E ReadProtectionState.<br>"
                "<b>Type:</b> Read-only telemetry.<br>"
                "<b>Note:</b> Состояние внутренних защит (0=Ок, иначе ошибка).");
        case MksParameter::MotorStatus:
            return QStringLiteral(
                "<b>Source:</b> 0xF1 QueryMotorStatus.<br>"
                "<b>Type:</b> Read-only telemetry.<br>"
                "<b>Values:</b> 0=Ошибка, 1=Остановлен, 2=Ускорение, 3=Замедление, 4=Макс. скорость, 5=Homing, 6=Калибровка.");
        case MksParameter::EnableMotor:
            return QStringLiteral(
                "<b>Command:</b> 0xF3 EnableMotor<br>"
                "<b>Values:</b> false/0 = Выключить (Освободить вал), true/1 = Включить (Удержание вала).<br>"
                "<b>Behavior:</b> Подтверждается опросом 0xF1.");
        default:
            break;
    }

    return {};
}

} // namespace

AxisWorkspace::AxisWorkspace(int axis_id, mks::AxisManager* manager, QWidget* parent)
    : QWidget(parent), axis_id_(axis_id), manager_(manager) {
    setupUi();

    connect(manager_, SIGNAL(telemetryUpdated(int,QVariantMap)),
            this, SLOT(onTelemetryUpdated(int,QVariantMap)));
            
    // Parameter Async Signals
    connect(manager_, SIGNAL(parameterListReady(int,QVariantList)),
            this, SLOT(onParameterListReady(int,QVariantList)));
    connect(manager_, SIGNAL(parametersRead(int,QVariantList)),
            this, SLOT(onParametersRead(int,QVariantList)));

    queue_fill_timer_ = new QTimer(this);
    connect(queue_fill_timer_, &QTimer::timeout, this, &AxisWorkspace::fillTrajectoryQueue);
    queue_fill_timer_->start(100); // Check every 100ms

    trajectory_thread_active_ = true;
    trajectory_thread_ = std::thread(&AxisWorkspace::trajectoryLoop, this);

    scheduleWatchAxis(true);
}

AxisWorkspace::~AxisWorkspace() {
    trajectory_thread_active_ = false;
    if (trajectory_thread_.joinable()) {
        trajectory_thread_.join();
    }
    scheduleWatchAxis(false);
}

void AxisWorkspace::scheduleWatchAxis(const bool enabled) {
    auto* manager = manager_.data();
    if (!manager) {
        return;
    }

    const auto ok = QMetaObject::invokeMethod(
        manager,
        [manager, axis_id = axis_id_, enabled]() {
            manager->watchAxis(axis_id, enabled);
        },
        Qt::QueuedConnection);

    if (!ok) {
        qWarning("AxisWorkspace: failed to schedule watchAxis(%d, %d)", axis_id_, enabled ? 1 : 0);
    }
}

void AxisWorkspace::setupUi() {
    auto* root = new QVBoxLayout(this);
    tabs_ = new QTabWidget(this);
    root->addWidget(tabs_);

    setupControlTab();
    setupConfigTab();
    setupCommandsTab();
    
    sequencer_ = new mks::SequencerWidget(this);
    tabs_->addTab(sequencer_, "Sequencer (Trapezoidal)");
}

void AxisWorkspace::setupControlTab() {
    auto* tab = new QWidget(this);
    auto* layout = new QVBoxLayout(tab);

    auto* filter = new WheelEventFilter(this);

    // Top actions row (requested: controls on top)
    auto* actions_box = new QGroupBox("Actions", tab);
    auto* actions_layout = new QHBoxLayout(actions_box);

    auto* btn_enable = new QPushButton("ENABLE", actions_box);
    auto* btn_disable = new QPushButton("DISABLE", actions_box);
    auto* btn_clear_err = new QPushButton("CLEAR ERRORS", actions_box);
    auto* btn_set_zero = new QPushButton("SET ZERO", actions_box);
    auto* btn_home = new QPushButton("GO HOME", actions_box);
    auto* btn_estop = new QPushButton("E-STOP", actions_box);

    btn_enable->setStyleSheet("background-color: #2ea043; color: white; font-weight: bold;");
    btn_disable->setStyleSheet("background-color: #da3633; color: white; font-weight: bold;");
    btn_clear_err->setStyleSheet("background-color: #d29922; color: black;");
    btn_estop->setStyleSheet("background-color: #da3633; color: white; font-weight: bold; padding: 8px;");

    actions_layout->addWidget(btn_enable);
    actions_layout->addWidget(btn_disable);
    actions_layout->addWidget(btn_clear_err);
    actions_layout->addWidget(btn_set_zero);
    actions_layout->addWidget(btn_home);
    actions_layout->addSpacing(12);
    actions_layout->addWidget(btn_estop);
    actions_layout->addStretch();

    layout->addWidget(actions_box);

    // Middle split: Basic Motion (left) + Telemetry one-column (right)
    auto* top_row = new QHBoxLayout();

    auto* motion_grp = new QGroupBox("Basic Motion", tab);
    auto* motion_form = new QFormLayout(motion_grp);

    mode_combo_ = new QComboBox(motion_grp);
    mode_combo_->addItem("Position (Generic/PP)", 1);
    mode_combo_->addItem("CSP (EtherCAT mode 8)", 8);
    mode_combo_->addItem("CSV (EtherCAT mode 9)", 9);
    mode_combo_->addItem("Homing (EtherCAT mode 6)", 6);
    mode_combo_->setFixedWidth(160);
    motion_form->addRow("Op Mode:", mode_combo_);

    speed_spin_ = new QSpinBox(motion_grp);
    speed_spin_->setRange(0, 3000);
    speed_spin_->setValue(400);
    speed_spin_->setSuffix(" RPM");
    speed_spin_->setFixedWidth(160);
    speed_spin_->installEventFilter(filter);
    motion_form->addRow("Speed:", speed_spin_);

    accel_spin_ = new QSpinBox(motion_grp);
    accel_spin_->setRange(0, 100);
    accel_spin_->setValue(50);
    accel_spin_->setSuffix(" %");
    accel_spin_->setFixedWidth(160);
    accel_spin_->installEventFilter(filter);
    motion_form->addRow("Accel:", accel_spin_);

    target_pos_spin_ = new QDoubleSpinBox(motion_grp);
    target_pos_spin_->setRange(-1e6, 1e6);
    target_pos_spin_->setDecimals(2);
    target_pos_spin_->setValue(0.0);
    target_pos_spin_->setFixedWidth(160);
    target_pos_spin_->setAlignment(Qt::AlignRight);
    target_pos_spin_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    target_pos_spin_->setKeyboardTracking(false);
    target_pos_spin_->installEventFilter(filter);
    motion_form->addRow("Target Position:", target_pos_spin_);

    target_slider_ = new QSlider(Qt::Horizontal, motion_grp);
    target_slider_->setRange(-100000000, 100000000); // [-1e6; 1e6] deg with x100 scale
    target_slider_->setSingleStep(10);
    target_slider_->setPageStep(1000);
    motion_form->addRow("Live Target:", target_slider_);

    auto* type_row = new QWidget(motion_grp);
    auto* type_layout = new QHBoxLayout(type_row);
    type_layout->setContentsMargins(0, 0, 0, 0);
    radio_move_abs_ = new QRadioButton("Abs", type_row);
    radio_move_rel_ = new QRadioButton("Rel", type_row);
    radio_move_abs_->setChecked(true);
    type_layout->addWidget(radio_move_abs_);
    type_layout->addWidget(radio_move_rel_);
    type_layout->addStretch();
    motion_form->addRow("Move Type:", type_row);

    auto* btn_move = new QPushButton("Move", motion_grp);
    motion_form->addRow(QString(), btn_move);

    chk_sine_enable_ = new QCheckBox("Enable Sine Mode", motion_grp);
    motion_form->addRow(QString(), chk_sine_enable_);

    spin_sine_amp_ = new QDoubleSpinBox(motion_grp);
    spin_sine_amp_->setRange(0.1, 360000.0);
    spin_sine_amp_->setValue(20.0);
    spin_sine_amp_->setSuffix(" deg");
    spin_sine_amp_->setFixedWidth(160);
    spin_sine_amp_->installEventFilter(filter);
    motion_form->addRow("Sine Amplitude:", spin_sine_amp_);

    spin_sine_freq_ = new QDoubleSpinBox(motion_grp);
    spin_sine_freq_->setRange(0.01, 50.0);
    spin_sine_freq_->setDecimals(3);
    spin_sine_freq_->setValue(0.5);
    spin_sine_freq_->setSuffix(" Hz");
    spin_sine_freq_->setFixedWidth(160);
    spin_sine_freq_->installEventFilter(filter);
    motion_form->addRow("Sine Frequency:", spin_sine_freq_);

    jog_step_spin_ = new QDoubleSpinBox(motion_grp);
    jog_step_spin_->setRange(0.01, 360.0);
    jog_step_spin_->setValue(1.0);
    jog_step_spin_->setSuffix("°");
    auto* jog_row = new QWidget(motion_grp);
    auto* jog_layout = new QHBoxLayout(jog_row);
    jog_layout->setContentsMargins(0, 0, 0, 0);
    btn_jog_neg_ = new QPushButton("Jog -", jog_row);
    btn_jog_pos_ = new QPushButton("Jog +", jog_row);
    jog_layout->addWidget(btn_jog_neg_);
    jog_layout->addWidget(btn_jog_pos_);
    jog_layout->addStretch();
    motion_form->addRow("Jog step:", jog_step_spin_);
    motion_form->addRow(QString(), jog_row);

    top_row->addWidget(motion_grp, 3);

    auto* status = new QGroupBox("Telemetry (Live)", tab);
    auto* status_form = new QFormLayout(status);
    lbl_sys_state_ = new QLabel("N/A", status);
    lbl_state_ = new QLabel("N/A", status);
    lbl_protection_ = new QLabel("N/A", status);
    lbl_axis_ = new QLabel("N/A", status);
    lbl_target_ = new QLabel("N/A", status);
    lbl_speed_ = new QLabel("N/A", status);
    lbl_torque_ = new QLabel("N/A", status);

    status_form->addRow("System State:", lbl_sys_state_);
    status_form->addRow("Motor Status:", lbl_state_);
    status_form->addRow("Protection:", lbl_protection_);
    status_form->addRow("Actual Pos:", lbl_axis_);
    status_form->addRow("Target Pos:", lbl_target_);
    status_form->addRow("Speed:", lbl_speed_);
    status_form->addRow("Torque:", lbl_torque_);

    top_row->addWidget(status, 2);
    layout->addLayout(top_row, 1);

    auto* scope_group = new QGroupBox("Real-time Monitor", tab);
    auto* scope_layout = new QVBoxLayout(scope_group);

    auto* scope_ctrl = new QHBoxLayout();
    scope_ctrl->addWidget(new QLabel("Signal:"));
    cmb_scope_signal_ = new QComboBox(scope_group);
    cmb_scope_signal_->addItems({"Position (deg)", "Velocity (deg/s)", "Torque (%)"});
    scope_ctrl->addWidget(cmb_scope_signal_);

    scope_ctrl->addWidget(new QLabel("Time Window:"));
    sld_scope_time_ = new QSlider(Qt::Horizontal, scope_group);
    sld_scope_time_->setRange(2, 60);
    sld_scope_time_->setValue(10);
    sld_scope_time_->setFixedWidth(140);
    lbl_scope_time_ = new QLabel("10 s", scope_group);
    scope_ctrl->addWidget(sld_scope_time_);
    scope_ctrl->addWidget(lbl_scope_time_);

    chk_plot_actual_pos_ = new QCheckBox("Actual", scope_group);
    chk_plot_actual_pos_->setChecked(true);
    chk_plot_target_pos_ = new QCheckBox("Target", scope_group);
    chk_plot_target_pos_->setChecked(true);
    chk_plot_actual_vel_ = new QCheckBox("Actual Vel", scope_group);
    chk_plot_actual_vel_->setChecked(true);
    chk_plot_target_vel_ = new QCheckBox("Target Vel", scope_group);
    chk_plot_target_vel_->setChecked(true);
    chk_plot_pos_error_ = new QCheckBox("Pos Error", scope_group);
    chk_plot_pos_error_->setChecked(true);

    scope_ctrl->addWidget(chk_plot_actual_pos_);
    scope_ctrl->addWidget(chk_plot_target_pos_);
    scope_ctrl->addWidget(chk_plot_actual_vel_);
    scope_ctrl->addWidget(chk_plot_target_vel_);
    scope_ctrl->addWidget(chk_plot_pos_error_);

    chk_auto_scale_ = new QCheckBox("Auto Scale", scope_group);
    chk_auto_scale_->setChecked(true);
    scope_ctrl->addWidget(chk_auto_scale_);
    scope_ctrl->addStretch();

    scope_ = new RDT::ScopeWidget(scope_group);
    scope_->setMinimumHeight(240);
    scope_->setXRange(10.0);
    scope_->setAutoRange(true);
    scope_->addChannel("actual", QColor(80, 200, 255));
    scope_->addChannel("target", QColor(255, 80, 80));
    scope_->addChannel("speed", QColor(255, 180, 80));
    scope_->addChannel("target_speed", QColor(180, 255, 120));
    scope_->addChannel("pos_error", QColor(255, 120, 255));
    scope_->addChannel("torque", QColor(180, 140, 255));

    scope_->setChannelVisibility("actual", chk_plot_actual_pos_->isChecked());
    scope_->setChannelVisibility("target", chk_plot_target_pos_->isChecked());
    scope_->setChannelVisibility("speed", chk_plot_actual_vel_->isChecked());
    scope_->setChannelVisibility("target_speed", chk_plot_target_vel_->isChecked());
    scope_->setChannelVisibility("pos_error", chk_plot_pos_error_->isChecked());

    connect(chk_plot_actual_pos_, &QCheckBox::toggled, this, [this](bool checked) {
        if (scope_) scope_->setChannelVisibility("actual", checked);
    });
    connect(chk_plot_target_pos_, &QCheckBox::toggled, this, [this](bool checked) {
        if (scope_) scope_->setChannelVisibility("target", checked);
    });
    connect(chk_plot_actual_vel_, &QCheckBox::toggled, this, [this](bool checked) {
        if (scope_) scope_->setChannelVisibility("speed", checked);
    });
    connect(chk_plot_target_vel_, &QCheckBox::toggled, this, [this](bool checked) {
        if (scope_) scope_->setChannelVisibility("target_speed", checked);
    });
    connect(chk_plot_pos_error_, &QCheckBox::toggled, this, [this](bool checked) {
        if (scope_) scope_->setChannelVisibility("pos_error", checked);
    });
    connect(chk_auto_scale_, &QCheckBox::toggled, this, [this](bool checked) {
        if (scope_) scope_->setAutoRange(checked);
    });
    connect(sld_scope_time_, &QSlider::valueChanged, this, [this](int sec) {
        if (scope_) scope_->setXRange(static_cast<double>(sec));
        if (lbl_scope_time_) lbl_scope_time_->setText(QString::number(sec) + " s");
    });
    connect(cmb_scope_signal_, &QComboBox::currentTextChanged, this, [this](const QString&) {
        if (scope_) scope_->clear();
    });

    scope_layout->addLayout(scope_ctrl);
    scope_layout->addWidget(scope_);

    layout->addStretch(1);
    layout->addWidget(scope_group, 0, Qt::AlignBottom);

    connect(chk_sine_enable_, &QCheckBox::toggled, this, &AxisWorkspace::onSineToggled);

    connect(btn_enable, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "enableMotor", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_), Q_ARG(bool, true));
    });
    connect(btn_disable, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "enableMotor", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_), Q_ARG(bool, false));
    });
    connect(btn_estop, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "emergencyStop", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_));
    });
    connect(btn_clear_err, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "clearErrors", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_));
    });
    connect(btn_home, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "goHome", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_));
    });
    connect(btn_set_zero, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "setZeroPosition", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_));
    });

    connect(btn_move, &QPushButton::clicked, this, &AxisWorkspace::triggerAbsoluteMove);

    connect(mode_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int idx) {
        if (!mode_combo_ || !manager_) {
            return;
        }
        const int mode_code = mode_combo_->itemData(idx).toInt();
        QMetaObject::invokeMethod(manager_, "setAxisMode", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_), Q_ARG(int, mode_code));
    });

    connect(target_slider_, &QSlider::valueChanged, this, [this](int raw) {
        if (!target_pos_spin_) {
            return;
        }
        const double v = static_cast<double>(raw) / 100.0;
        target_pos_spin_->blockSignals(true);
        target_pos_spin_->setValue(v);
        target_pos_spin_->blockSignals(false);
        desired_target_deg_.store(v, std::memory_order_relaxed);
        commanded_target_deg_ = v;
        manual_target_hold_until_ms_ = QDateTime::currentMSecsSinceEpoch() + 1200;
        QMetaObject::invokeMethod(manager_, "moveAbsoluteAxis", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, speed_spin_ ? speed_spin_->value() : 400),
                                  Q_ARG(int, accel_spin_ ? accel_spin_->value() : 50),
                                  Q_ARG(double, v));
    });

    connect(target_pos_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double v) {
        if (target_pos_spin_) {
            desired_target_deg_.store(v, std::memory_order_relaxed);
            commanded_target_deg_ = v;
            manual_target_hold_until_ms_ = QDateTime::currentMSecsSinceEpoch() + 1500;
            if (target_slider_) {
                target_slider_->blockSignals(true);
                target_slider_->setValue(static_cast<int>(std::llround(v * 100.0)));
                target_slider_->blockSignals(false);
            }
        }
    });

    connect(btn_jog_neg_, &QPushButton::clicked, this, [this]() {
        if (!manager_) return;
        const double step = jog_step_spin_ ? jog_step_spin_->value() : 1.0;
        QMetaObject::invokeMethod(manager_, "moveRelativeAxis", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, speed_spin_ ? speed_spin_->value() : 400),
                                  Q_ARG(int, accel_spin_ ? accel_spin_->value() : 50),
                                  Q_ARG(double, -step));
    });
    connect(btn_jog_pos_, &QPushButton::clicked, this, [this]() {
        if (!manager_) return;
        const double step = jog_step_spin_ ? jog_step_spin_->value() : 1.0;
        QMetaObject::invokeMethod(manager_, "moveRelativeAxis", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, speed_spin_ ? speed_spin_->value() : 400),
                                  Q_ARG(int, accel_spin_ ? accel_spin_->value() : 50),
                                  Q_ARG(double, step));
    });
    
    // Fast path for background thread speeds
    connect(speed_spin_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int v){ current_speed_.store(v, std::memory_order_relaxed); });
    connect(accel_spin_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int v){ current_accel_.store(v, std::memory_order_relaxed); });

    tabs_->addTab(tab, "Control");
}

void AxisWorkspace::setupConfigTab() {
    auto* tab = new QWidget(this);
    auto* layout = new QVBoxLayout(tab);

    auto* btn_row = new QHBoxLayout();
    auto* btn_refresh_list = new QPushButton("Refresh List", tab);
    auto* btn_read = new QPushButton("Read Values", tab);
    auto* btn_apply = new QPushButton("Apply Changes", tab);
    auto* btn_export_full = new QPushButton("Export Config (AxisConfig)", tab);
    auto* btn_import_full = new QPushButton("Import Config (AxisConfig)", tab);
    auto* btn_save = new QPushButton("Save Current to JSON...", tab);
    auto* btn_load = new QPushButton("Load JSON...", tab);
    btn_row->addWidget(btn_refresh_list);
    btn_row->addWidget(btn_read);
    btn_row->addWidget(btn_apply);
    btn_row->addWidget(btn_export_full);
    btn_row->addWidget(btn_import_full);
    btn_row->addStretch();
    btn_row->addWidget(btn_save);
    btn_row->addWidget(btn_load);
    layout->addLayout(btn_row);

    auto* split = new QSplitter(Qt::Horizontal, tab);
    
    config_tree_ = new QTreeWidget(split);
    config_tree_->setColumnCount(6);
    config_tree_->setHeaderLabels({"Name", "Group", "Unit", "Read Only", "Current Value", "New Value"});
    config_tree_->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
    config_tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    config_tree_->setSelectionBehavior(QAbstractItemView::SelectRows);
    config_tree_->setSelectionMode(QAbstractItemView::SingleSelection);
    config_tree_->setEditTriggers(QAbstractItemView::DoubleClicked |
                                  QAbstractItemView::SelectedClicked |
                                  QAbstractItemView::EditKeyPressed);
    config_tree_->setItemDelegateForColumn(kNewValueColumn, new NewValueColumnDelegate(config_tree_));
    
    txt_description_ = new QTextEdit(split);
    txt_description_->setReadOnly(true);
    txt_description_->setPlaceholderText("Select a parameter to view its details...");
    
    split->setSizes({600, 200});
    layout->addWidget(split, 1);
    
    connect(config_tree_, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem* current, QTreeWidgetItem*) {
        if (!current) {
            txt_description_->clear();
            return;
        }
        
        // Build description text
        QString desc;
        desc += "<b>Name:</b> " + current->text(kNameColumn) + "<br>";
        desc += "<b>Group:</b> " + current->text(kGroupColumn) + "<br>";
        if (!current->text(kUnitColumn).isEmpty()) {
            desc += "<b>Unit:</b> " + current->text(kUnitColumn) + "<br>";
        }
        const int domain = current->data(kNameColumn, Qt::UserRole).toInt();
        const int value_id = current->data(kGroupColumn, Qt::UserRole).toInt();
        desc += "<b>Domain:</b> " + QString::number(domain) + "<br>";
        desc += "<b>Value ID:</b> " + QString::number(value_id) + "<br>";
        desc += "<b>Current:</b> " + current->text(kCurrentValueColumn) + "<br>";
        desc += "<b>Pending:</b> " + (current->text(kNewValueColumn).isEmpty() ? QString("-") : current->text(kNewValueColumn)) + "<br>";
        
        bool has_min = current->data(kUnitColumn, Qt::UserRole).toBool();
        bool has_max = current->data(kReadOnlyColumn, Qt::UserRole).toBool();
        QString min_val = current->data(kCurrentValueColumn, Qt::UserRole).toString();
        QString max_val = current->data(kNewValueColumn, Qt::UserRole).toString();
        
        if (has_min || has_max) {
            desc += "<br><b>Bounds:</b><br>";
            if (has_min) desc += "- Min: " + min_val + "<br>";
            if (has_max) desc += "- Max: " + max_val + "<br>";
        }

        const QString protocol_details = protocolDetailsForParameter(domain, value_id);
        if (!protocol_details.isEmpty()) {
            desc += "<br><b>Protocol details:</b><br>" + protocol_details;
        }
        
        txt_description_->setHtml(desc);
    });

    connect(btn_refresh_list, &QPushButton::clicked, this, &AxisWorkspace::refreshParameterList);
    connect(btn_read, &QPushButton::clicked, this, &AxisWorkspace::readParametersFromDrive);
    connect(btn_apply, &QPushButton::clicked, this, &AxisWorkspace::applyParametersPatch);
    connect(btn_export_full, &QPushButton::clicked, this, &AxisWorkspace::exportAxisConfig);
    connect(btn_import_full, &QPushButton::clicked, this, &AxisWorkspace::importAxisConfig);
    connect(btn_save, &QPushButton::clicked, this, &AxisWorkspace::saveParametersToFile);
    connect(btn_load, &QPushButton::clicked, this, &AxisWorkspace::loadParametersFromFile);

    tabs_->addTab(tab, "Configuration");
}

void AxisWorkspace::setupCommandsTab() {
    auto* tab = new QWidget(this);
    auto* layout = new QVBoxLayout(tab);

    auto* form = new QFormLayout();
    txt_cmd_byte_ = new QLineEdit(tab);
    txt_cmd_data_ = new QLineEdit(tab);
    txt_cmd_byte_->setPlaceholderText("e.g. F3");
    txt_cmd_data_->setPlaceholderText("e.g. 01");
    form->addRow("Command byte (hex):", txt_cmd_byte_);
    form->addRow("Data (hex, space separated):", txt_cmd_data_);
    layout->addLayout(form);

    auto* btn_row = new QHBoxLayout();
    auto* btn_send = new QPushButton("Send", tab);
    auto* btn_clear = new QPushButton("Clear log", tab);
    btn_row->addStretch();
    btn_row->addWidget(btn_send);
    btn_row->addWidget(btn_clear);
    layout->addLayout(btn_row);

    txt_cmd_log_ = new QTextEdit(tab);
    txt_cmd_log_->setReadOnly(true);
    txt_cmd_log_->append("Raw command panel. Enter bytes in hex.");
    layout->addWidget(txt_cmd_log_, 1);

    connect(btn_clear, &QPushButton::clicked, txt_cmd_log_, &QTextEdit::clear);
    connect(btn_send, &QPushButton::clicked, this, [this]() {
        bool byte_ok = false;
        const int cmd_byte = txt_cmd_byte_->text().toInt(&byte_ok, 16);
        if (!byte_ok || cmd_byte < 0 || cmd_byte > 255) {
            txt_cmd_log_->append("Invalid command byte");
            return;
        }

        const QStringList parts = txt_cmd_data_->text().split(' ', Qt::SkipEmptyParts);
        QByteArray data;
        for (const QString& p : parts) {
            bool hex_ok = false;
            int b = p.toInt(&hex_ok, 16);
            if (!hex_ok || b < 0 || b > 255) {
                txt_cmd_log_->append("Invalid data byte: " + p);
                return;
            }
            data.append(static_cast<char>(b));
        }

        txt_cmd_log_->append(QString("-> Cmd: %1 Data: %2")
                                 .arg(txt_cmd_byte_->text())
                                 .arg(txt_cmd_data_->text().isEmpty() ? "(none)" : txt_cmd_data_->text()));

        if (manager_) {
            QMetaObject::invokeMethod(manager_.data(), "sendRawCommand", Qt::QueuedConnection,
                                      Q_ARG(int, axis_id_),
                                      Q_ARG(quint8, static_cast<quint8>(cmd_byte)),
                                      Q_ARG(QByteArray, data));
        }
    });

    tabs_->addTab(tab, "Commands");
}

void AxisWorkspace::refreshParameterList() {
    config_tree_->clear();
    txt_description_->clear();
    if (!manager_) return;

    QMetaObject::invokeMethod(manager_.data(), "requestListParameters", Qt::QueuedConnection, Q_ARG(int, axis_id_));
}

void AxisWorkspace::onParameterListReady(int axis_id, const QVariantList& p_list) {
    if (axis_id != axis_id_) return;
    config_tree_->clear();
    txt_description_->clear();
    
    for (const auto& v : p_list) {
        QVariantMap map = v.toMap();
        auto* item = new QTreeWidgetItem(config_tree_);
        item->setText(0, map["name"].toString());
        item->setText(1, map["group"].toString());
        item->setText(2, map["unit"].toString());
        item->setText(3, map["read_only"].toBool() ? "Yes" : "No");
        item->setText(4, "Unknown");
        
        item->setData(kNameColumn, Qt::UserRole, map["domain"]);
        item->setData(kGroupColumn, Qt::UserRole, map["value"]);
        item->setData(kUnitColumn, Qt::UserRole, map["has_min"]);
        item->setData(kReadOnlyColumn, Qt::UserRole, map["has_max"]);
        item->setData(kCurrentValueColumn, Qt::UserRole, map["min_value"]);
        item->setData(kNewValueColumn, Qt::UserRole, map["max_value"]);
        
        if (!map["read_only"].toBool()) {
            item->setFlags(item->flags() | Qt::ItemIsEditable);
            item->setText(kNewValueColumn, "");
        } else {
            item->setText(kNewValueColumn, "N/A");
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
        }
    }
}

void AxisWorkspace::readParametersFromDrive() {
    if (!manager_) return;
    QMetaObject::invokeMethod(manager_.data(), "requestReadParameters", Qt::QueuedConnection, Q_ARG(int, axis_id_));
}

void AxisWorkspace::onParametersRead(int axis_id, const QVariantList& vals) {
    if (axis_id != axis_id_) return;

    for (const auto& v : vals) {
        QVariantMap map = v.toMap();
        int domain = map["domain"].toInt();
        int val_id = map["value"].toInt();
        QString data = map["data"].toString();

        for (int i = 0; i < config_tree_->topLevelItemCount(); ++i) {
            auto* item = config_tree_->topLevelItem(i);
            if (item->data(kNameColumn, Qt::UserRole).toInt() == domain &&
                item->data(kGroupColumn, Qt::UserRole).toInt() == val_id) {
                item->setText(kCurrentValueColumn, data);
                break;
            }
        }
    }
}

void AxisWorkspace::applyParametersPatch() {
    if (!manager_) return;

    auto parse_bool_token = [](const QString& token, bool* ok_out) -> bool {
        const QString normalized = token.trimmed().toLower();
        if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on") {
            if (ok_out) *ok_out = true;
            return true;
        }
        if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off") {
            if (ok_out) *ok_out = true;
            return false;
        }
        if (ok_out) *ok_out = false;
        return false;
    };

    auto variant_to_double = [](const QVariant& value, bool* ok_out) -> double {
        if (value.userType() == QMetaType::Bool) {
            if (ok_out) *ok_out = true;
            return value.toBool() ? 1.0 : 0.0;
        }
        bool ok = false;
        const double out = value.toDouble(&ok);
        if (ok_out) *ok_out = ok;
        return out;
    };

    QVariantList patch;
    QStringList validation_errors;

    for (int i = 0; i < config_tree_->topLevelItemCount(); ++i) {
        auto* item = config_tree_->topLevelItem(i);
        const QString new_val = item->text(kNewValueColumn).trimmed();
        if (new_val.isEmpty() || new_val == "N/A") {
            continue;
        }

        const bool has_min = item->data(kUnitColumn, Qt::UserRole).toBool();
        const bool has_max = item->data(kReadOnlyColumn, Qt::UserRole).toBool();
        const QVariant min_value = item->data(kCurrentValueColumn, Qt::UserRole);
        const QVariant max_value = item->data(kNewValueColumn, Qt::UserRole);
        const bool bool_like_bounds = has_min && has_max &&
                                      min_value.userType() == QMetaType::Bool &&
                                      max_value.userType() == QMetaType::Bool;

        QVariant parsed_value;
        bool parsed_ok = false;
        if (bool_like_bounds) {
            parsed_value = parse_bool_token(new_val, &parsed_ok);
        } else {
            bool bool_ok = false;
            const bool bool_value = parse_bool_token(new_val, &bool_ok);
            if (bool_ok) {
                parsed_value = bool_value;
                parsed_ok = true;
            } else {
                bool int_ok = false;
                const qlonglong int_value = new_val.toLongLong(&int_ok);
                if (int_ok && !new_val.contains('.') && !new_val.contains('e', Qt::CaseInsensitive)) {
                    parsed_value = int_value;
                    parsed_ok = true;
                } else {
                    bool double_ok = false;
                    const double double_value = new_val.toDouble(&double_ok);
                    if (double_ok) {
                        parsed_value = double_value;
                        parsed_ok = true;
                    } else {
                        parsed_value = new_val;
                        parsed_ok = !(has_min || has_max);
                    }
                }
            }
        }

        if (!parsed_ok) {
            validation_errors.push_back(QString("%1: invalid value '%2'")
                                            .arg(item->text(kNameColumn), new_val));
            continue;
        }

        if (has_min || has_max) {
            bool parsed_numeric_ok = false;
            const double parsed_numeric = variant_to_double(parsed_value, &parsed_numeric_ok);
            if (!parsed_numeric_ok) {
                validation_errors.push_back(QString("%1: value must be numeric/boolean")
                                                .arg(item->text(kNameColumn)));
                continue;
            }

            if (has_min) {
                bool min_ok = false;
                const double min_numeric = variant_to_double(min_value, &min_ok);
                if (min_ok && parsed_numeric < min_numeric) {
                    validation_errors.push_back(QString("%1: value %2 is below min %3")
                                                    .arg(item->text(kNameColumn))
                                                    .arg(new_val)
                                                    .arg(QString::number(min_numeric, 'g', 12)));
                    continue;
                }
            }
            if (has_max) {
                bool max_ok = false;
                const double max_numeric = variant_to_double(max_value, &max_ok);
                if (max_ok && parsed_numeric > max_numeric) {
                    validation_errors.push_back(QString("%1: value %2 is above max %3")
                                                    .arg(item->text(kNameColumn))
                                                    .arg(new_val)
                                                    .arg(QString::number(max_numeric, 'g', 12)));
                    continue;
                }
            }
        }

        QVariantMap map;
        map["domain"] = item->data(kNameColumn, Qt::UserRole);
        map["value"] = item->data(kGroupColumn, Qt::UserRole);
        map["data"] = parsed_value;

        patch.push_back(map);
        item->setText(kNewValueColumn, "");
    }

    if (!validation_errors.isEmpty()) {
        QMessageBox::warning(this,
                             "Invalid parameter values",
                             "Please fix invalid values before apply:\n- " + validation_errors.join("\n- "));
        return;
    }

    if (!patch.isEmpty()) {
        auto* manager = manager_.data();
        QMetaObject::invokeMethod(manager, [manager, id = axis_id_, p = patch]() {
            manager->applyParameterPatch(id, p);
        }, Qt::QueuedConnection);
        QTimer::singleShot(120, this, [this]() { readParametersFromDrive(); });
    }
}

void AxisWorkspace::saveParametersToFile() {
    if (config_tree_->topLevelItemCount() == 0) return;

    const QString path = QFileDialog::getSaveFileName(this, "Save Parameters", 
                                                      QString("axis_%1_params.json").arg(axis_id_), 
                                                      "JSON Files (*.json)");
    if (path.isEmpty()) return;

    QJsonArray array;
    for (int i = 0; i < config_tree_->topLevelItemCount(); ++i) {
        auto* item = config_tree_->topLevelItem(i);
        QString val = item->text(kCurrentValueColumn).trimmed();
        if (val.isEmpty() || val == "Unknown" || val == "N/A") continue;

        QJsonObject obj;
        obj["name"] = item->text(kNameColumn);
        obj["domain"] = item->data(kNameColumn, Qt::UserRole).toInt();
        obj["value"] = item->data(kGroupColumn, Qt::UserRole).toInt();
        
        bool ok = false;
        double d = val.toDouble(&ok);
        if (ok && val.contains('.')) {
            obj["data"] = d;
        } else {
            qlonglong l = val.toLongLong(&ok);
            if (ok) obj["data"] = l;
            else obj["data"] = val; 
        }
        array.append(obj);
    }

    QJsonObject root;
    root["axis_id"] = axis_id_;
    root["parameters"] = array;

    QFile f(path);
    if (f.open(QIODevice::WriteOnly)) {
        f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
        f.close();
    }
}

void AxisWorkspace::loadParametersFromFile() {
    const QString path = QFileDialog::getOpenFileName(this, "Load Parameters", "", "JSON Files (*.json)");
    if (path.isEmpty()) return;

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return;

    QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    f.close();

    if (!doc.isObject()) return;
    QJsonArray array = doc.object()["parameters"].toArray();

    for (int i = 0; i < array.size(); ++i) {
        QJsonObject obj = array[i].toObject();
        int domain = obj["domain"].toInt();
        int val_id = obj["value"].toInt();
        QVariant data = obj["data"].toVariant();

        for (int j = 0; j < config_tree_->topLevelItemCount(); ++j) {
            auto* item = config_tree_->topLevelItem(j);
            if (item->data(kNameColumn, Qt::UserRole).toInt() == domain &&
                item->data(kGroupColumn, Qt::UserRole).toInt() == val_id) {
                // don't apply read only
                if (!(item->flags() & Qt::ItemIsEditable)) break;
                item->setText(kNewValueColumn, data.toString());
                break;
            }
        }
    }
}

void AxisWorkspace::onSineToggled(bool enabled) {
    if (target_pos_spin_) target_pos_spin_->setEnabled(!enabled);
    
    if (manager_ && enabled) {
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            std::queue<double> empty;
            std::swap(trajectory_queue_, empty);
        }
        
        sine_center_deg_.store(commanded_target_deg_, std::memory_order_relaxed);
        sine_phase_accum_rad_ = 0.0;
        
        QFile::remove("sine_dump.csv");
        fillTrajectoryQueue(); // Pre-calculate the first segment immediately
    }

    if (!enabled) {
        desired_target_deg_.store(commanded_target_deg_, std::memory_order_relaxed);
    }
}

void AxisWorkspace::fillTrajectoryQueue() {
    if (!chk_sine_enable_ || !chk_sine_enable_->isChecked()) return;

    constexpr int kTargetDtMs = 4;
    constexpr double kTargetDtSec = kTargetDtMs / 1000.0;
    constexpr size_t kTargetBufferSize = 250; // 1 second buffer (250 * 4ms)

    size_t current_size = 0;
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        current_size = trajectory_queue_.size();
    }

    if (current_size >= kTargetBufferSize) return;

    const double target_amp = spin_sine_amp_ ? spin_sine_amp_->value() : 0.0;
    const double target_freq = spin_sine_freq_ ? spin_sine_freq_->value() : 0.0;
    const double center = sine_center_deg_.load(std::memory_order_relaxed);

    std::vector<double> new_points;
    new_points.reserve(kTargetBufferSize - current_size);

    for (size_t i = current_size; i < kTargetBufferSize; ++i) {
        sine_phase_accum_rad_ += 2.0 * M_PI * target_freq * kTargetDtSec;
        if (sine_phase_accum_rad_ > 2.0 * M_PI) {
            sine_phase_accum_rad_ = std::fmod(sine_phase_accum_rad_, 2.0 * M_PI);
        }
        new_points.push_back(center + target_amp * std::sin(sine_phase_accum_rad_));
    }

    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        for (double p : new_points) {
            trajectory_queue_.push(p);
        }
    }
}

void AxisWorkspace::trajectoryLoop() {
    constexpr int kTargetDtMs = 4; // 250Hz target loop
    
    auto next_wakeup = std::chrono::high_resolution_clock::now();

    while (trajectory_thread_active_.load()) {
        next_wakeup += std::chrono::milliseconds(kTargetDtMs);
        
        if (!manager_) {
            std::this_thread::sleep_until(next_wakeup);
            continue;
        }

        bool has_target = false;
        double new_target = 0.0;

        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            if (!trajectory_queue_.empty()) {
                new_target = trajectory_queue_.front();
                trajectory_queue_.pop();
                has_target = true;
            }
        }

        if (has_target) {
            desired_target_deg_.store(new_target, std::memory_order_relaxed);
            
            QMetaObject::invokeMethod(manager_.data(), "moveAbsoluteAxis", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, current_speed_.load(std::memory_order_relaxed)),
                                  Q_ARG(int, current_accel_.load(std::memory_order_relaxed)),
                                  Q_ARG(double, new_target));
                                  
            QMetaObject::invokeMethod(this, [this, new_target](){ 
                if (target_pos_spin_) {
                    target_pos_spin_->blockSignals(true);
                    target_pos_spin_->setValue(new_target);
                    target_pos_spin_->blockSignals(false);
                }
                if (target_slider_) {
                    target_slider_->blockSignals(true);
                    target_slider_->setValue(static_cast<int>(std::llround(new_target * 100.0)));
                    target_slider_->blockSignals(false);
                }
            }, Qt::QueuedConnection);
            
        } else {
            // Re-sync with UI if necessary/starved
            double ui_target = 0;
             QMetaObject::invokeMethod(this, [this, &ui_target](){ 
                if(target_pos_spin_) ui_target = target_pos_spin_->value();
            }, Qt::BlockingQueuedConnection);
            desired_target_deg_.store(ui_target, std::memory_order_relaxed);
        }
        
        commanded_target_deg_ = desired_target_deg_.load(std::memory_order_relaxed);
        double cmd_tgt = commanded_target_deg_;
        
        double t_sec = 0.0;
        if (telemetry_t0_ms_ > 0) {
            t_sec = static_cast<double>(QDateTime::currentMSecsSinceEpoch() - telemetry_t0_ms_) / 1000.0;
        }

        // 1. Thread-safe CSV writing (pure background operation)
        if (has_target && telemetry_t0_ms_ > 0) {
            QFile f("sine_dump.csv");
            if (f.open(QIODevice::Append | QIODevice::Text)) {
                QTextStream out(&f);
                out << t_sec << "," << cmd_tgt << "\n";
                f.close();
            }

            QMetaObject::invokeMethod(this, [this, t_sec, cmd_tgt]() {
                if (!scope_) return;
                if (chk_plot_target_pos_ && chk_plot_target_pos_->isChecked()) {
                    scope_->addData("target", t_sec, cmd_tgt);
                }
                if (chk_plot_target_vel_ && chk_plot_target_vel_->isChecked()) {
                    if (have_prev_target_sample_ && (t_sec - prev_target_sample_time_sec_) > 1e-6) {
                        const double target_vel = (cmd_tgt - prev_target_sample_deg_) / (t_sec - prev_target_sample_time_sec_);
                        scope_->addData("target_speed", t_sec, target_vel);
                    }
                    prev_target_sample_deg_ = cmd_tgt;
                    prev_target_sample_time_sec_ = t_sec;
                    have_prev_target_sample_ = true;
                }
            }, Qt::QueuedConnection);
        }
        
        std::this_thread::sleep_until(next_wakeup);
    }
}

void AxisWorkspace::setTargetPosition(double pos_deg) {
    if (target_pos_spin_) {
        manual_target_hold_until_ms_ = QDateTime::currentMSecsSinceEpoch() + 800;
        desired_target_deg_.store(pos_deg, std::memory_order_relaxed);
        commanded_target_deg_ = pos_deg;
        target_pos_spin_->setValue(pos_deg);
    }
    if (target_slider_) {
        target_slider_->blockSignals(true);
        target_slider_->setValue(static_cast<int>(std::llround(pos_deg * 100.0)));
        target_slider_->blockSignals(false);
    }
}

void AxisWorkspace::triggerAbsoluteMove() {
    if (!manager_) return;

    if (target_pos_spin_) {
        // Commit in-progress typed text before reading value (important when user types and directly clicks Move).
        target_pos_spin_->interpretText();
    }

    const double manual_target = target_pos_spin_ ? target_pos_spin_->value() : 0.0;
    desired_target_deg_.store(manual_target, std::memory_order_relaxed);
    commanded_target_deg_ = manual_target;
    manual_target_hold_until_ms_ = QDateTime::currentMSecsSinceEpoch() + 1200;
    
    if (radio_move_rel_ && radio_move_rel_->isChecked()) {
        QMetaObject::invokeMethod(manager_, "moveRelativeAxis", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, speed_spin_ ? speed_spin_->value() : 400),
                                  Q_ARG(int, accel_spin_ ? accel_spin_->value() : 50),
                                  Q_ARG(double, manual_target));
    } else {
        QMetaObject::invokeMethod(manager_, "moveAbsoluteAxis", Qt::QueuedConnection,
                                  Q_ARG(int, axis_id_),
                                  Q_ARG(int, speed_spin_ ? speed_spin_->value() : 400),
                                  Q_ARG(int, accel_spin_ ? accel_spin_->value() : 50),
                                  Q_ARG(double, manual_target));
    }
}

bool AxisWorkspace::isTargetReached(double tolerance_deg) const {
    return std::abs(displayed_actual_deg_ - (target_pos_spin_ ? target_pos_spin_->value() : 0)) <= tolerance_deg;
}

void AxisWorkspace::onTelemetryUpdated(int axis_id, const QVariantMap& telemetry) {
    if (axis_id != axis_id_) return;

    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    if (telemetry_t0_ms_ == 0) {
        telemetry_t0_ms_ = now_ms;
    }
    const double t_sec = static_cast<double>(now_ms - telemetry_t0_ms_) / 1000.0;

    if (scope_) {
        const QString signal = cmb_scope_signal_ ? cmb_scope_signal_->currentText() : QStringLiteral("Position (deg)");
        const bool show_position = signal.startsWith("Position");
        const bool show_velocity = signal.startsWith("Velocity");
        const bool show_torque = signal.startsWith("Torque");

        scope_->setChannelVisibility("actual", show_position && chk_plot_actual_pos_ && chk_plot_actual_pos_->isChecked());
        scope_->setChannelVisibility("target", show_position && chk_plot_target_pos_ && chk_plot_target_pos_->isChecked());
        scope_->setChannelVisibility("pos_error", show_position && chk_plot_pos_error_ && chk_plot_pos_error_->isChecked());
        scope_->setChannelVisibility("speed", show_velocity && chk_plot_actual_vel_ && chk_plot_actual_vel_->isChecked());
        scope_->setChannelVisibility("target_speed", show_velocity && chk_plot_target_vel_ && chk_plot_target_vel_->isChecked());
        scope_->setChannelVisibility("torque", show_torque && chk_plot_actual_vel_ && chk_plot_actual_vel_->isChecked());

        if (show_position && chk_plot_actual_pos_ && chk_plot_actual_pos_->isChecked() && telemetry.contains("axis")) {
            scope_->addData("actual", t_sec, telemetry.value("axis").toDouble());
        }
        if (show_position && chk_plot_target_pos_ && chk_plot_target_pos_->isChecked() && telemetry.contains("target") && !chk_sine_enable_->isChecked()) {
            scope_->addData("target", t_sec, telemetry.value("target").toDouble());
        }
        if (show_position && chk_plot_pos_error_ && chk_plot_pos_error_->isChecked() && telemetry.contains("axis") && telemetry.contains("target")) {
            const double err = telemetry.value("target").toDouble() - telemetry.value("axis").toDouble();
            scope_->addData("pos_error", t_sec, err);
        }
        if (show_velocity && chk_plot_actual_vel_ && chk_plot_actual_vel_->isChecked() && telemetry.contains("speed")) {
            scope_->addData("speed", t_sec, telemetry.value("speed").toDouble());
        }
        if (show_torque && chk_plot_actual_vel_ && chk_plot_actual_vel_->isChecked() && telemetry.contains("torque")) {
            scope_->addData("torque", t_sec, telemetry.value("torque").toDouble());
        }
    }

    if (telemetry.contains("state")) {
        int state = telemetry.value("state").toInt();
        QString state_str;
        switch (state) {
            case 0: state_str = "Unknown"; break;
            case 1: state_str = "Disabled"; break;
            case 2: state_str = "Ready"; break;
            case 3: state_str = "Moving/Enabled"; break;
            case 4: state_str = "Fault / E-Stop"; break;
            default: state_str = "N/A"; break;
        }
        if (lbl_sys_state_) lbl_sys_state_->setText(state_str);
    }
    if (telemetry.contains("status")) {
        lbl_state_->setText(QString("0x%1").arg(telemetry.value("status").toInt(), 4, 16, QLatin1Char('0')));
    }
    if (telemetry.contains("axis")) {
        const double axis_val = telemetry.value("axis").toDouble();
        displayed_actual_deg_ = axis_val; // Save for sequencer checking
        lbl_axis_->setText(QString::number(axis_val, 'f', 2) + " °");
        if (!target_seeded_from_telemetry_ && target_pos_spin_) {
            target_seeded_from_telemetry_ = true;
            target_pos_spin_->blockSignals(true);
            target_pos_spin_->setValue(axis_val);
            target_pos_spin_->blockSignals(false);
            desired_target_deg_.store(axis_val, std::memory_order_relaxed);
            commanded_target_deg_ = axis_val;
        }
    }
    if (telemetry.contains("target")) {
        const double target_val = telemetry.value("target").toDouble();
        if (lbl_target_) lbl_target_->setText(QString::number(target_val, 'f', 2) + " °");
        const qint64 now_ms_guard = QDateTime::currentMSecsSinceEpoch();
        const bool manual_hold_active = now_ms_guard < manual_target_hold_until_ms_;
        if (target_pos_spin_ && (!chk_sine_enable_ || !chk_sine_enable_->isChecked()) && !target_pos_spin_->hasFocus() && !manual_hold_active) {
            target_pos_spin_->blockSignals(true);
            target_pos_spin_->setValue(target_val);
            target_pos_spin_->blockSignals(false);
            if (target_slider_) {
                target_slider_->blockSignals(true);
                target_slider_->setValue(static_cast<int>(std::llround(target_val * 100.0)));
                target_slider_->blockSignals(false);
            }
        }
    } else if (lbl_target_) {
        lbl_target_->setText(QString::number(target_pos_spin_->value(), 'f', 2) + " °");
    }

    if (telemetry.contains("speed")) {
        lbl_speed_->setText(QString::number(telemetry.value("speed").toDouble(), 'f', 1) + " °/s");
    }
    if (telemetry.contains("torque")) {
        lbl_torque_->setText(QString::number(telemetry.value("torque").toDouble(), 'f', 1) + " %");
    }
    if (telemetry.contains("protection")) {
        lbl_protection_->setText(QString::number(telemetry.value("protection").toInt()));
    }
}

// Removed config readback slot
