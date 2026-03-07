#include "mks/main_window.h"

#include "mks/axis_manager.h"
#include "mks/axis_workspace.h"
#include "mks/port/gs_usb_can_port.h"

#include <QDateTime>
#include <QComboBox>
#include <QDockWidget>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMdiArea>
#include <QMdiSubWindow>
#include <QMetaObject>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTextEdit>
#include <QThread>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QMenu>
#include <QInputDialog>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QCheckBox>
#include <QStatusBar>
#include <QFileDialog>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setupUi();

    manager_thread_ = new QThread(this);
    manager_ = new mks::AxisManager();
    manager_->moveToThread(manager_thread_);
    connect(manager_thread_, &QThread::finished, manager_, &QObject::deleteLater);
    manager_thread_->start();

    connect(manager_, &mks::AxisManager::logMessage, this, [this](const QString& s) { appendLog("[MKS] " + s); });
    connect(manager_, &mks::AxisManager::logMessage, this, [this](const QString& s) {
        if (s.contains("EtherCAT", Qt::CaseInsensitive) || s.contains("ECAT", Qt::CaseInsensitive)) {
            appendLog("[ECAT] " + s);
        }
    });
    
    connect(manager_, &mks::AxisManager::busStatisticsUpdated, this, [this](double cycle_rate_hz, double bus_load_percent) {
        if (status_stats_label_) {
            status_stats_label_->setText(QString("Bus Load: %1% | Cycle Rate: %2 Hz")
                                            .arg(bus_load_percent, 0, 'f', 1)
                                            .arg(cycle_rate_hz, 0, 'f', 1));
        }
        if (bus_load_value_label_) {
            bus_load_value_label_->setText(QString::number(bus_load_percent, 'f', 1) + "%");
        }
        if (bus_rate_value_label_) {
            bus_rate_value_label_->setText(QString::number(cycle_rate_hz, 'f', 1) + " Hz");
        }
    });

    connect(manager_, &mks::AxisManager::scanFinished, this, [this](const QVariantList& ids) {
        if (!topology_tree_) return;
        QTreeWidgetItem* root = nullptr;
        if (topology_tree_->topLevelItemCount() == 0) {
            root = new QTreeWidgetItem(topology_tree_);
            root->setText(0, "HexaKinetica Runtime");
            root->setExpanded(true);
        } else {
            root = topology_tree_->topLevelItem(0);
        }

        // Find or create MKS bus node
        QTreeWidgetItem* bus = nullptr;
        for (int i = 0; i < root->childCount(); ++i) {
            if (root->child(i)->data(0, Qt::UserRole).toString() == "mks_bus") {
                bus = root->child(i);
                break;
            }
        }
        if (!bus) {
            bus = new QTreeWidgetItem(root);
            bus->setText(0, "MKS CAN Bus");
            bus->setData(0, Qt::UserRole, "mks_bus");
            bus->setExpanded(true);
        } else {
            qDeleteAll(bus->takeChildren());
        }

        for (int i = 0; i < ids.size(); ++i) {
            auto* axis = new QTreeWidgetItem(bus);
            const int id = ids[i].toInt();
            axis->setText(0, QString("MKS Axis %1 (Online)").arg(id));
            axis->setData(0, Qt::UserRole, "mks_axis");
            axis->setData(0, Qt::UserRole + 1, id);
        }
    });

    connect(manager_, &mks::AxisManager::ethercatScanFinished, this, [this](const QVariantList& ids) {
        if (!topology_tree_) return;
        QTreeWidgetItem* root = nullptr;
        if (topology_tree_->topLevelItemCount() == 0) {
            root = new QTreeWidgetItem(topology_tree_);
            root->setText(0, "HexaKinetica Runtime");
            root->setExpanded(true);
        } else {
            root = topology_tree_->topLevelItem(0);
        }

        QTreeWidgetItem* bus = nullptr;
        for (int i = 0; i < root->childCount(); ++i) {
            if (root->child(i)->data(0, Qt::UserRole).toString() == "ecat_bus") {
                bus = root->child(i);
                break;
            }
        }
        if (!bus) {
            bus = new QTreeWidgetItem(root);
            bus->setText(0, "EtherCAT Bus");
            bus->setData(0, Qt::UserRole, "ecat_bus");
            bus->setExpanded(true);
        } else {
            qDeleteAll(bus->takeChildren());
        }

        for (int i = 0; i < ids.size(); ++i) {
            auto* axis = new QTreeWidgetItem(bus);
            const int id = ids[i].toInt();
            axis->setText(0, QString("ECAT Axis %1 (Online)").arg(id));
            axis->setData(0, Qt::UserRole, "ecat_axis");
            axis->setData(0, Qt::UserRole + 1, id);
        }

        if (chk_ecat_auto_connect_ && chk_ecat_auto_connect_->isChecked()) {
            if (btn_ecat_start_runtime_) {
                btn_ecat_start_runtime_->click();
            }
        }
    });
    
    connect(btn_open_, &QPushButton::clicked, this, [this]() {
        const QString dev = device_combo_->currentData().toString();
        const int baud = baud_combo_->currentData().toInt();
        QMetaObject::invokeMethod(manager_, "openDevice", Qt::QueuedConnection,
                                  Q_ARG(QString, dev), Q_ARG(int, baud));
    });

    connect(btn_close_, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "closeDevice", Qt::QueuedConnection);
    });

    connect(btn_load_hal_, &QPushButton::clicked, this, [this]() {
        const QString path = QFileDialog::getOpenFileName(this, "Load Master HAL Config", "", "JSON Files (*.json)");
        if (!path.isEmpty()) {
            QMetaObject::invokeMethod(manager_, "loadHalConfig", Qt::QueuedConnection, Q_ARG(QString, path));
        }
    });

    connect(findChild<QPushButton*>("btn_refresh"), &QPushButton::clicked, this, [this]() {
        const QString prev_selected = device_combo_->currentData().toString();
        device_combo_->clear();

        const auto devs = mks::GsUsbCanPort::enumerateDevices();
        int selected_index = -1;

        for (const auto& d : devs) {
            device_combo_->addItem(QString::fromStdString(d.description + " (" + d.path + ")"),
                                   QString::fromStdString(d.path));
            if (QString::fromStdString(d.path) == prev_selected) {
                selected_index = device_combo_->count() - 1;
            }
        }

        const int simulator_index = device_combo_->count();
        device_combo_->addItem("Simulator (virtual MKS bus)", QString("sim:default"));
        if (prev_selected == QStringLiteral("sim:default")) {
            selected_index = simulator_index;
        }

        // Prefer real hardware by default. Simulator stays available but is no longer implicit.
        if (selected_index < 0) {
            selected_index = devs.empty() ? simulator_index : 0;
        }
        if (selected_index >= 0 && selected_index < device_combo_->count()) {
            device_combo_->setCurrentIndex(selected_index);
        }

        appendLog(QString("Devices refreshed: %1 (selected: %2)")
                      .arg(devs.size())
                      .arg(device_combo_->currentText()));

        if (devs.empty()) {
            appendLog("No GS-USB hardware found. Simulator selected.");
        }
    });

    // MKS Scan Button connection
    connect(findChild<QPushButton*>("btn_scan"), &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "scanMotors", Qt::QueuedConnection,
                                  Q_ARG(int, scan_max_id_spin_->value()));
    });

    // EtherCAT Panel Connections
    connect(btn_ecat_open_, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "openEthercatDevice", Qt::QueuedConnection,
                                  Q_ARG(QString, ethercat_iface_edit_->text()));
    });
    connect(btn_ecat_scan_, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "scanEthercatMotors", Qt::QueuedConnection);
    });
    connect(btn_ecat_start_runtime_, &QPushButton::clicked, this, [this]() {
        QMetaObject::invokeMethod(manager_, "startRuntime", Qt::QueuedConnection);
    });

    findChild<QPushButton*>("btn_refresh")->click();
}

MainWindow::~MainWindow() {
    if (manager_thread_) {
        manager_thread_->quit();
        manager_thread_->wait();
    }
}

void MainWindow::setupUi() {
    setWindowTitle("MKS CAN Axis Studio");
    resize(1300, 850);

    mdi_area_ = new QMdiArea(this);
    mdi_area_->setViewMode(QMdiArea::TabbedView);
    mdi_area_->setTabsClosable(true);
    setCentralWidget(mdi_area_);

    auto* dock_network = new QDockWidget("Network", this);
    dock_network->setMinimumWidth(250);
    dock_network->setMaximumWidth(350);
    auto* network_host = new QWidget(dock_network);
    auto* network_layout = new QVBoxLayout(network_host);

    auto* form = new QFormLayout();
    device_combo_ = new QComboBox(this);
    baud_combo_ = new QComboBox(this);
    baud_combo_->addItem("125000", 125000);
    baud_combo_->addItem("250000", 250000);
    baud_combo_->addItem("500000", 500000);
    baud_combo_->addItem("1000000", 1000000);
    baud_combo_->setCurrentText("500000");
    scan_max_id_spin_ = new QSpinBox(this);
    scan_max_id_spin_->setRange(1, 2047);
    scan_max_id_spin_->setValue(64);
    
    form->addRow("Device:", device_combo_);
    form->addRow("Baud:", baud_combo_);
    form->addRow("Scan max:", scan_max_id_spin_);
    network_layout->addLayout(form);

    auto* btn_row1 = new QHBoxLayout();
    auto* btn_refresh = new QPushButton("Refresh", this);
    btn_refresh->setObjectName("btn_refresh");
    btn_open_ = new QPushButton("Open", this);
    btn_close_ = new QPushButton("Close", this);
    btn_load_hal_ = new QPushButton("Load Master HAL", this);
    btn_row1->addWidget(btn_refresh);
    btn_row1->addWidget(btn_open_);
    btn_row1->addWidget(btn_close_);
    network_layout->addLayout(btn_row1);
    network_layout->addWidget(btn_load_hal_);

    auto* btn_row2 = new QHBoxLayout();
    auto* btn_scan = new QPushButton("Scan", this);
    btn_scan->setObjectName("btn_scan");
    btn_row2->addWidget(btn_scan);
    network_layout->addLayout(btn_row2);

    // EtherCAT Panel
    auto* ecat_group = new QGroupBox("EtherCAT Network", this);
    auto* ecat_layout = new QVBoxLayout(ecat_group);
    
    auto* ecat_form = new QFormLayout();
    ethercat_iface_edit_ = new QLineEdit("enp2s0", this); // Hardware interface name
    ecat_form->addRow("Interface:", ethercat_iface_edit_);
    ecat_layout->addLayout(ecat_form);
    
    auto* ecat_btn_row = new QHBoxLayout();
    btn_ecat_open_ = new QPushButton("Master Open", this);
    btn_ecat_scan_ = new QPushButton("Scan", this);
    
    chk_ecat_auto_connect_ = new QCheckBox("Auto-Connect", this);
    chk_ecat_auto_connect_->setChecked(true);
    
    btn_ecat_start_runtime_ = new QPushButton("Connect", this);
    btn_ecat_start_runtime_->setStyleSheet("color: blue; font-weight: bold;");
    
    ecat_btn_row->addWidget(btn_ecat_open_);
    ecat_btn_row->addWidget(btn_ecat_scan_);
    ecat_btn_row->addWidget(chk_ecat_auto_connect_);
    ecat_btn_row->addWidget(btn_ecat_start_runtime_);
    ecat_layout->addLayout(ecat_btn_row);
    
    network_layout->addWidget(ecat_group);

    auto* bus_stats_group = new QGroupBox("Bus statistics", this);
    auto* bus_stats_form = new QFormLayout(bus_stats_group);
    bus_load_value_label_ = new QLabel("--- %", bus_stats_group);
    bus_rate_value_label_ = new QLabel("--- Hz", bus_stats_group);
    bus_stats_form->addRow("Load:", bus_load_value_label_);
    bus_stats_form->addRow("Cycle rate:", bus_rate_value_label_);
    network_layout->addWidget(bus_stats_group);

    topology_tree_ = new QTreeWidget(this);
    topology_tree_->setHeaderLabel("Deployment Topology");
    topology_tree_->setContextMenuPolicy(Qt::CustomContextMenu);
    network_layout->addWidget(topology_tree_, 1);

    connect(topology_tree_, &QTreeWidget::itemDoubleClicked, this, [this](QTreeWidgetItem* item, int) {
        if (!item) return;
        if (item->data(0, Qt::UserRole).toString() == "mks_axis") {
            bool ok = false;
            const int id = item->data(0, Qt::UserRole + 1).toInt(&ok);
            if (ok) openAxisWorkspace(static_cast<uint16_t>(id));
        } else if (item->data(0, Qt::UserRole).toString() == "ecat_axis") {
            bool ok = false;
            const int id = item->data(0, Qt::UserRole + 1).toInt(&ok);
            if (ok) openEthercatWorkspace(static_cast<uint16_t>(id));
        }
    });

    connect(topology_tree_, &QTreeWidget::customContextMenuRequested, this, &MainWindow::onTopologyContextMenu);

    dock_network->setWidget(network_host);
    addDockWidget(Qt::LeftDockWidgetArea, dock_network);

    dock_log_ = new QDockWidget("Log", this);
    log_view_ = new QTextEdit(this);
    log_view_->setReadOnly(true);
    dock_log_->setWidget(log_view_);
    addDockWidget(Qt::BottomDockWidgetArea, dock_log_);

    status_stats_label_ = new QLabel("Bus Load: ---% | Cycle Rate: --- Hz", this);
    statusBar()->addPermanentWidget(status_stats_label_);
}

void MainWindow::appendLog(const QString& line) {
    if (!log_view_) return;
    log_view_->append(QDateTime::currentDateTime().toString("HH:mm:ss.zzz") + " | " + line);
}

void MainWindow::openAxisWorkspace(uint16_t axis_id) {
    if (opened_axes_.contains(axis_id) && opened_axes_[axis_id]) {
        if (auto* sub = qobject_cast<QMdiSubWindow*>(opened_axes_[axis_id]->parentWidget())) {
            mdi_area_->setActiveSubWindow(sub);
            return;
        }
    }

    auto* ws = new AxisWorkspace(static_cast<int>(axis_id), manager_);
    connect(ws, &QObject::destroyed, this, [this, axis_id]() { opened_axes_.remove(axis_id); });
    auto* sub = mdi_area_->addSubWindow(ws);
    sub->setWindowTitle(QString("MKS Axis %1").arg(axis_id));
    sub->resize(860, 620);
    sub->show();
    opened_axes_.insert(axis_id, ws);
}

void MainWindow::openEthercatWorkspace(uint16_t axis_id) {
    if (opened_ecat_axes_.contains(axis_id) && opened_ecat_axes_[axis_id]) {
        if (auto* sub = qobject_cast<QMdiSubWindow*>(opened_ecat_axes_[axis_id]->parentWidget())) {
            mdi_area_->setActiveSubWindow(sub);
            return;
        }
    }

    auto* ws = new AxisWorkspace(static_cast<int>(axis_id), manager_);
    connect(ws, &QObject::destroyed, this, [this, axis_id]() { opened_ecat_axes_.remove(axis_id); });
    auto* sub = mdi_area_->addSubWindow(ws);
    sub->setWindowTitle(QString("ECAT Axis %1").arg(axis_id));
    sub->resize(860, 620);
    sub->show();
    opened_ecat_axes_.insert(axis_id, ws);
}

void MainWindow::onTopologyContextMenu(const QPoint& pos) {
    Q_UNUSED(pos);
}
