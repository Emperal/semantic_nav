#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback

from bsonrpc import JSONRpc

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QTableWidget,
    QTableWidgetItem,
    QPushButton,
    QLabel,
    QLineEdit,
    QDoubleSpinBox,
    QTextEdit,
    QMessageBox,
    QHeaderView,
    QGroupBox,
    QFormLayout,
    QSplitter,
    QFileDialog,
    QCheckBox,
)


DEFAULT_WS_DIR = os.path.expanduser("~/hzf/1asemantic_nav")


class ProcessSignals(QObject):
    log = pyqtSignal(str)
    state = pyqtSignal(str, str)


class ManagedProcess:
    """Start and stop one long-running shell process as a Linux process group."""

    def __init__(self, name, signals):
        self.name = name
        self.signals = signals
        self.proc = None
        self.reader_thread = None

    def is_running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, cmd, cwd=None):
        if self.is_running():
            raise RuntimeError(f"{self.name} 已经在运行")

        self.signals.log.emit(f"\n===== 启动 {self.name} =====\n$ {cmd}\n")

        self.proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,
        )
        self.signals.state.emit(self.name, "running")

        self.reader_thread = threading.Thread(target=self._read_output, daemon=True)
        self.reader_thread.start()

    def _read_output(self):
        try:
            if self.proc is None or self.proc.stdout is None:
                return
            for line in self.proc.stdout:
                self.signals.log.emit(f"[{self.name}] {line.rstrip()}")
        except Exception as e:
            self.signals.log.emit(f"[{self.name}] 读取输出失败: {e}")
        finally:
            code = None
            if self.proc is not None:
                code = self.proc.poll()
                if code is None:
                    code = self.proc.wait()
            self.signals.log.emit(f"===== {self.name} 已退出，returncode={code} =====")
            self.signals.state.emit(self.name, "stopped")

    def stop(self, sig=signal.SIGINT, timeout=5.0):
        if not self.is_running():
            self.signals.log.emit(f"{self.name} 当前没有运行")
            self.signals.state.emit(self.name, "stopped")
            return

        pgid = os.getpgid(self.proc.pid)
        self.signals.log.emit(f"\n===== 停止 {self.name}: 发送 {sig.name} 到进程组 {pgid} =====")
        os.killpg(pgid, sig)

        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.is_running():
                return
            time.sleep(0.1)

        if self.is_running():
            self.signals.log.emit(f"{self.name} 未正常退出，发送 SIGTERM")
            os.killpg(pgid, signal.SIGTERM)
            time.sleep(1.0)

        if self.is_running():
            self.signals.log.emit(f"{self.name} 仍未退出，发送 SIGKILL")
            os.killpg(pgid, signal.SIGKILL)


class RpcWorker(QThread):
    done = pyqtSignal(str, object)
    failed = pyqtSignal(str, str)

    def __init__(self, host, port, timeout, command, args=None):
        super().__init__()
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self.command = command
        self.args = args or {}

    def run(self):
        try:
            result = self.call_rpc()
            self.done.emit(self.command, result)
        except Exception as e:
            self.failed.emit(self.command, f"{type(e).__name__}: {e}\n{traceback.format_exc()}")

    def call_rpc(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect((self.host, self.port))

        rpc = JSONRpc(s)
        server = rpc.get_peer_proxy()

        try:
            if self.command == "state":
                return server.get_state()
            if self.command == "list":
                return server.list_objects()
            if self.command == "labels":
                return server.list_labels()
            if self.command == "find_label":
                return server.find_by_label(str(self.args["label"]))
            if self.command == "get_object":
                return server.get_object(int(self.args["object_id"]))
            if self.command == "publish_cloud":
                return server.publish_object_cloud(int(self.args["object_id"]))
            if self.command == "candidates":
                return server.candidate_goals_for_object(
                    int(self.args["object_id"]),
                    float(self.args["approach_distance"]),
                )
            if self.command == "goto_object":
                return server.goto_object(
                    int(self.args["object_id"]),
                    float(self.args["approach_distance"]),
                )
            if self.command == "goto_label":
                return server.goto_label(
                    str(self.args["label"]),
                    float(self.args["approach_distance"]),
                )
            if self.command == "reload":
                return server.reload_map()
            if self.command == "recompute_icp":
                return server.recompute_icp()
            raise ValueError(f"unknown command: {self.command}")
        finally:
            rpc.close()


class SemanticNavGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("语义物体目标导航上位机")
        self.resize(1450, 860)

        self.objects = []
        self.current_object = None
        self.current_label_table = []
        self.workers = []

        self.process_signals = ProcessSignals()
        self.process_signals.log.connect(self.append_log)
        self.process_signals.state.connect(self.on_process_state)
        self.proc_mapping = ManagedProcess("mapping", self.process_signals)
        self.proc_nav = ManagedProcess("nav_final", self.process_signals)
        self.proc_rviz = ManagedProcess("rviz2", self.process_signals)
        self.proc_extra = ManagedProcess("extra", self.process_signals)
        self.proc_terminal = ManagedProcess("terminal", self.process_signals)

        self.build_ui()

    def build_ui(self):
        root = QVBoxLayout(self)

        process_box = QGroupBox("系统启动 / 建图 / 保存地图 / 导航可视化")
        process_layout = QVBoxLayout(process_box)

        path_layout = QHBoxLayout()
        self.ws_dir_edit = QLineEdit(DEFAULT_WS_DIR)
        self.mapping_script_edit = QLineEdit(os.path.join(DEFAULT_WS_DIR, "mapping.sh"))
        self.save_map_script_edit = QLineEdit(os.path.join(DEFAULT_WS_DIR, "save_map.sh"))
        self.nav_script_edit = QLineEdit(os.path.join(DEFAULT_WS_DIR, "nav_final.sh"))
        self.rviz_config_edit = QLineEdit("")
        self.rviz_config_edit.setPlaceholderText("可选：RViz 配置文件路径，例如 .../rviz/nav2.rviz")

        self.btn_choose_ws = QPushButton("选择工作空间")
        self.btn_choose_mapping = QPushButton("选择 mapping.sh")
        self.btn_choose_save = QPushButton("选择 save_map.sh")
        self.btn_choose_nav = QPushButton("选择 nav_final.sh")
        self.btn_choose_rviz = QPushButton("选择 RViz 配置")

        path_layout.addWidget(QLabel("工作空间:"))
        path_layout.addWidget(self.ws_dir_edit, 2)
        path_layout.addWidget(self.btn_choose_ws)
        process_layout.addLayout(path_layout)

        script_layout_1 = QHBoxLayout()
        script_layout_1.addWidget(QLabel("建图脚本:"))
        script_layout_1.addWidget(self.mapping_script_edit, 2)
        script_layout_1.addWidget(self.btn_choose_mapping)
        script_layout_1.addWidget(QLabel("保存脚本:"))
        script_layout_1.addWidget(self.save_map_script_edit, 2)
        script_layout_1.addWidget(self.btn_choose_save)
        process_layout.addLayout(script_layout_1)

        script_layout_2 = QHBoxLayout()
        script_layout_2.addWidget(QLabel("导航脚本:"))
        script_layout_2.addWidget(self.nav_script_edit, 2)
        script_layout_2.addWidget(self.btn_choose_nav)
        script_layout_2.addWidget(QLabel("RViz:"))
        script_layout_2.addWidget(self.rviz_config_edit, 2)
        script_layout_2.addWidget(self.btn_choose_rviz)
        process_layout.addLayout(script_layout_2)

        btn_layout = QHBoxLayout()
        self.btn_start_mapping = QPushButton("启动建图 mapping.sh")
        self.btn_stop_mapping = QPushButton("结束建图进程")
        self.btn_save_map = QPushButton("保存地图 save_map.sh")
        self.btn_finish_mapping_save = QPushButton("结束建图并保存地图")
        self.btn_start_nav = QPushButton("启动导航 nav_final.sh")
        self.btn_stop_nav = QPushButton("停止导航")
        self.btn_start_rviz = QPushButton("打开 RViz 可视化导航")
        self.btn_stop_rviz = QPushButton("关闭 RViz")

        btn_layout.addWidget(self.btn_start_mapping)
        btn_layout.addWidget(self.btn_stop_mapping)
        btn_layout.addWidget(self.btn_save_map)
        btn_layout.addWidget(self.btn_finish_mapping_save)
        btn_layout.addWidget(self.btn_start_nav)
        btn_layout.addWidget(self.btn_stop_nav)
        btn_layout.addWidget(self.btn_start_rviz)
        btn_layout.addWidget(self.btn_stop_rviz)
        process_layout.addLayout(btn_layout)

        status_layout = QHBoxLayout()
        self.mapping_status = QLabel("mapping: stopped")
        self.nav_status = QLabel("nav_final: stopped")
        self.rviz_status = QLabel("rviz2: stopped")
        status_layout.addWidget(self.mapping_status)
        status_layout.addWidget(self.nav_status)
        status_layout.addWidget(self.rviz_status)
        status_layout.addStretch()
        process_layout.addLayout(status_layout)

        root.addWidget(process_box)

        conn_box = QGroupBox("RPC 连接")
        conn_layout = QHBoxLayout(conn_box)

        self.host_edit = QLineEdit("127.0.0.1")
        self.port_edit = QLineEdit("6001")
        self.timeout_spin = QDoubleSpinBox()
        self.timeout_spin.setRange(1.0, 60.0)
        self.timeout_spin.setSingleStep(1.0)
        self.timeout_spin.setValue(10.0)

        conn_layout.addWidget(QLabel("Host:"))
        conn_layout.addWidget(self.host_edit)
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.port_edit)
        conn_layout.addWidget(QLabel("Timeout:"))
        conn_layout.addWidget(self.timeout_spin)

        self.btn_state = QPushButton("查看状态")
        self.btn_reload = QPushButton("重载地图")
        self.btn_recompute_icp = QPushButton("重新 ICP")
        self.btn_refresh = QPushButton("刷新物体列表")
        self.btn_labels = QPushButton("刷新标签")

        conn_layout.addWidget(self.btn_state)
        conn_layout.addWidget(self.btn_reload)
        conn_layout.addWidget(self.btn_recompute_icp)
        conn_layout.addWidget(self.btn_refresh)
        conn_layout.addWidget(self.btn_labels)
        root.addWidget(conn_box)

        filter_box = QGroupBox("筛选与导航参数")
        filter_layout = QHBoxLayout(filter_box)

        self.label_filter_edit = QLineEdit()
        self.label_filter_edit.setPlaceholderText("输入 label，例如 chair / cup / bottle")
        self.object_id_edit = QLineEdit()
        self.object_id_edit.setPlaceholderText("object_id，例如 81")
        self.approach_spin = QDoubleSpinBox()
        self.approach_spin.setRange(0.1, 5.0)
        self.approach_spin.setSingleStep(0.1)
        self.approach_spin.setValue(0.8)

        self.btn_find_label = QPushButton("按 Label 查询")
        self.btn_get_object = QPushButton("按 ID 查询")
        self.btn_show_cloud = QPushButton("显示选中物体点云")
        self.btn_candidates = QPushButton("查看候选点")
        self.btn_goto_object = QPushButton("导航到选中物体")
        self.btn_goto_label = QPushButton("导航到 Label 最佳物体")

        filter_layout.addWidget(QLabel("Label:"))
        filter_layout.addWidget(self.label_filter_edit)
        filter_layout.addWidget(QLabel("Object ID:"))
        filter_layout.addWidget(self.object_id_edit)
        filter_layout.addWidget(QLabel("靠近距离:"))
        filter_layout.addWidget(self.approach_spin)
        filter_layout.addWidget(self.btn_find_label)
        filter_layout.addWidget(self.btn_get_object)
        filter_layout.addWidget(self.btn_show_cloud)
        filter_layout.addWidget(self.btn_candidates)
        filter_layout.addWidget(self.btn_goto_object)
        filter_layout.addWidget(self.btn_goto_label)
        root.addWidget(filter_box)

        splitter = QSplitter(Qt.Horizontal)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        self.table = QTableWidget(0, 9)
        self.table.setHorizontalHeaderLabels([
            "object_id", "label", "class_id", "num_points", "center_x", "center_y", "center_z", "extent_xy", "extent_z",
        ])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSortingEnabled(True)
        left_layout.addWidget(QLabel("语义物体列表：点击一行后，可显示点云或导航"))
        left_layout.addWidget(self.table)

        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        info_box = QGroupBox("当前选中物体")
        form = QFormLayout(info_box)
        self.selected_id_label = QLabel("-")
        self.selected_label_label = QLabel("-")
        self.selected_class_label = QLabel("-")
        self.selected_points_label = QLabel("-")
        self.selected_center_label = QLabel("-")
        self.selected_extent_label = QLabel("-")
        form.addRow("object_id:", self.selected_id_label)
        form.addRow("label:", self.selected_label_label)
        form.addRow("class_id:", self.selected_class_label)
        form.addRow("num_points:", self.selected_points_label)
        form.addRow("center:", self.selected_center_label)
        form.addRow("extent:", self.selected_extent_label)
        right_layout.addWidget(info_box)
        terminal_box = QGroupBox("终端调试")
        terminal_layout = QVBoxLayout(terminal_box)

        terminal_tip = QLabel(
            "在这里输入 Linux/ROS2 命令进行调试。按 Enter 或点击执行。"
            "默认在工作空间目录执行。"
        )
        terminal_layout.addWidget(terminal_tip)

        terminal_cmd_layout = QHBoxLayout()
        self.terminal_cmd_edit = QLineEdit()
        self.terminal_cmd_edit.setPlaceholderText(
            "例如：ros2 topic list / ros2 node list / ros2 topic echo /scan --once"
        )
        self.terminal_source_check = QCheckBox("执行前 source install/setup.bash")
        self.terminal_source_check.setChecked(True)
        self.btn_terminal_run = QPushButton("执行命令")
        self.btn_terminal_stop = QPushButton("停止命令")
        self.btn_terminal_clear = QPushButton("清空输出")

        terminal_cmd_layout.addWidget(QLabel("$"))
        terminal_cmd_layout.addWidget(self.terminal_cmd_edit, 1)
        terminal_cmd_layout.addWidget(self.terminal_source_check)
        terminal_cmd_layout.addWidget(self.btn_terminal_run)
        terminal_cmd_layout.addWidget(self.btn_terminal_stop)
        terminal_cmd_layout.addWidget(self.btn_terminal_clear)
        terminal_layout.addLayout(terminal_cmd_layout)

        right_layout.addWidget(terminal_box)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        right_layout.addWidget(QLabel("运行日志 / RPC 返回 / shell 输出："))
        right_layout.addWidget(self.log)
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setSizes([920, 530])
        root.addWidget(splitter)

        self.btn_choose_ws.clicked.connect(self.choose_ws_dir)
        self.btn_choose_mapping.clicked.connect(lambda: self.choose_file(self.mapping_script_edit, "选择 mapping.sh"))
        self.btn_choose_save.clicked.connect(lambda: self.choose_file(self.save_map_script_edit, "选择 save_map.sh"))
        self.btn_choose_nav.clicked.connect(lambda: self.choose_file(self.nav_script_edit, "选择 nav_final.sh"))
        self.btn_choose_rviz.clicked.connect(lambda: self.choose_file(self.rviz_config_edit, "选择 RViz 配置文件"))
        self.btn_start_mapping.clicked.connect(self.on_start_mapping)
        self.btn_stop_mapping.clicked.connect(self.on_stop_mapping)
        self.btn_save_map.clicked.connect(self.on_save_map)
        self.btn_finish_mapping_save.clicked.connect(self.on_finish_mapping_and_save)
        self.btn_start_nav.clicked.connect(self.on_start_nav)
        self.btn_stop_nav.clicked.connect(self.on_stop_nav)
        self.btn_start_rviz.clicked.connect(self.on_start_rviz)
        self.btn_stop_rviz.clicked.connect(self.on_stop_rviz)

        self.btn_terminal_run.clicked.connect(self.on_terminal_run)
        self.btn_terminal_stop.clicked.connect(self.on_terminal_stop)
        self.btn_terminal_clear.clicked.connect(self.log.clear)
        self.terminal_cmd_edit.returnPressed.connect(self.on_terminal_run)

        self.btn_state.clicked.connect(self.on_state)
        self.btn_reload.clicked.connect(self.on_reload)
        self.btn_recompute_icp.clicked.connect(self.on_recompute_icp)
        self.btn_refresh.clicked.connect(self.on_refresh)
        self.btn_labels.clicked.connect(self.on_labels)
        self.btn_find_label.clicked.connect(self.on_find_label)
        self.btn_get_object.clicked.connect(self.on_get_object)
        self.btn_show_cloud.clicked.connect(self.on_show_cloud)
        self.btn_candidates.clicked.connect(self.on_candidates)
        self.btn_goto_object.clicked.connect(self.on_goto_object)
        self.btn_goto_label.clicked.connect(self.on_goto_label)
        self.table.itemSelectionChanged.connect(self.on_table_selection_changed)

    def choose_ws_dir(self):
        path = QFileDialog.getExistingDirectory(self, "选择 ROS2 工作空间", self.ws_dir_edit.text().strip() or os.path.expanduser("~"))
        if path:
            self.ws_dir_edit.setText(path)
            self.mapping_script_edit.setText(os.path.join(path, "mapping.sh"))
            self.save_map_script_edit.setText(os.path.join(path, "save_map.sh"))
            self.nav_script_edit.setText(os.path.join(path, "nav_final.sh"))

    def choose_file(self, edit, title):
        path, _ = QFileDialog.getOpenFileName(self, title, self.ws_dir_edit.text().strip() or os.path.expanduser("~"), "All files (*)")
        if path:
            edit.setText(path)

    def script_cmd(self, script_path):
        script_path = os.path.expanduser(script_path.strip())
        if not os.path.exists(script_path):
            raise RuntimeError(f"脚本不存在: {script_path}")
        return f"cd {self.shell_quote(self.ws_dir_edit.text().strip())} && bash {self.shell_quote(script_path)}"

    @staticmethod
    def shell_quote(s):
        return "'" + s.replace("'", "'\\''") + "'"

    def on_start_mapping(self):
        try:
            self.proc_mapping.start(self.script_cmd(self.mapping_script_edit.text()), cwd=self.ws_dir_edit.text().strip())
        except Exception as e:
            QMessageBox.warning(self, "启动建图失败", str(e))

    def on_stop_mapping(self):
        self.proc_mapping.stop(signal.SIGINT)

    def on_save_map(self):
        try:
            cmd = self.script_cmd(self.save_map_script_edit.text())
            self.proc_extra.start(cmd, cwd=self.ws_dir_edit.text().strip())
        except Exception as e:
            QMessageBox.warning(self, "保存地图失败", str(e))

    def on_finish_mapping_and_save(self):
        ret = QMessageBox.question(
            self,
            "确认结束建图并保存地图",
            "将先停止 mapping.sh 对应的 ROS launch，然后调用 save_map.sh 保存地图。确认执行？",
            QMessageBox.Yes | QMessageBox.No,
        )
        if ret != QMessageBox.Yes:
            return
        self.on_stop_mapping()
        # map_saver 需要 map_server/slam_toolbox 的 /map 仍可用。若停止后保存失败，改用“保存地图”按钮在停止前执行。
        time.sleep(1.0)
        self.on_save_map()

    def on_start_nav(self):
        try:
            self.proc_nav.start(self.script_cmd(self.nav_script_edit.text()), cwd=self.ws_dir_edit.text().strip())
            # 同时打开 RViz，满足“调用 nav_final 的时候有可视化导航界面”
            if not self.proc_rviz.is_running():
                self.on_start_rviz(show_warning=False)
        except Exception as e:
            QMessageBox.warning(self, "启动导航失败", str(e))

    def on_stop_nav(self):
        self.proc_nav.stop(signal.SIGINT)

    def on_start_rviz(self, show_warning=True):
        try:
            rviz_config = self.rviz_config_edit.text().strip()
            if rviz_config:
                if not os.path.exists(os.path.expanduser(rviz_config)):
                    raise RuntimeError(f"RViz 配置文件不存在: {rviz_config}")
                cmd = (
                    f"cd {self.shell_quote(self.ws_dir_edit.text().strip())} && "
                    f"source install/setup.bash && rviz2 -d {self.shell_quote(os.path.expanduser(rviz_config))}"
                )
            else:
                cmd = f"cd {self.shell_quote(self.ws_dir_edit.text().strip())} && source install/setup.bash && rviz2"
            self.proc_rviz.start(cmd, cwd=self.ws_dir_edit.text().strip())
        except Exception as e:
            if show_warning:
                QMessageBox.warning(self, "启动 RViz 失败", str(e))
            else:
                self.append_log(f"自动打开 RViz 失败: {e}")

    def on_stop_rviz(self):
        self.proc_rviz.stop(signal.SIGINT)

    def on_terminal_run(self):
        try:
            raw_cmd = self.terminal_cmd_edit.text().strip()
            if not raw_cmd:
                QMessageBox.warning(self, "命令为空", "请输入要执行的命令，例如：ros2 topic list")
                return

            ws_dir = os.path.expanduser(self.ws_dir_edit.text().strip())
            if not os.path.isdir(ws_dir):
                raise RuntimeError(f"工作空间目录不存在: {ws_dir}")

            if self.proc_terminal.is_running():
                QMessageBox.warning(self, "命令正在运行", "当前已有终端命令在运行，请先停止或等待结束。")
                return

            if self.terminal_source_check.isChecked():
                cmd = (
                    f"cd {self.shell_quote(ws_dir)} && "
                    f"source install/setup.bash && "
                    f"{raw_cmd}"
                )
            else:
                cmd = f"cd {self.shell_quote(ws_dir)} && {raw_cmd}"

            self.append_log(f"\n===== 终端调试命令 =====\n$ {raw_cmd}")
            self.proc_terminal.start(cmd, cwd=ws_dir)

        except Exception as e:
            QMessageBox.warning(self, "执行命令失败", str(e))

    def on_terminal_stop(self):
        self.proc_terminal.stop(signal.SIGINT)

    def on_process_state(self, name, state):
        text = f"{name}: {state}"
        if name == "mapping":
            self.mapping_status.setText(text)
        elif name == "nav_final":
            self.nav_status.setText(text)
        elif name == "rviz2":
            self.rviz_status.setText(text)

    def closeEvent(self, event):
        for proc in [self.proc_mapping, self.proc_nav, self.proc_rviz, self.proc_extra, self.proc_terminal]:
            if proc.is_running():
                proc.stop(signal.SIGINT, timeout=2.0)
        event.accept()

    def rpc_config(self):
        return self.host_edit.text().strip(), int(self.port_edit.text().strip()), float(self.timeout_spin.value())

    def append_log(self, text):
        self.log.append(str(text))

    def append_json(self, title, data):
        self.append_log(f"===== {title} =====\n" + json.dumps(data, ensure_ascii=False, indent=2) + "\n")

    def run_rpc(self, command, args=None):
        host, port, timeout = self.rpc_config()
        worker = RpcWorker(host, port, timeout, command, args=args)
        worker.done.connect(self.on_rpc_done)
        worker.failed.connect(self.on_rpc_failed)
        self.workers.append(worker)
        worker.start()

    def get_selected_object_id(self):
        if self.current_object is not None:
            return int(self.current_object["object_id"])
        text = self.object_id_edit.text().strip()
        if text:
            return int(text)
        raise RuntimeError("请先在表格中选择物体，或手动输入 object_id")

    def get_label_text(self):
        label = self.label_filter_edit.text().strip()
        if not label:
            raise RuntimeError("请输入 label，例如 chair / cup")
        return label

    def get_approach_distance(self):
        return float(self.approach_spin.value())

    def on_state(self): self.run_rpc("state")
    def on_reload(self): self.run_rpc("reload")
    def on_recompute_icp(self): self.run_rpc("recompute_icp")
    def on_refresh(self): self.run_rpc("list")
    def on_labels(self): self.run_rpc("labels")

    def on_find_label(self):
        try:
            self.run_rpc("find_label", {"label": self.get_label_text()})
        except Exception as e:
            QMessageBox.warning(self, "输入错误", str(e))

    def on_get_object(self):
        try:
            self.run_rpc("get_object", {"object_id": int(self.object_id_edit.text().strip())})
        except Exception as e:
            QMessageBox.warning(self, "输入错误", str(e))

    def on_show_cloud(self):
        try:
            self.run_rpc("publish_cloud", {"object_id": self.get_selected_object_id()})
        except Exception as e:
            QMessageBox.warning(self, "输入错误", str(e))

    def on_candidates(self):
        try:
            self.run_rpc("candidates", {"object_id": self.get_selected_object_id(), "approach_distance": self.get_approach_distance()})
        except Exception as e:
            QMessageBox.warning(self, "输入错误", str(e))

    def on_goto_object(self):
        try:
            oid = self.get_selected_object_id()
            ret = QMessageBox.question(self, "确认导航", f"确认导航到 object_id={oid} 附近？\n\n请确认机器人周围安全。", QMessageBox.Yes | QMessageBox.No)
            if ret == QMessageBox.Yes:
                self.run_rpc("goto_object", {"object_id": oid, "approach_distance": self.get_approach_distance()})
        except Exception as e:
            QMessageBox.warning(self, "输入错误", str(e))

    def on_goto_label(self):
        try:
            label = self.get_label_text()
            ret = QMessageBox.question(self, "确认导航", f"确认导航到 label={label} 的最佳物体附近？\n\n请确认机器人周围安全。", QMessageBox.Yes | QMessageBox.No)
            if ret == QMessageBox.Yes:
                self.run_rpc("goto_label", {"label": label, "approach_distance": self.get_approach_distance()})
        except Exception as e:
            QMessageBox.warning(self, "输入错误", str(e))

    def on_rpc_done(self, command, result):
        self.append_json(command, result)
        if command in ["list", "find_label"] and isinstance(result, list):
            self.objects = result
            self.populate_table(result)
        elif command == "get_object" and isinstance(result, dict):
            self.objects = [result]
            self.populate_table([result])
        elif command == "labels":
            self.current_label_table = result
            if isinstance(result, list):
                label_summary = [f"{item.get('label')}  objects={item.get('num_objects')}  ids={item.get('object_ids')}" for item in result]
                self.append_log("可用标签：\n" + "\n".join(label_summary))
        elif command == "publish_cloud" and isinstance(result, dict) and result.get("ok"):
            self.append_log(f"已发布目标物体点云到 RViz 话题：{result.get('topic')}\nobject_id={result.get('object_id')}, label={result.get('label')}, points={result.get('num_points')}")
        elif command == "goto_object" and isinstance(result, dict) and result.get("ok"):
            goal = result.get("goal", {})
            cloud = result.get("target_object_cloud", {})
            self.append_log(f"导航请求已发送。\ngoal=({goal.get('x')}, {goal.get('y')}, {goal.get('yaw')})\ntarget_cloud={cloud}")
        elif command == "goto_label" and isinstance(result, dict) and result.get("ok"):
            obj = result.get("object", {})
            goal = result.get("goal", {})
            self.append_log(f"导航请求已发送。\nselected object_id={obj.get('object_id')}, label={obj.get('label')}\ngoal=({goal.get('x')}, {goal.get('y')}, {goal.get('yaw')})")

    def on_rpc_failed(self, command, error_text):
        self.append_log(f"===== {command} FAILED =====\n{error_text}")
        QMessageBox.critical(self, "RPC 调用失败", error_text)

    def populate_table(self, objects):
        self.table.setSortingEnabled(False)
        self.table.setRowCount(0)
        for obj in objects:
            row = self.table.rowCount()
            self.table.insertRow(row)
            center = obj.get("center", [0, 0, 0])
            extent = obj.get("extent", [0, 0, 0])
            extent_xy = (float(extent[0]) ** 2 + float(extent[1]) ** 2) ** 0.5
            values = [
                obj.get("object_id", ""), obj.get("label", ""), obj.get("class_id", ""), obj.get("num_points", ""),
                f"{float(center[0]):.3f}", f"{float(center[1]):.3f}", f"{float(center[2]):.3f}", f"{extent_xy:.3f}", f"{float(extent[2]):.3f}",
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                if col in [0, 2, 3]:
                    try:
                        item.setData(Qt.UserRole, int(value))
                    except Exception:
                        pass
                self.table.setItem(row, col, item)
        self.table.setSortingEnabled(True)

    def on_table_selection_changed(self):
        selected = self.table.selectedItems()
        if not selected:
            return
        row = selected[0].row()
        oid_item = self.table.item(row, 0)
        if oid_item is None:
            return
        oid = int(oid_item.text())
        for obj in self.objects:
            if int(obj.get("object_id")) == oid:
                self.current_object = obj
                self.object_id_edit.setText(str(oid))
                self.update_selected_object_panel(obj)
                return

    def update_selected_object_panel(self, obj):
        center = obj.get("center", [0, 0, 0])
        extent = obj.get("extent", [0, 0, 0])
        self.selected_id_label.setText(str(obj.get("object_id")))
        self.selected_label_label.setText(str(obj.get("label")))
        self.selected_class_label.setText(str(obj.get("class_id")))
        self.selected_points_label.setText(str(obj.get("num_points")))
        self.selected_center_label.setText(f"[{float(center[0]):.3f}, {float(center[1]):.3f}, {float(center[2]):.3f}]")
        self.selected_extent_label.setText(f"[{float(extent[0]):.3f}, {float(extent[1]):.3f}, {float(extent[2]):.3f}]")


def main():
    app = QApplication(sys.argv)
    win = SemanticNavGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
