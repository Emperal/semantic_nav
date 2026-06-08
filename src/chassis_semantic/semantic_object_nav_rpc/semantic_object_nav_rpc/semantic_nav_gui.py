#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import socket
import sys
import traceback

from bsonrpc import JSONRpc

from PyQt5.QtCore import Qt, QThread, pyqtSignal
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
)


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
        self.resize(1300, 760)

        self.objects = []
        self.current_object = None
        self.current_label_table = []

        self.workers = []

        self.build_ui()

    def build_ui(self):
        root = QVBoxLayout(self)

        # ============================================================
        # Connection config
        # ============================================================
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

        # ============================================================
        # Filter controls
        # ============================================================
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

        # ============================================================
        # Main splitter
        # ============================================================
        splitter = QSplitter(Qt.Horizontal)

        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)

        self.table = QTableWidget(0, 9)
        self.table.setHorizontalHeaderLabels([
            "object_id",
            "label",
            "class_id",
            "num_points",
            "center_x",
            "center_y",
            "center_z",
            "extent_xy",
            "extent_z",
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

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        right_layout.addWidget(QLabel("运行日志 / RPC 返回："))
        right_layout.addWidget(self.log)

        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setSizes([850, 450])

        root.addWidget(splitter)

        # ============================================================
        # Signals
        # ============================================================
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

    # ============================================================
    # Helpers
    # ============================================================

    def rpc_config(self):
        return (
            self.host_edit.text().strip(),
            int(self.port_edit.text().strip()),
            float(self.timeout_spin.value()),
        )

    def append_log(self, text):
        self.log.append(text)
        self.log.append("")

    def append_json(self, title, data):
        self.append_log(f"===== {title} =====\n" + json.dumps(data, ensure_ascii=False, indent=2))

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

    # ============================================================
    # Button callbacks
    # ============================================================

    def on_state(self):
        self.run_rpc("state")

    def on_reload(self):
        self.run_rpc("reload")

    def on_recompute_icp(self):
        self.run_rpc("recompute_icp")

    def on_refresh(self):
        self.run_rpc("list")

    def on_labels(self):
        self.run_rpc("labels")

    def on_find_label(self):
        try:
            label = self.get_label_text()
            self.run_rpc("find_label", {"label": label})
        except Exception as e:
            QMessageBox.warning(self, "输入错误", str(e))

    def on_get_object(self):
        try:
            oid = int(self.object_id_edit.text().strip())
            self.run_rpc("get_object", {"object_id": oid})
        except Exception as e:
            QMessageBox.warning(self, "输入错误", str(e))

    def on_show_cloud(self):
        try:
            oid = self.get_selected_object_id()
            self.run_rpc("publish_cloud", {"object_id": oid})
        except Exception as e:
            QMessageBox.warning(self, "输入错误", str(e))

    def on_candidates(self):
        try:
            oid = self.get_selected_object_id()
            self.run_rpc(
                "candidates",
                {
                    "object_id": oid,
                    "approach_distance": self.get_approach_distance(),
                },
            )
        except Exception as e:
            QMessageBox.warning(self, "输入错误", str(e))

    def on_goto_object(self):
        try:
            oid = self.get_selected_object_id()
            ret = QMessageBox.question(
                self,
                "确认导航",
                f"确认导航到 object_id={oid} 附近？\n\n请确认机器人周围安全。",
                QMessageBox.Yes | QMessageBox.No,
            )
            if ret != QMessageBox.Yes:
                return

            self.run_rpc(
                "goto_object",
                {
                    "object_id": oid,
                    "approach_distance": self.get_approach_distance(),
                },
            )
        except Exception as e:
            QMessageBox.warning(self, "输入错误", str(e))

    def on_goto_label(self):
        try:
            label = self.get_label_text()
            ret = QMessageBox.question(
                self,
                "确认导航",
                f"确认导航到 label={label} 的最佳物体附近？\n\n请确认机器人周围安全。",
                QMessageBox.Yes | QMessageBox.No,
            )
            if ret != QMessageBox.Yes:
                return

            self.run_rpc(
                "goto_label",
                {
                    "label": label,
                    "approach_distance": self.get_approach_distance(),
                },
            )
        except Exception as e:
            QMessageBox.warning(self, "输入错误", str(e))

    # ============================================================
    # RPC results
    # ============================================================

    def on_rpc_done(self, command, result):
        self.append_json(command, result)

        if command in ["list", "find_label"]:
            if isinstance(result, list):
                self.objects = result
                self.populate_table(result)

        elif command == "get_object":
            if isinstance(result, dict):
                self.objects = [result]
                self.populate_table([result])

        elif command == "labels":
            self.current_label_table = result
            label_summary = []
            if isinstance(result, list):
                for item in result:
                    label_summary.append(
                        f"{item.get('label')}  objects={item.get('num_objects')}  ids={item.get('object_ids')}"
                    )
            self.append_log("可用标签：\n" + "\n".join(label_summary))

        elif command == "publish_cloud":
            if isinstance(result, dict) and result.get("ok"):
                self.append_log(
                    f"已发布目标物体点云到 RViz 话题：{result.get('topic')}\n"
                    f"object_id={result.get('object_id')}, label={result.get('label')}, "
                    f"points={result.get('num_points')}"
                )

        elif command == "goto_object":
            if isinstance(result, dict) and result.get("ok"):
                goal = result.get("goal", {})
                cloud = result.get("target_object_cloud", {})
                self.append_log(
                    "导航请求已发送。\n"
                    f"goal=({goal.get('x')}, {goal.get('y')}, {goal.get('yaw')})\n"
                    f"target_cloud={cloud}"
                )

        elif command == "goto_label":
            if isinstance(result, dict) and result.get("ok"):
                obj = result.get("object", {})
                goal = result.get("goal", {})
                self.append_log(
                    "导航请求已发送。\n"
                    f"selected object_id={obj.get('object_id')}, label={obj.get('label')}\n"
                    f"goal=({goal.get('x')}, {goal.get('y')}, {goal.get('yaw')})"
                )

    def on_rpc_failed(self, command, error_text):
        self.append_log(f"===== {command} FAILED =====\n{error_text}")
        QMessageBox.critical(self, "RPC 调用失败", error_text)

    # ============================================================
    # Table
    # ============================================================

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
                obj.get("object_id", ""),
                obj.get("label", ""),
                obj.get("class_id", ""),
                obj.get("num_points", ""),
                f"{float(center[0]):.3f}",
                f"{float(center[1]):.3f}",
                f"{float(center[2]):.3f}",
                f"{extent_xy:.3f}",
                f"{float(extent[2]):.3f}",
            ]

            for col, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                if col in [0, 2, 3]:
                    item.setData(Qt.UserRole, int(value))
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
        self.selected_center_label.setText(
            f"[{float(center[0]):.3f}, {float(center[1]):.3f}, {float(center[2]):.3f}]"
        )
        self.selected_extent_label.setText(
            f"[{float(extent[0]):.3f}, {float(extent[1]):.3f}, {float(extent[2]):.3f}]"
        )


def main():
    app = QApplication(sys.argv)
    win = SemanticNavGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()