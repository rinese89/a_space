#!/usr/bin/env python3
"""
ROS 2 rosbag odometry plotter with graphical interface.

Supported message types:
  - nav_msgs/msg/Odometry
  - px4_msgs/msg/VehicleOdometry

Features:
  - Select a rosbag2 directory, metadata.yaml, or .db3 file.
  - Automatically discover supported odometry topics.
  - Select one or more topics.
  - Select any variable for the X axis.
  - Select one or more variables for the Y axis.
  - Plot time series, XY trajectories, X vs Y, velocity relationships, etc.
  - Embedded Matplotlib plot with navigation toolbar.
  - Save the current figure as PNG/PDF/SVG.

ROS 2 Humble example:
    source /opt/ros/humble/setup.bash
    source ~/a_space_ws/install/setup.bash
    python3 rosbag_odometry_gui.py
"""

from __future__ import annotations

import math
import sys
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import rosbag2_py
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


NAV_ODOM_TYPE = "nav_msgs/msg/Odometry"
PX4_ODOM_TYPE = "px4_msgs/msg/VehicleOdometry"
SUPPORTED_TYPES = {NAV_ODOM_TYPE, PX4_ODOM_TYPE}


@dataclass(frozen=True)
class VariableSpec:
    key: str
    label: str
    unit: str


VARIABLES: List[VariableSpec] = [
    VariableSpec("time_s", "Tiempo", "s"),
    VariableSpec("x", "X", "m"),
    VariableSpec("y", "Y", "m"),
    VariableSpec("z", "Z", "m"),
    VariableSpec("vx", "Vx", "m/s"),
    VariableSpec("vy", "Vy", "m/s"),
    VariableSpec("vz", "Vz", "m/s"),
    VariableSpec("speed", "Velocidad |v|", "m/s"),
    VariableSpec("roll_deg", "Roll", "deg"),
    VariableSpec("pitch_deg", "Pitch", "deg"),
    VariableSpec("yaw_deg", "Yaw", "deg"),
    VariableSpec("wx", "Wx", "rad/s"),
    VariableSpec("wy", "Wy", "rad/s"),
    VariableSpec("wz", "Wz", "rad/s"),
    VariableSpec("angular_speed", "Velocidad angular |w|", "rad/s"),
]

VARIABLE_BY_LABEL = {spec.label: spec for spec in VARIABLES}
VARIABLE_BY_KEY = {spec.key: spec for spec in VARIABLES}


@dataclass
class OdomSeries:
    topic: str
    msg_type: str

    bag_time_ns: List[int] = field(default_factory=list)
    time_s: List[float] = field(default_factory=list)

    x: List[float] = field(default_factory=list)
    y: List[float] = field(default_factory=list)
    z: List[float] = field(default_factory=list)

    vx: List[float] = field(default_factory=list)
    vy: List[float] = field(default_factory=list)
    vz: List[float] = field(default_factory=list)

    roll_deg: List[float] = field(default_factory=list)
    pitch_deg: List[float] = field(default_factory=list)
    yaw_deg: List[float] = field(default_factory=list)

    wx: List[float] = field(default_factory=list)
    wy: List[float] = field(default_factory=list)
    wz: List[float] = field(default_factory=list)

    speed: List[float] = field(default_factory=list)
    angular_speed: List[float] = field(default_factory=list)

    def append(
        self,
        bag_time_ns: int,
        position: Sequence[float],
        velocity: Sequence[float],
        rpy_rad: Sequence[float],
        angular_velocity: Sequence[float],
    ) -> None:
        self.bag_time_ns.append(int(bag_time_ns))

        px, py, pz = map(float, position)
        vx, vy, vz = map(float, velocity)
        wx, wy, wz = map(float, angular_velocity)

        self.x.append(px)
        self.y.append(py)
        self.z.append(pz)

        self.vx.append(vx)
        self.vy.append(vy)
        self.vz.append(vz)
        self.speed.append(math.sqrt(vx * vx + vy * vy + vz * vz))

        self.roll_deg.append(math.degrees(float(rpy_rad[0])))
        self.pitch_deg.append(math.degrees(float(rpy_rad[1])))
        self.yaw_deg.append(math.degrees(float(rpy_rad[2])))

        self.wx.append(wx)
        self.wy.append(wy)
        self.wz.append(wz)
        self.angular_speed.append(math.sqrt(wx * wx + wy * wy + wz * wz))

    def finalise_time(self, t0_ns: int) -> None:
        self.time_s = [(t - t0_ns) * 1.0e-9 for t in self.bag_time_ns]

    def values(self, key: str) -> List[float]:
        return getattr(self, key)


def quaternion_xyzw_to_rpy(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def decode_nav_odometry(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    v = msg.twist.twist.linear
    w = msg.twist.twist.angular

    position = (p.x, p.y, p.z)
    velocity = (v.x, v.y, v.z)
    rpy = quaternion_xyzw_to_rpy(q.x, q.y, q.z, q.w)
    angular_velocity = (w.x, w.y, w.z)

    return position, velocity, rpy, angular_velocity


def decode_px4_odometry(msg):
    # px4_msgs/msg/VehicleOdometry.q uses Hamilton order [w, x, y, z].
    position = tuple(float(v) for v in msg.position[:3])
    velocity = tuple(float(v) for v in msg.velocity[:3])

    qw = float(msg.q[0])
    qx = float(msg.q[1])
    qy = float(msg.q[2])
    qz = float(msg.q[3])
    rpy = quaternion_xyzw_to_rpy(qx, qy, qz, qw)

    angular_velocity = tuple(float(v) for v in msg.angular_velocity[:3])

    return position, velocity, rpy, angular_velocity


def normalise_bag_path(selected: str) -> Path:
    path = Path(selected).expanduser().resolve()

    if path.is_dir():
        return path

    if path.is_file() and path.name == "metadata.yaml":
        return path.parent

    if path.is_file() and path.suffix == ".db3":
        return path.parent

    raise ValueError(
        "Selecciona el directorio del rosbag, metadata.yaml o un fichero .db3."
    )


def create_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id="sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def topic_types(bag_path: Path) -> Dict[str, str]:
    reader = create_reader(bag_path)
    return {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }


def short_topic_name(topic: str) -> str:
    parts = [part for part in topic.split("/") if part]

    if parts and parts[-1] == "odom" and len(parts) >= 2:
        return parts[-2]

    if parts and parts[-1] == "vehicle_odometry":
        return parts[0]

    return topic


class RosbagOdometryGUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("ROS 2 Rosbag Odometry Plotter")
        self.root.geometry("1450x900")
        self.root.minsize(1100, 700)

        self.bag_path: Optional[Path] = None
        self.available_topics: Dict[str, str] = {}
        self.loaded_data: Dict[str, OdomSeries] = {}

        self.bag_var = tk.StringVar()
        self.x_var = tk.StringVar(value="Tiempo")
        self.plot_mode_var = tk.StringVar(value="Línea")
        self.equal_axis_var = tk.BooleanVar(value=False)
        self.grid_var = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar(
            value="Selecciona un rosbag para comenzar."
        )

        self._build_ui()
        self._create_empty_plot()

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=8)
        outer.pack(fill=tk.BOTH, expand=True)

        controls = ttk.Frame(outer)
        controls.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        plot_frame = ttk.Frame(outer)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # --------------------------------------------------------------
        # Rosbag selector
        # --------------------------------------------------------------
        bag_box = ttk.LabelFrame(
            controls,
            text="1. Rosbag",
            padding=8,
        )
        bag_box.pack(fill=tk.X, pady=(0, 8))

        ttk.Entry(
            bag_box,
            textvariable=self.bag_var,
            width=46,
        ).grid(
            row=0,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(0, 6),
        )

        ttk.Button(
            bag_box,
            text="Seleccionar carpeta",
            command=self.select_bag_directory,
        ).grid(row=1, column=0, sticky="ew", padx=(0, 4))

        ttk.Button(
            bag_box,
            text="Seleccionar metadata/.db3",
            command=self.select_bag_file,
        ).grid(row=1, column=1, sticky="ew", padx=(4, 0))

        ttk.Button(
            bag_box,
            text="Cargar tópicos",
            command=self.load_topic_list,
        ).grid(
            row=2,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(6, 0),
        )

        bag_box.columnconfigure(0, weight=1)
        bag_box.columnconfigure(1, weight=1)

        # --------------------------------------------------------------
        # Topic selector
        # --------------------------------------------------------------
        topic_box = ttk.LabelFrame(
            controls,
            text="2. Tópicos de odometría",
            padding=8,
        )
        topic_box.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

        list_frame = ttk.Frame(topic_box)
        list_frame.pack(fill=tk.BOTH, expand=True)

        self.topic_list = tk.Listbox(
            list_frame,
            selectmode=tk.EXTENDED,
            exportselection=False,
            height=12,
            width=54,
        )
        topic_scroll = ttk.Scrollbar(
            list_frame,
            orient=tk.VERTICAL,
            command=self.topic_list.yview,
        )
        self.topic_list.configure(yscrollcommand=topic_scroll.set)

        self.topic_list.pack(
            side=tk.LEFT,
            fill=tk.BOTH,
            expand=True,
        )
        topic_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        topic_buttons = ttk.Frame(topic_box)
        topic_buttons.pack(fill=tk.X, pady=(6, 0))

        ttk.Button(
            topic_buttons,
            text="Seleccionar todos",
            command=self.select_all_topics,
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 3))

        ttk.Button(
            topic_buttons,
            text="Deseleccionar",
            command=lambda: self.topic_list.selection_clear(0, tk.END),
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(3, 0))

        # --------------------------------------------------------------
        # Variables
        # --------------------------------------------------------------
        variable_box = ttk.LabelFrame(
            controls,
            text="3. Variables",
            padding=8,
        )
        variable_box.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(
            variable_box,
            text="Eje X:",
        ).grid(row=0, column=0, sticky="w")

        self.x_combo = ttk.Combobox(
            variable_box,
            textvariable=self.x_var,
            values=[v.label for v in VARIABLES],
            state="readonly",
            width=28,
        )
        self.x_combo.grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(8, 0),
        )

        ttk.Label(
            variable_box,
            text="Eje Y:",
        ).grid(
            row=1,
            column=0,
            sticky="nw",
            pady=(8, 0),
        )

        y_frame = ttk.Frame(variable_box)
        y_frame.grid(
            row=1,
            column=1,
            sticky="nsew",
            padx=(8, 0),
            pady=(8, 0),
        )

        self.y_list = tk.Listbox(
            y_frame,
            selectmode=tk.EXTENDED,
            exportselection=False,
            height=8,
            width=28,
        )
        y_scroll = ttk.Scrollbar(
            y_frame,
            orient=tk.VERTICAL,
            command=self.y_list.yview,
        )
        self.y_list.configure(yscrollcommand=y_scroll.set)

        for spec in VARIABLES:
            if spec.key != "time_s":
                self.y_list.insert(tk.END, spec.label)

        self.y_list.selection_set(0)

        self.y_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        y_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        variable_box.columnconfigure(1, weight=1)

        # --------------------------------------------------------------
        # Plot options
        # --------------------------------------------------------------
        option_box = ttk.LabelFrame(
            controls,
            text="4. Gráfica",
            padding=8,
        )
        option_box.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(
            option_box,
            text="Modo:",
        ).grid(row=0, column=0, sticky="w")

        ttk.Combobox(
            option_box,
            textvariable=self.plot_mode_var,
            values=["Línea", "Dispersión"],
            state="readonly",
            width=18,
        ).grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(8, 0),
        )

        ttk.Checkbutton(
            option_box,
            text="Misma escala X/Y",
            variable=self.equal_axis_var,
        ).grid(
            row=1,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(6, 0),
        )

        ttk.Checkbutton(
            option_box,
            text="Mostrar rejilla",
            variable=self.grid_var,
        ).grid(
            row=2,
            column=0,
            columnspan=2,
            sticky="w",
        )

        ttk.Button(
            option_box,
            text="Cargar datos y graficar",
            command=self.plot_selected,
        ).grid(
            row=3,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(10, 0),
        )

        ttk.Button(
            option_box,
            text="Guardar gráfica",
            command=self.save_current_plot,
        ).grid(
            row=4,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(5, 0),
        )

        option_box.columnconfigure(1, weight=1)

        # --------------------------------------------------------------
        # Status
        # --------------------------------------------------------------
        status_box = ttk.LabelFrame(
            controls,
            text="Estado",
            padding=8,
        )
        status_box.pack(fill=tk.X)

        ttk.Label(
            status_box,
            textvariable=self.status_var,
            wraplength=390,
            justify=tk.LEFT,
        ).pack(fill=tk.X)

        # --------------------------------------------------------------
        # Matplotlib
        # --------------------------------------------------------------
        self.figure = plt.Figure(figsize=(9, 7), dpi=100)
        self.ax = self.figure.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(
            self.figure,
            master=plot_frame,
        )
        self.canvas.get_tk_widget().pack(
            fill=tk.BOTH,
            expand=True,
        )

        toolbar_frame = ttk.Frame(plot_frame)
        toolbar_frame.pack(fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(
            self.canvas,
            toolbar_frame,
            pack_toolbar=False,
        )
        self.toolbar.update()
        self.toolbar.pack(fill=tk.X)

    def _create_empty_plot(self) -> None:
        self.ax.clear()
        self.ax.set_title("Selecciona un rosbag y las variables a representar")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid(True)
        self.canvas.draw_idle()

    def select_bag_directory(self) -> None:
        selected = filedialog.askdirectory(
            title="Seleccionar directorio rosbag2"
        )
        if selected:
            self.bag_var.set(selected)
            self.load_topic_list()

    def select_bag_file(self) -> None:
        selected = filedialog.askopenfilename(
            title="Seleccionar metadata.yaml o .db3",
            filetypes=[
                ("ROS bag", ("*.yaml", "*.db3")),
                ("metadata.yaml", "metadata.yaml"),
                ("SQLite3 rosbag", "*.db3"),
                ("Todos los ficheros", "*"),
            ],
        )
        if selected:
            self.bag_var.set(selected)
            self.load_topic_list()

    def _resolve_bag(self) -> Path:
        raw = self.bag_var.get().strip()
        if not raw:
            raise ValueError("No se ha seleccionado ningún rosbag.")

        bag_path = normalise_bag_path(raw)

        if not (bag_path / "metadata.yaml").exists():
            raise ValueError(
                f"No se encuentra metadata.yaml en:\n{bag_path}"
            )

        return bag_path

    def load_topic_list(self) -> None:
        try:
            self.bag_path = self._resolve_bag()
            self.available_topics = topic_types(self.bag_path)
        except Exception as exc:
            messagebox.showerror(
                "Error al abrir rosbag",
                str(exc),
            )
            return

        supported = [
            (topic, msg_type)
            for topic, msg_type in self.available_topics.items()
            if msg_type in SUPPORTED_TYPES
        ]
        supported.sort(key=lambda item: item[0])

        self.topic_list.delete(0, tk.END)

        for topic, msg_type in supported:
            short_type = (
                "ROS Odometry"
                if msg_type == NAV_ODOM_TYPE
                else "PX4 VehicleOdometry"
            )
            self.topic_list.insert(
                tk.END,
                f"{topic}   [{short_type}]",
            )

        if supported:
            # Prefer ROS nav_msgs/Odometry by default.
            nav_indices = [
                i
                for i, (_, msg_type) in enumerate(supported)
                if msg_type == NAV_ODOM_TYPE
            ]

            if nav_indices:
                for i in nav_indices:
                    self.topic_list.selection_set(i)
            else:
                self.topic_list.selection_set(0, tk.END)

            self.status_var.set(
                f"Rosbag cargado: {self.bag_path}\n"
                f"{len(supported)} tópicos de odometría compatibles."
            )
        else:
            self.status_var.set(
                "El rosbag no contiene tópicos nav_msgs/Odometry "
                "ni px4_msgs/VehicleOdometry."
            )

        self.loaded_data.clear()

    def select_all_topics(self) -> None:
        if self.topic_list.size() > 0:
            self.topic_list.selection_set(0, tk.END)

    def _supported_topic_names(self) -> List[str]:
        return sorted(
            topic
            for topic, msg_type in self.available_topics.items()
            if msg_type in SUPPORTED_TYPES
        )

    def selected_topics(self) -> List[str]:
        topic_names = self._supported_topic_names()
        indices = self.topic_list.curselection()
        return [
            topic_names[index]
            for index in indices
            if index < len(topic_names)
        ]

    def selected_y_specs(self) -> List[VariableSpec]:
        labels = [
            self.y_list.get(index)
            for index in self.y_list.curselection()
        ]
        return [VARIABLE_BY_LABEL[label] for label in labels]

    def load_selected_data(
        self,
        topics: Sequence[str],
    ) -> Dict[str, OdomSeries]:
        if self.bag_path is None:
            raise RuntimeError("No hay ningún rosbag cargado.")

        reader = create_reader(self.bag_path)

        types = {
            item.name: item.type
            for item in reader.get_all_topics_and_types()
        }

        message_classes = {}
        for topic in topics:
            msg_type = types[topic]
            if msg_type not in message_classes:
                message_classes[msg_type] = get_message(msg_type)

        result = {
            topic: OdomSeries(
                topic=topic,
                msg_type=types[topic],
            )
            for topic in topics
        }

        selected_set = set(topics)

        self.status_var.set(
            "Leyendo mensajes del rosbag..."
        )
        self.root.update_idletasks()

        while reader.has_next():
            topic, raw, timestamp_ns = reader.read_next()

            if topic not in selected_set:
                continue

            msg_type = types[topic]
            msg = deserialize_message(
                raw,
                message_classes[msg_type],
            )

            if msg_type == NAV_ODOM_TYPE:
                position, velocity, rpy, angular_velocity = (
                    decode_nav_odometry(msg)
                )
            elif msg_type == PX4_ODOM_TYPE:
                position, velocity, rpy, angular_velocity = (
                    decode_px4_odometry(msg)
                )
            else:
                continue

            result[topic].append(
                bag_time_ns=timestamp_ns,
                position=position,
                velocity=velocity,
                rpy_rad=rpy,
                angular_velocity=angular_velocity,
            )

        nonempty = [
            s
            for s in result.values()
            if s.bag_time_ns
        ]

        if not nonempty:
            raise RuntimeError(
                "No se han encontrado mensajes en los tópicos seleccionados."
            )

        t0_ns = min(
            series.bag_time_ns[0]
            for series in nonempty
        )

        for series in result.values():
            series.finalise_time(t0_ns)

        return result

    def plot_selected(self) -> None:
        if self.bag_path is None:
            self.load_topic_list()

        if self.bag_path is None:
            return

        topics = self.selected_topics()
        if not topics:
            messagebox.showwarning(
                "Sin tópicos",
                "Selecciona al menos un tópico.",
            )
            return

        y_specs = self.selected_y_specs()
        if not y_specs:
            messagebox.showwarning(
                "Sin variable Y",
                "Selecciona al menos una variable para el eje Y.",
            )
            return

        x_label = self.x_var.get()
        if x_label not in VARIABLE_BY_LABEL:
            messagebox.showerror(
                "Variable X inválida",
                x_label,
            )
            return

        x_spec = VARIABLE_BY_LABEL[x_label]

        try:
            self.loaded_data = self.load_selected_data(topics)
        except Exception as exc:
            messagebox.showerror(
                "Error leyendo rosbag",
                str(exc),
            )
            return

        self.ax.clear()

        mode = self.plot_mode_var.get()

        total_curves = 0
        total_samples = 0

        for topic, series in self.loaded_data.items():
            if not series.time_s:
                continue

            x_values = series.values(x_spec.key)
            total_samples += len(x_values)

            for y_spec in y_specs:
                y_values = series.values(y_spec.key)

                label = short_topic_name(topic)
                if len(y_specs) > 1:
                    label += f" — {y_spec.label}"

                if mode == "Dispersión":
                    self.ax.scatter(
                        x_values,
                        y_values,
                        s=8,
                        label=label,
                    )
                else:
                    self.ax.plot(
                        x_values,
                        y_values,
                        label=label,
                    )

                total_curves += 1

        self.ax.set_xlabel(
            self._axis_text(x_spec)
        )

        if len(y_specs) == 1:
            self.ax.set_ylabel(
                self._axis_text(y_specs[0])
            )
        else:
            units = {spec.unit for spec in y_specs}
            if len(units) == 1:
                unit = next(iter(units))
                self.ax.set_ylabel(
                    f"Variables seleccionadas [{unit}]"
                    if unit
                    else "Variables seleccionadas"
                )
            else:
                self.ax.set_ylabel("Variables seleccionadas")

        title_y = ", ".join(spec.label for spec in y_specs)
        self.ax.set_title(
            f"{title_y} frente a {x_spec.label}"
        )

        self.ax.grid(self.grid_var.get())

        if self.equal_axis_var.get():
            self.ax.axis("equal")

        if total_curves > 0:
            self.ax.legend()

        self.figure.tight_layout()
        self.canvas.draw_idle()

        self.status_var.set(
            f"Gráfica generada: {total_curves} curva(s), "
            f"{len(self.loaded_data)} tópico(s).\n"
            f"Variables: X={x_spec.label}; "
            f"Y={', '.join(spec.label for spec in y_specs)}.\n"
            f"Muestras leídas: {total_samples}."
        )

    @staticmethod
    def _axis_text(spec: VariableSpec) -> str:
        if spec.unit:
            return f"{spec.label} [{spec.unit}]"
        return spec.label

    def save_current_plot(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Guardar gráfica",
            defaultextension=".png",
            filetypes=[
                ("PNG", "*.png"),
                ("PDF", "*.pdf"),
                ("SVG", "*.svg"),
                ("Todos los ficheros", "*"),
            ],
        )

        if not path:
            return

        try:
            self.figure.savefig(
                path,
                dpi=180,
                bbox_inches="tight",
            )
        except Exception as exc:
            messagebox.showerror(
                "Error guardando gráfica",
                str(exc),
            )
            return

        self.status_var.set(
            f"Gráfica guardada en:\n{path}"
        )


def main() -> None:
    root = tk.Tk()

    try:
        ttk.Style().theme_use("clam")
    except tk.TclError:
        pass

    RosbagOdometryGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
