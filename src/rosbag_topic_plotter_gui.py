#!/usr/bin/env python3
"""
A-space ROS 2 rosbag topic plotter GUI.

Supported topic/message families:
  - nav_msgs/msg/Odometry
  - px4_msgs/msg/VehicleOdometry
  - */zone_status or any ROS type whose leaf name is ZoneStatus

ZoneStatus exposes only: Tiempo, X, Y, Z.
"""

from __future__ import annotations

import math
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


@dataclass(frozen=True)
class VariableSpec:
    key: str
    label: str
    unit: str


TIME = VariableSpec("time_s", "Tiempo", "s")
X = VariableSpec("x", "X", "m")
Y = VariableSpec("y", "Y", "m")
Z = VariableSpec("z", "Z", "m")

ODOM_VARIABLES: Tuple[VariableSpec, ...] = (
    TIME, X, Y, Z,
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
)

ZONE_STATUS_VARIABLES: Tuple[VariableSpec, ...] = (TIME, X, Y, Z)


@dataclass(frozen=True)
class TopicDescriptor:
    topic: str
    msg_type: str
    adapter_id: str
    adapter_label: str
    variables: Tuple[VariableSpec, ...]


@dataclass
class TopicSeries:
    topic: str
    msg_type: str
    adapter_id: str
    bag_time_ns: List[int] = field(default_factory=list)
    values_map: Dict[str, List[float]] = field(default_factory=dict)

    def append_value(self, key: str, value: float) -> None:
        self.values_map.setdefault(key, []).append(float(value))

    def values(self, key: str) -> List[float]:
        return self.values_map.get(key, [])

    def finalise_time(self, t0_ns: int) -> None:
        self.values_map["time_s"] = [
            (timestamp - t0_ns) * 1.0e-9 for timestamp in self.bag_time_ns
        ]

    @property
    def sample_count(self) -> int:
        return len(self.bag_time_ns)


@dataclass(frozen=True)
class CurveDefinition:
    topic: str
    x_key: str
    y_key: str


def quaternion_xyzw_to_rpy(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def is_zone_status_topic(topic: str, msg_type: str) -> bool:
    return (
        topic.rstrip("/").split("/")[-1] == "zone_status"
        or msg_type.split("/")[-1] == "ZoneStatus"
    )


def adapter_for(topic: str, msg_type: str) -> Optional[TopicDescriptor]:
    if msg_type == NAV_ODOM_TYPE:
        return TopicDescriptor(topic, msg_type, "nav_odom", "nav_msgs/Odometry", ODOM_VARIABLES)
    if msg_type == PX4_ODOM_TYPE:
        return TopicDescriptor(topic, msg_type, "px4_odom", "PX4 VehicleOdometry", ODOM_VARIABLES)
    if is_zone_status_topic(topic, msg_type):
        return TopicDescriptor(topic, msg_type, "zone_status", "ZoneStatus", ZONE_STATUS_VARIABLES)
    return None


def decode_nav_odometry(msg) -> Dict[str, float]:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    v = msg.twist.twist.linear
    w = msg.twist.twist.angular
    roll, pitch, yaw = quaternion_xyzw_to_rpy(q.x, q.y, q.z, q.w)
    vx, vy, vz = float(v.x), float(v.y), float(v.z)
    wx, wy, wz = float(w.x), float(w.y), float(w.z)
    return {
        "x": float(p.x), "y": float(p.y), "z": float(p.z),
        "vx": vx, "vy": vy, "vz": vz,
        "speed": math.sqrt(vx * vx + vy * vy + vz * vz),
        "roll_deg": math.degrees(roll),
        "pitch_deg": math.degrees(pitch),
        "yaw_deg": math.degrees(yaw),
        "wx": wx, "wy": wy, "wz": wz,
        "angular_speed": math.sqrt(wx * wx + wy * wy + wz * wz),
    }


def decode_px4_odometry(msg) -> Dict[str, float]:
    px, py, pz = map(float, msg.position[:3])
    vx, vy, vz = map(float, msg.velocity[:3])
    qw, qx, qy, qz = map(float, msg.q[:4])
    roll, pitch, yaw = quaternion_xyzw_to_rpy(qx, qy, qz, qw)
    wx, wy, wz = map(float, msg.angular_velocity[:3])
    return {
        "x": px, "y": py, "z": pz,
        "vx": vx, "vy": vy, "vz": vz,
        "speed": math.sqrt(vx * vx + vy * vy + vz * vz),
        "roll_deg": math.degrees(roll),
        "pitch_deg": math.degrees(pitch),
        "yaw_deg": math.degrees(yaw),
        "wx": wx, "wy": wy, "wz": wz,
        "angular_speed": math.sqrt(wx * wx + wy * wy + wz * wz),
    }


def decode_zone_status(msg) -> Dict[str, float]:
    p = msg.position
    return {"x": float(p.x), "y": float(p.y), "z": float(p.z)}


def decode_message(adapter_id: str, msg) -> Dict[str, float]:
    if adapter_id == "nav_odom":
        return decode_nav_odometry(msg)
    if adapter_id == "px4_odom":
        return decode_px4_odometry(msg)
    if adapter_id == "zone_status":
        return decode_zone_status(msg)
    raise ValueError(f"Adaptador no soportado: {adapter_id}")


def normalise_bag_path(selected: str) -> Path:
    path = Path(selected).expanduser().resolve()
    if path.is_dir():
        return path
    if path.is_file() and (path.name == "metadata.yaml" or path.suffix == ".db3"):
        return path.parent
    raise ValueError("Selecciona el directorio del rosbag, metadata.yaml o un fichero .db3.")


def create_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def discover_topics(bag_path: Path) -> Dict[str, TopicDescriptor]:
    reader = create_reader(bag_path)
    result: Dict[str, TopicDescriptor] = {}
    for item in reader.get_all_topics_and_types():
        descriptor = adapter_for(item.name, item.type)
        if descriptor is not None:
            result[item.name] = descriptor
    return result


def short_topic_name(topic: str) -> str:
    parts = [p for p in topic.split("/") if p]
    if len(parts) >= 2 and parts[-1] in ("odom", "zone_status"):
        return f"{parts[-2]}/{parts[-1]}"
    if parts and parts[-1] == "vehicle_odometry":
        return parts[0]
    return topic


class RosbagTopicPlotterGUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("A-space ROS 2 Rosbag Topic Plotter")
        self.root.geometry("1550x920")
        self.root.minsize(1180, 720)

        self.bag_path: Optional[Path] = None
        self.descriptors: Dict[str, TopicDescriptor] = {}
        self.curves: List[CurveDefinition] = []
        self.loaded_data: Dict[str, TopicSeries] = {}

        self.bag_var = tk.StringVar()
        self.topic_var = tk.StringVar()
        self.x_var = tk.StringVar()
        self.y_var = tk.StringVar()
        self.plot_mode_var = tk.StringVar(value="Línea")
        self.equal_axis_var = tk.BooleanVar(value=False)
        self.grid_var = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar(value="Selecciona un rosbag para comenzar.")
        self.topic_info_var = tk.StringVar(value="No hay tópico seleccionado.")

        self._build_ui()
        self._create_empty_plot()

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=8)
        outer.pack(fill=tk.BOTH, expand=True)
        controls = ttk.Frame(outer)
        controls.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        plot_frame = ttk.Frame(outer)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        bag_box = ttk.LabelFrame(controls, text="1. Rosbag", padding=8)
        bag_box.pack(fill=tk.X, pady=(0, 8))
        ttk.Entry(bag_box, textvariable=self.bag_var, width=52).grid(
            row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6)
        )
        ttk.Button(bag_box, text="Seleccionar carpeta", command=self.select_bag_directory).grid(
            row=1, column=0, sticky="ew", padx=(0, 4)
        )
        ttk.Button(bag_box, text="Seleccionar metadata/.db3", command=self.select_bag_file).grid(
            row=1, column=1, sticky="ew", padx=(4, 0)
        )
        ttk.Button(bag_box, text="Cargar tópicos", command=self.load_topic_list).grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0)
        )
        bag_box.columnconfigure(0, weight=1)
        bag_box.columnconfigure(1, weight=1)

        selector_box = ttk.LabelFrame(controls, text="2. Tópico y variables", padding=8)
        selector_box.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(selector_box, text="Tópico:").grid(row=0, column=0, sticky="w")
        self.topic_combo = ttk.Combobox(
            selector_box, textvariable=self.topic_var, state="readonly", width=43
        )
        self.topic_combo.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(3, 4))
        self.topic_combo.bind("<<ComboboxSelected>>", self._on_topic_changed)
        ttk.Label(
            selector_box, textvariable=self.topic_info_var, wraplength=420, justify=tk.LEFT
        ).grid(row=2, column=0, columnspan=2, sticky="w", pady=(0, 8))

        ttk.Label(selector_box, text="Eje X:").grid(row=3, column=0, sticky="w")
        ttk.Label(selector_box, text="Eje Y:").grid(row=3, column=1, sticky="w")
        self.x_combo = ttk.Combobox(selector_box, textvariable=self.x_var, state="readonly", width=20)
        self.y_combo = ttk.Combobox(selector_box, textvariable=self.y_var, state="readonly", width=20)
        self.x_combo.grid(row=4, column=0, sticky="ew", padx=(0, 4))
        self.y_combo.grid(row=4, column=1, sticky="ew", padx=(4, 0))
        ttk.Button(selector_box, text="Añadir serie a la gráfica", command=self.add_curve).grid(
            row=5, column=0, columnspan=2, sticky="ew", pady=(9, 0)
        )
        selector_box.columnconfigure(0, weight=1)
        selector_box.columnconfigure(1, weight=1)

        curve_box = ttk.LabelFrame(controls, text="3. Series seleccionadas", padding=8)
        curve_box.pack(fill=tk.BOTH, expand=True, pady=(0, 8))
        curve_frame = ttk.Frame(curve_box)
        curve_frame.pack(fill=tk.BOTH, expand=True)
        self.curve_list = tk.Listbox(
            curve_frame, selectmode=tk.EXTENDED, exportselection=False, width=55, height=12
        )
        scroll = ttk.Scrollbar(curve_frame, orient=tk.VERTICAL, command=self.curve_list.yview)
        self.curve_list.configure(yscrollcommand=scroll.set)
        self.curve_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        buttons = ttk.Frame(curve_box)
        buttons.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(buttons, text="Eliminar seleccionadas", command=self.remove_selected_curves).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 3)
        )
        ttk.Button(buttons, text="Limpiar", command=self.clear_curves).pack(
            side=tk.LEFT, expand=True, fill=tk.X, padx=(3, 0)
        )

        option_box = ttk.LabelFrame(controls, text="4. Representación", padding=8)
        option_box.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(option_box, text="Modo:").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            option_box, textvariable=self.plot_mode_var,
            values=("Línea", "Dispersión"), state="readonly", width=18
        ).grid(row=0, column=1, sticky="ew", padx=(8, 0))
        ttk.Checkbutton(option_box, text="Misma escala X/Y", variable=self.equal_axis_var).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(6, 0)
        )
        ttk.Checkbutton(option_box, text="Mostrar rejilla", variable=self.grid_var).grid(
            row=2, column=0, columnspan=2, sticky="w"
        )
        ttk.Button(option_box, text="Graficar series", command=self.plot_curves).grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(10, 0)
        )
        ttk.Button(option_box, text="Guardar gráfica", command=self.save_current_plot).grid(
            row=4, column=0, columnspan=2, sticky="ew", pady=(5, 0)
        )
        option_box.columnconfigure(1, weight=1)

        status_box = ttk.LabelFrame(controls, text="Estado", padding=8)
        status_box.pack(fill=tk.X)
        ttk.Label(
            status_box, textvariable=self.status_var, wraplength=420, justify=tk.LEFT
        ).pack(fill=tk.X)

        self.figure = plt.Figure(figsize=(9.5, 7.3), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        toolbar_frame = ttk.Frame(plot_frame)
        toolbar_frame.pack(fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.pack(fill=tk.X)

    def _create_empty_plot(self) -> None:
        self.ax.clear()
        self.ax.set_title("Añade una o varias series y pulsa «Graficar series»")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid(True)
        self.canvas.draw_idle()

    def select_bag_directory(self) -> None:
        selected = filedialog.askdirectory(title="Seleccionar directorio rosbag2")
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
            raise ValueError(f"No se encuentra metadata.yaml en:\n{bag_path}")
        return bag_path

    def load_topic_list(self) -> None:
        try:
            self.bag_path = self._resolve_bag()
            self.descriptors = discover_topics(self.bag_path)
        except Exception as exc:
            messagebox.showerror("Error al abrir rosbag", str(exc))
            return

        topics = sorted(self.descriptors)
        self.topic_combo["values"] = topics
        self.loaded_data.clear()
        self.clear_curves()

        if not topics:
            self.topic_var.set("")
            self.x_combo["values"] = ()
            self.y_combo["values"] = ()
            self.status_var.set("No se encontraron tópicos compatibles.")
            self.topic_info_var.set("No hay tópico seleccionado.")
            return

        self.topic_var.set(topics[0])
        self._update_variable_selectors(self.descriptors[topics[0]])

        counts: Dict[str, int] = {}
        for descriptor in self.descriptors.values():
            counts[descriptor.adapter_label] = counts.get(descriptor.adapter_label, 0) + 1
        summary = ", ".join(f"{label}: {count}" for label, count in counts.items())
        self.status_var.set(
            f"Rosbag cargado: {self.bag_path}\n{len(topics)} tópico(s) compatibles. {summary}"
        )

    def _on_topic_changed(self, _event=None) -> None:
        descriptor = self.descriptors.get(self.topic_var.get())
        if descriptor is not None:
            self._update_variable_selectors(descriptor)

    def _update_variable_selectors(self, descriptor: TopicDescriptor) -> None:
        labels = [variable.label for variable in descriptor.variables]
        self.x_combo["values"] = labels
        self.y_combo["values"] = labels
        self.x_var.set("Tiempo" if "Tiempo" in labels else (labels[0] if labels else ""))
        self.y_var.set("X" if "X" in labels else (labels[0] if labels else ""))
        self.topic_info_var.set(
            f"Tipo: {descriptor.msg_type}\n"
            f"Adaptador: {descriptor.adapter_label}\n"
            f"Variables: {', '.join(labels)}"
        )

    @staticmethod
    def _spec_from_label(descriptor: TopicDescriptor, label: str) -> VariableSpec:
        for spec in descriptor.variables:
            if spec.label == label:
                return spec
        raise KeyError(label)

    def add_curve(self) -> None:
        topic = self.topic_var.get()
        descriptor = self.descriptors.get(topic)
        if descriptor is None:
            messagebox.showwarning("Sin tópico", "Selecciona un tópico válido.")
            return
        try:
            x_spec = self._spec_from_label(descriptor, self.x_var.get())
            y_spec = self._spec_from_label(descriptor, self.y_var.get())
        except KeyError:
            messagebox.showwarning("Variables inválidas", "Selecciona las variables X e Y.")
            return

        curve = CurveDefinition(topic=topic, x_key=x_spec.key, y_key=y_spec.key)
        if curve in self.curves:
            messagebox.showinfo("Serie existente", "Esa combinación ya está añadida.")
            return
        self.curves.append(curve)
        self.curve_list.insert(tk.END, self._curve_text(curve))

    def _curve_text(self, curve: CurveDefinition) -> str:
        descriptor = self.descriptors[curve.topic]
        x_spec = next(v for v in descriptor.variables if v.key == curve.x_key)
        y_spec = next(v for v in descriptor.variables if v.key == curve.y_key)
        return f"{curve.topic}   |   {y_spec.label} vs {x_spec.label}"

    def remove_selected_curves(self) -> None:
        for index in reversed(self.curve_list.curselection()):
            del self.curves[index]
            self.curve_list.delete(index)

    def clear_curves(self) -> None:
        self.curves.clear()
        self.curve_list.delete(0, tk.END)

    def _load_topics(self, topics: Sequence[str]) -> Dict[str, TopicSeries]:
        if self.bag_path is None:
            raise RuntimeError("No hay ningún rosbag cargado.")

        requested = sorted(set(topics))
        reader = create_reader(self.bag_path)
        msg_classes = {}

        for topic in requested:
            descriptor = self.descriptors[topic]
            try:
                msg_classes[descriptor.msg_type] = get_message(descriptor.msg_type)
            except Exception as exc:
                raise RuntimeError(
                    f"No se pudo cargar el tipo ROS:\n{descriptor.msg_type}\n\n"
                    "Comprueba que el workspace que define ese mensaje está sourced.\n\n"
                    f"{exc}"
                ) from exc

        result = {
            topic: TopicSeries(
                topic=topic,
                msg_type=self.descriptors[topic].msg_type,
                adapter_id=self.descriptors[topic].adapter_id,
            )
            for topic in requested
        }

        selected = set(requested)
        self.status_var.set("Leyendo mensajes del rosbag...")
        self.root.update_idletasks()

        while reader.has_next():
            topic, raw, timestamp_ns = reader.read_next()
            if topic not in selected:
                continue
            descriptor = self.descriptors[topic]
            msg = deserialize_message(raw, msg_classes[descriptor.msg_type])
            decoded = decode_message(descriptor.adapter_id, msg)
            series = result[topic]
            series.bag_time_ns.append(int(timestamp_ns))
            for key, value in decoded.items():
                series.append_value(key, value)

        nonempty = [series for series in result.values() if series.sample_count > 0]
        if not nonempty:
            raise RuntimeError("No se encontraron mensajes en los tópicos seleccionados.")

        t0_ns = min(series.bag_time_ns[0] for series in nonempty)
        for series in nonempty:
            series.finalise_time(t0_ns)
        return result

    def plot_curves(self) -> None:
        if not self.curves:
            messagebox.showwarning("Sin series", "Añade al menos una serie antes de graficar.")
            return

        topics = [curve.topic for curve in self.curves]
        try:
            self.loaded_data = self._load_topics(topics)
        except Exception as exc:
            messagebox.showerror("Error leyendo rosbag", str(exc))
            return

        self.ax.clear()
        mode = self.plot_mode_var.get()
        x_specs: List[VariableSpec] = []
        y_specs: List[VariableSpec] = []
        plotted = 0

        for curve in self.curves:
            descriptor = self.descriptors[curve.topic]
            series = self.loaded_data[curve.topic]
            x_spec = next(v for v in descriptor.variables if v.key == curve.x_key)
            y_spec = next(v for v in descriptor.variables if v.key == curve.y_key)
            x_values = series.values(curve.x_key)
            y_values = series.values(curve.y_key)
            if not x_values or not y_values:
                continue
            n = min(len(x_values), len(y_values))
            label = f"{short_topic_name(curve.topic)} — {y_spec.label} vs {x_spec.label}"
            if mode == "Dispersión":
                self.ax.scatter(x_values[:n], y_values[:n], s=8, label=label)
            else:
                self.ax.plot(x_values[:n], y_values[:n], label=label)
            x_specs.append(x_spec)
            y_specs.append(y_spec)
            plotted += 1

        if plotted == 0:
            messagebox.showwarning("Sin datos", "No se pudo representar ninguna serie.")
            self._create_empty_plot()
            return

        self.ax.set_xlabel(self._combined_axis_label(x_specs, "X"))
        self.ax.set_ylabel(self._combined_axis_label(y_specs, "Y"))

        if len(self.curves) == 1:
            curve = self.curves[0]
            descriptor = self.descriptors[curve.topic]
            x_spec = next(v for v in descriptor.variables if v.key == curve.x_key)
            y_spec = next(v for v in descriptor.variables if v.key == curve.y_key)
            self.ax.set_title(f"{y_spec.label} frente a {x_spec.label}\n{curve.topic}")
        else:
            self.ax.set_title(f"Rosbag — {plotted} series")

        self.ax.grid(self.grid_var.get())
        if self.equal_axis_var.get():
            self.ax.axis("equal")
        self.ax.legend()
        self.figure.tight_layout()
        self.canvas.draw_idle()

        samples = sum(self.loaded_data[t].sample_count for t in set(topics))
        self.status_var.set(
            f"Gráfica generada: {plotted} serie(s), {len(set(topics))} tópico(s), "
            f"{samples} mensajes cargados."
        )

    @staticmethod
    def _combined_axis_label(specs: Sequence[VariableSpec], fallback: str) -> str:
        if not specs:
            return fallback
        labels = {spec.label for spec in specs}
        units = {spec.unit for spec in specs}
        if len(labels) == 1:
            spec = specs[0]
            return f"{spec.label} [{spec.unit}]" if spec.unit else spec.label
        if len(units) == 1:
            unit = next(iter(units))
            return f"Variables seleccionadas [{unit}]" if unit else "Variables seleccionadas"
        return "Variables seleccionadas"

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
            self.figure.savefig(path, dpi=180, bbox_inches="tight")
        except Exception as exc:
            messagebox.showerror("Error guardando gráfica", str(exc))
            return
        self.status_var.set(f"Gráfica guardada en:\n{path}")


def main() -> None:
    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except tk.TclError:
        pass
    RosbagTopicPlotterGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
