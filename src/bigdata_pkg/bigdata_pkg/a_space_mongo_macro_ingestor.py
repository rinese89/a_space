#!/usr/bin/env python3

from datetime import datetime, timezone
from typing import Any, Dict, List
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from pymongo import MongoClient, ASCENDING
from pymongo.errors import PyMongoError

from px4_msgs.msg import (
    VehicleGlobalPosition,
    VehicleOdometry,
)


class ASpaceMongoMacroIngestor(Node):

    def __init__(self):
        super().__init__("a_space_mongo_macro_ingestor")

        #self.declare_parameter("mongo_uri", "mongodb://localhost:27017/")
        self.declare_parameter("mongo_uri", "")
        self.declare_parameter("db_name", "uas-telemetry")
        self.declare_parameter("collection_name", "macro_topics")
        self.declare_parameter("drone_id", "drone_01")
        self.declare_parameter("topic_prefix", "/fmu/out")

        # Insercion por lotes.
        self.declare_parameter("bulk_insert_size", 50)
        self.declare_parameter("flush_period_s", 2.0)

        # Muestreo macro.
        # Guarda como maximo 1 muestra cada N segundos por topico.
        self.declare_parameter("global_position_sample_period_s", 1.0)
        self.declare_parameter("odometry_sample_period_s", 1.0)

        self.mongo_uri = self.get_parameter("mongo_uri").value
        self.db_name = self.get_parameter("db_name").value
        self.collection_name = self.get_parameter("collection_name").value
        self.drone_id = self.get_parameter("drone_id").value
        self.topic_prefix = self.get_parameter("topic_prefix").value.rstrip("/")

        self.bulk_insert_size = int(self.get_parameter("bulk_insert_size").value)
        self.flush_period_s = float(self.get_parameter("flush_period_s").value)

        self.global_position_sample_period_s = float(
            self.get_parameter("global_position_sample_period_s").value
        )
        self.odometry_sample_period_s = float(
            self.get_parameter("odometry_sample_period_s").value
        )

        self.buffer: List[Dict[str, Any]] = []
        self.buffer_lock = threading.Lock()

        # Ultimo instante ROS almacenado por topico, en nanosegundos.
        self.last_stored_ros_ns: Dict[str, int] = {}

        self.mongo_client = MongoClient(self.mongo_uri)
        self.db = self.mongo_client[self.db_name]
        self.collection = self.db[self.collection_name]

        self.create_indexes()

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_px4_subscriptions()

        self.flush_timer = self.create_timer(
            self.flush_period_s,
            self.flush_buffer
        )

        self.get_logger().info("A-space Mongo macro ingestor iniciado.")
        self.get_logger().info(f"Mongo URI: {self.mongo_uri}")
        self.get_logger().info(f"Database: {self.db_name}")
        self.get_logger().info(f"Collection: {self.collection_name}")
        self.get_logger().info(f"Drone ID: {self.drone_id}")
        self.get_logger().info(f"Topic prefix: {self.topic_prefix}")
        self.get_logger().info(
            f"Global position sample period: {self.global_position_sample_period_s} s"
        )
        self.get_logger().info(
            f"Odometry sample period: {self.odometry_sample_period_s} s"
        )

    def create_indexes(self):
        """
        Crea indices utiles para la fase ETL.
        """

        try:
            self.collection.create_index(
                [("drone_id", ASCENDING), ("timestamp_px4_us", ASCENDING)]
            )
            self.collection.create_index(
                [
                    ("drone_id", ASCENDING),
                    ("logical_topic", ASCENDING),
                    ("timestamp_px4_us", ASCENDING),
                ]
            )
            self.collection.create_index(
                [("logical_topic", ASCENDING)]
            )
            self.collection.create_index(
                [("timestamp_ros_ns", ASCENDING)]
            )

            self.get_logger().info("Indices MongoDB creados/verificados.")

        except PyMongoError as exc:
            self.get_logger().error(f"Error creando indices en MongoDB: {exc}")

    def create_px4_subscriptions(self):
        """
        Crea las suscripciones a los topicos PX4 que se quieren capturar.

        De momento:
            /fmu/out/vehicle_global_position
            /fmu/out/vehicle_odometry
        """

        self.create_subscription(
            VehicleGlobalPosition,
            f"{self.topic_prefix}/vehicle_global_position",
            self.vehicle_global_position_callback,
            self.qos_profile,
        )

        self.create_subscription(
            VehicleOdometry,
            f"{self.topic_prefix}/vehicle_odometry",
            self.vehicle_odometry_callback,
            self.qos_profile,
        )

        self.get_logger().info(
            f"Suscrito a {self.topic_prefix}/vehicle_global_position"
        )
        self.get_logger().info(
            f"Suscrito a {self.topic_prefix}/vehicle_odometry"
        )

    def now_ros_ns(self) -> int:
        """
        Devuelve el tiempo ROS actual en nanosegundos.
        """

        ros_now = self.get_clock().now()
        ros_now_msg = ros_now.to_msg()

        return (
            int(ros_now_msg.sec) * 1_000_000_000
            + int(ros_now_msg.nanosec)
        )

    def should_store(self, logical_topic: str, sample_period_s: float, now_ns: int) -> bool:
        """
        Control de muestreo macro por topico.

        Si sample_period_s <= 0, almacena todos los mensajes de ese topico.
        """

        if sample_period_s <= 0:
            return True

        last_ns = self.last_stored_ros_ns.get(logical_topic)

        if last_ns is None:
            self.last_stored_ros_ns[logical_topic] = now_ns
            return True

        elapsed_s = (now_ns - last_ns) / 1_000_000_000.0

        if elapsed_s >= sample_period_s:
            self.last_stored_ros_ns[logical_topic] = now_ns
            return True

        return False

    def vehicle_global_position_callback(self, msg: VehicleGlobalPosition):
        """
        Extrae solo los campos macro relevantes de VehicleGlobalPosition.

        Campos almacenados:
            timestamp
            timestamp_sample
            lat
            lon
            alt
            alt_ellipsoid
        """

        logical_topic = "vehicle_global_position"
        now_ns = self.now_ros_ns()

        if not self.should_store(
            logical_topic,
            self.global_position_sample_period_s,
            now_ns
        ):
            return

        data = {
            "timestamp": int(msg.timestamp),
            "timestamp_sample": int(msg.timestamp_sample),
            "lat": float(msg.lat),
            "lon": float(msg.lon),
            "alt": float(msg.alt),
            "alt_ellipsoid": float(msg.alt_ellipsoid),
        }

        document = self.build_document(
            logical_topic=logical_topic,
            topic_name=f"{self.topic_prefix}/vehicle_global_position",
            message_type="px4_msgs/msg/VehicleGlobalPosition",
            timestamp_ros_ns=now_ns,
            timestamp_px4_us=int(msg.timestamp),
            timestamp_sample_px4_us=int(msg.timestamp_sample),
            data=data,
        )

        self.add_to_buffer(document)

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        """
        Extrae solo los campos macro relevantes de VehicleOdometry.

        Campos almacenados:
            timestamp
            position
            q
            velocity
        """

        logical_topic = "vehicle_odometry"
        now_ns = self.now_ros_ns()

        if not self.should_store(
            logical_topic,
            self.odometry_sample_period_s,
            now_ns
        ):
            return

        data = {
            "timestamp": int(msg.timestamp),
            "position": [float(v) for v in msg.position],
            "q": [float(v) for v in msg.q],
            "velocity": [float(v) for v in msg.velocity],
        }

        document = self.build_document(
            logical_topic=logical_topic,
            topic_name=f"{self.topic_prefix}/vehicle_odometry",
            message_type="px4_msgs/msg/VehicleOdometry",
            timestamp_ros_ns=now_ns,
            timestamp_px4_us=int(msg.timestamp),
            timestamp_sample_px4_us=None,
            data=data,
        )

        self.add_to_buffer(document)

    def build_document(
        self,
        logical_topic: str,
        topic_name: str,
        message_type: str,
        timestamp_ros_ns: int,
        timestamp_px4_us: int,
        timestamp_sample_px4_us: int | None,
        data: Dict[str, Any],
    ) -> Dict[str, Any]:
        """
        Construye el documento compacto para MongoDB.
        """

        document = {
            "project": "A-space",
            "source": "px4_ros2",
            "storage_level": "macro_raw",
            "drone_id": self.drone_id,
            "logical_topic": logical_topic,
            "topic_name": topic_name,
            "message_type": message_type,
            "timestamp_ros_ns": timestamp_ros_ns,
            "timestamp_ros_iso": datetime.now(timezone.utc).isoformat(),
            "timestamp_px4_us": timestamp_px4_us,
            "timestamp_sample_px4_us": timestamp_sample_px4_us,
            "data": data,
        }

        return document

    def add_to_buffer(self, document: Dict[str, Any]):
        """
        Anade un documento al buffer y fuerza escritura si se alcanza el lote.
        """

        should_flush = False

        with self.buffer_lock:
            self.buffer.append(document)

            if len(self.buffer) >= self.bulk_insert_size:
                should_flush = True

        if should_flush:
            self.flush_buffer()

    def flush_buffer(self):
        """
        Inserta periodicamente en MongoDB los documentos acumulados.
        """

        with self.buffer_lock:
            if not self.buffer:
                return

            documents_to_insert = self.buffer
            self.buffer = []

        try:
            result = self.collection.insert_many(
                documents_to_insert,
                ordered=False,
            )

            self.get_logger().info(
                f"Insertados {len(result.inserted_ids)} documentos en MongoDB."
            )

        except PyMongoError as exc:
            self.get_logger().error(
                f"Error insertando documentos en MongoDB: {exc}"
            )

            # Si falla MongoDB, se reinsertan en el buffer para no perderlos.
            with self.buffer_lock:
                self.buffer = documents_to_insert + self.buffer

    def destroy_node(self):
        """
        Antes de cerrar el nodo, intenta vaciar el buffer.
        """

        self.get_logger().info("Cerrando nodo. Vaciando buffer MongoDB...")

        try:
            self.flush_buffer()
        except Exception as exc:
            self.get_logger().error(f"Error final al vaciar buffer: {exc}")

        try:
            self.mongo_client.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ASpaceMongoMacroIngestor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupcion por teclado.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
