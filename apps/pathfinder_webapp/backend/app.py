import asyncio
import math
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, WebSocket
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time


from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint 
from sensor_msgs.msg import NavSatFix,Imu,NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseStamped
from nav2_msgs.action import FollowWaypoints
from visualization_msgs.msg import Marker, MarkerArray
# (Mission endpointi şimdilik placeholder)
# from nav2_msgs.action import FollowWaypoints

FRONTEND_DIR = (Path(__file__).parent.parent / "frontend").resolve()

app = FastAPI()
app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")


@app.get("/")
def root():
    return FileResponse(str(FRONTEND_DIR / "index.html"))


def quat_to_heading(x, y, z, w):
    """
    Quaternion (ENU, ROS standard) → Gerçek pusula heading (0–360°)
    """

    # 1️⃣ Quaternion → yaw (RAD)
    yaw = math.atan2(
        2.0 * (w*z + x*y),
        1.0 - 2.0 * (y*y + z*z)
    )

    # 2️⃣ Yaw → derece
    yaw_deg = math.degrees(yaw)

    # 3️⃣ Yaw → Heading dönüşümü
    heading_deg = (90.0 - yaw_deg) % 360.0

    return heading_deg

class WebBridgeNode(Node):
    def __init__(self):
        super().__init__("pathfinder_web_bridge")

        # Topics - change if needed
        self.gps_topic = self.declare_parameter("gps_topic", "/fix").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odometry/local").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel_stop").value
        self.heading_topic = self.declare_parameter("heading_topic", "/gps/imu").value
        self.fromll_client = self.create_client(FromLL, '/fromLL')
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.get_logger().info("FromLL Service Client initialized.")
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._heading_deg: float = 0.0
        self._vx: float = 0.0
        self._vyaw: float = 0.0

        # GPS status tracking (NavSatFix.status)
        self._gps_status_code: Optional[int] = None
        self._gps_status_service: Optional[int] = None
        self._gps_status_label: Optional[str] = None  # friendly label like 'RTK Fixed', 'RTK Float', '3D Fix'
        self._gps_ts: Optional[float] = None  # time.time() of last received NavSatFix

        self._trace: List[Dict[str, float]] = []
        self._trace_max = 1000

        self._mission_state: str = "IDLE"

        self.create_subscription(NavSatFix, self.gps_topic, self._on_gps, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(Imu, self.heading_topic, self._on_heading, 10)  # just to keep the topic alive
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._nav_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._current_goal_handle = None

        # Emergency stop state: when True publish zero cmd_vel continuously
        self._stopped: bool = False
        self._mission_state_before_stop: Optional[str] = None
        # timer to publish zero cmd_vel while stopped (10 Hz)
        self.create_timer(0.1, self._stopped_publisher_cb)

        self.get_logger().info(f"GPS topic: {self.gps_topic}")
        self.get_logger().info(f"Odom topic: {self.odom_topic}")
        self.get_logger().info(f"cmd_vel topic: {self.cmd_vel_topic}")

    def _on_gps(self, msg: NavSatFix):
        # Optional: ignore invalid fix (status < 0)
        self._lat = float(msg.latitude)
        self._lon = float(msg.longitude)

        # capture NavSatFix status (code & service) for frontend display
        try:
            self._gps_status_code = int(msg.status.status)
            self._gps_status_service = int(msg.status.service)
        except Exception:
            self._gps_status_code = None
            self._gps_status_service = None

        # Prefer RTD100 driver's semantics: explicit NAVSAT status codes indicate RTK
        label = None
        try:
            # Direct mapping from NavSatStatus when set by driver
            if self._gps_status_code == NavSatStatus.STATUS_GBAS_FIX:
                label = "RTK Fixed"
            elif self._gps_status_code == NavSatStatus.STATUS_SBAS_FIX:
                label = "RTK Float"
            elif self._gps_status_code == NavSatStatus.STATUS_FIX:
                label = "3D Fix"
            elif self._gps_status_code == -1:
                label = "No Fix"

            # Otherwise, fall back to covariance heuristic (driver sets covariance for BESTPOSA/GGA)
            if label is None:
                cov0 = None
                if hasattr(msg, 'position_covariance') and msg.position_covariance:
                    try:
                        cov0 = float(msg.position_covariance[0])
                    except Exception:
                        cov0 = None
                if cov0 is not None and cov0 > 0:
                    std_m = math.sqrt(cov0)
                    if std_m < 0.2:
                        label = "RTK Fixed"
                    elif std_m < 3.0:
                        label = "RTK Float"

            # Final fallback
            if label is None:
                label = self._gps_status_text(self._gps_status_code)
        except Exception:
            label = self._gps_status_text(self._gps_status_code)

        # Debug log for diagnosis (will only show when logger level allows debug)
        try:
            cov0_debug = None
            if hasattr(msg, 'position_covariance') and msg.position_covariance:
                cov0_debug = float(msg.position_covariance[0])
            self.get_logger().debug(f"GPS label calc: code={self._gps_status_code}, svc={self._gps_status_service}, cov0={cov0_debug}, label={label}")
        except Exception:
            pass

        self._gps_status_label = label
        try:
            self._gps_ts = time.time()
        except Exception:
            self._gps_ts = None

        if self._lat and self._lon:
            self._trace.append({"lat": self._lat, "lon": self._lon})
            if len(self._trace) > self._trace_max:
                self._trace = self._trace[-self._trace_max:]
    def _on_heading(self, msg: Imu):
        q = msg.orientation
        heading_deg = quat_to_heading(q.x, q.y, q.z, q.w)
        self._heading_deg = heading_deg

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # self._heading_deg = quat_to_heading_deg(q.x, q.y, q.z, q.w)
        self._vx = float(msg.twist.twist.linear.x)
        self._vyaw = float(msg.twist.twist.angular.z)

    def _gps_status_text(self, code: Optional[int]) -> str:
        if code is None:
            return "UNKNOWN"
        mapping = {
            -1: "NO_FIX",
             0: "FIX",
             1: "SBAS_FIX",
             2: "GBAS_FIX",
        }
        return mapping.get(code, f"CODE:{code}")

    def telemetry(self) -> Dict[str, Any]:
        lat = self._lat if self._lat is not None else 0.0
        lon = self._lon if self._lon is not None else 0.0
        heading_deg = self._heading_deg

        # Compute GPS label and age; report 'No GPS' if last message older than 1s
        gps_label = (self._gps_status_label if self._gps_status_label is not None else self._gps_status_text(self._gps_status_code))
        gps_age = None
        try:
            if self._gps_ts is not None:
                gps_age = time.time() - self._gps_ts
                if gps_age > 1.0:
                    gps_label = "No GPS"
        except Exception:
            pass

        return {
            "lat": lat,
            "lon": lon,
            "yaw_deg": heading_deg,
            "vx": self._vx,
            "vyaw": self._vyaw,
            "gps_status": {
                "code": self._gps_status_code,
                "service": self._gps_status_service,
                "str": gps_label,
                "age_s": round(gps_age, 3) if gps_age is not None else None,
            },
            "mission_state": self._mission_state,
            "trace": self._trace[-600:],
        }

    def emergency_stop(self):
        """Engage emergency stop: publish zero cmd_vel immediately and start continuous publishing."""
        msg = Twist()
        # publish immediate few messages for robustness
        for _ in range(5):
            self.cmd_pub.publish(msg)
        # set stopped flag so timer will continue publishing zeros
        if not self._stopped:
            self._mission_state_before_stop = self._mission_state
        self._stopped = True
        self._mission_state = "STOPPED"
        self.get_logger().warn("EMERGENCY STOP: engaged, publishing cmd_vel=0 continuously")

    def resume_stop(self):
        """Resume from emergency stop: stop continuous publishing and restore mission state."""
        if self._stopped:
            self._stopped = False
            self._mission_state = self._mission_state_before_stop or "IDLE"
            self._mission_state_before_stop = None
            self.get_logger().info("EMERGENCY STOP: resumed, stopped publishing cmd_vel")

    def _stopped_publisher_cb(self):
        """Timer callback that publishes zero cmd_vel while stopped."""
        if not self._stopped:
            return
        try:
            msg = Twist()
            self.cmd_pub.publish(msg)
        except Exception:
            self.get_logger().error("Failed to publish stop cmd_vel")
    def publish_markers(self, goal_msg):
        """
        Waypointleri RViz'de görmek için MarkerArray yayınlar.
        """
        marker_array = MarkerArray()

        # Eski markerları temizle (DELETEALL)
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Yeni noktaları ekle
        for i, pose in enumerate(goal_msg.poses):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "mission_waypoints"
            marker.id = i
            marker.type = Marker.SPHERE  # Noktaları küre olarak göster
            marker.action = Marker.ADD
            
            # Pozisyon
            marker.pose.position.x = pose.pose.position.x
            marker.pose.position.y = pose.pose.position.y
            marker.pose.position.z = 0.2  # Yerde kaybolmasın diye hafif yukarı
            
            # Boyut (0.5 metre çapında toplar)
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            # Renk (Cam Göbeği - Cyan)
            marker.color.a = 1.0 # Opaklık
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            
            marker_array.markers.append(marker)

            # Opsiyonel: Sıra numarasını gösteren yazı (TEXT)
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.ns = "mission_text"
            text_marker.id = 1000 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pose.pose.position.x
            text_marker.pose.position.y = pose.pose.position.y
            text_marker.pose.position.z = 0.8 # Topun üzerinde yazsın
            text_marker.scale.z = 0.4 # Yazı boyutu
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = str(i)
            
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info("RViz markerları yayınlandı.")
    async def get_map_coordinates(self, lat, lon):
        """
        /fromLL servisine güvenli asenkron çağrı yapar.
        ROS Future ile FastAPI async yapısını barıştırmak için polling kullanır.
        """
        # 1. Servis aktif mi kontrol et
        if not self.fromll_client.service_is_ready():
            self.get_logger().error("/fromLL servisi bulunamadı! 'ros2 run robot_localization navsat_transform_node' çalışıyor mu?")
            return None, None

        # 2. İsteği Hazırla
        req = FromLL.Request()
        req.ll_point = GeoPoint()
        req.ll_point.latitude = float(lat)
        req.ll_point.longitude = float(lon)
        req.ll_point.altitude = 0.0

        # 3. İsteği gönder (call_async)
        future = self.fromll_client.call_async(req)

        # 4. KRİTİK: ROS işlemini asyncio içinde bekletme (Polling)
        while not future.done():
            await asyncio.sleep(0.02) # 20ms bekle ve tekrar kontrol et

        # 5. Sonucu al
        try:
            result = future.result()
            return result.map_point.x, result.map_point.y
        except Exception as e:
            self.get_logger().error(f"Servis çağrısı hatası: {e}")
            return None, None

    async def send_mission_latlon(self, waypoints: List[Dict[str, float]]):
        """
        Waypoint listesini alır, dönüştürür ve Nav2'ye gönderir.
        """
        # 1. Action Client Kontrolü
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 Action Server (/follow_waypoints) bulunamadı!")
            raise RuntimeError("Nav2 sistemi hazır değil. (Simülasyon veya Robot açık mı?)")

        self.get_logger().info(f"{len(waypoints)} noktalı görev hazırlanıyor...")

        # BURADA FollowWaypoints KULLANIYORUZ
        goal_msg = FollowWaypoints.Goal()
        
        # 2. Tüm noktaları dönüştür
        valid_points = 0
        for i, wp in enumerate(waypoints):
            lat = wp.get('lat')
            lon = wp.get('lon')
            
            # Servisten koordinat çevir (Await ile bekle)
            map_x, map_y = await self.get_map_coordinates(lat, lon)
            self.get_logger().info(f"Dönüşüm Kontrolü: Giriş(Lat:{lat}, Lon:{lon}) -> Çıkış(X:{map_x}, Y:{map_y})")
            if map_x is None:
                self.get_logger().warn(f"Waypoint #{i} ({lat}, {lon}) dönüştürülemedi, atlanıyor.")
                continue

            # Pose oluştur
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = map_x
            pose.pose.position.y = map_y
            pose.pose.orientation.w = 1.0 # Varsayılan yönelim
            
            goal_msg.poses.append(pose)
            valid_points += 1

        if valid_points == 0:
            raise RuntimeError("Hiçbir waypoint geçerli bir harita koordinatına dönüştürülemedi!")

        self.get_logger().info(f"Nav2'ye {valid_points} noktalı görev gönderiliyor...")

        self.publish_markers(goal_msg)
        send_goal_future = self._nav_client.send_goal_async(goal_msg)

        while not send_goal_future.done():
            await asyncio.sleep(0.02)

        # 4. Sonucu İşle
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Görev Nav2 tarafından reddedildi!')
            self._mission_state = "MISSION_REJECTED"
            return

        self.get_logger().info('Görev kabul edildi, robot harekete başlıyor.')
        self._current_goal_handle = goal_handle
        self._mission_state = "MISSION_STARTED"

    def _mission_accepted_callback(self, future):
        """Nav2 görevi kabul etti mi etmedi mi burada anlarız ve Handle'ı saklarız."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Görev Nav2 tarafından reddedildi!')
            self._mission_state = "MISSION_REJECTED"
            return

        self.get_logger().info('Görev kabul edildi, robot harekete başlıyor.')
        # --- KRİTİK NOKTA: Handle'ı saklıyoruz ---
        self._current_goal_handle = goal_handle

    async def cancel_mission(self):
        self.get_logger().warn("İptal isteği alındı...")

        # 1. Aktif bir görev var mı kontrol et
        if self._current_goal_handle is None:
            self.get_logger().info("İptal edilecek aktif bir görev yok.")
            self._mission_state = "IDLE"
            return

        # 2. İptal isteğini Nav2'ye gönder
        self.get_logger().info("Nav2'ye iptal sinyali gönderiliyor...")
        future = self._current_goal_handle.cancel_goal_async()
        
        # İptal sonucunu beklemek için callback ekleyebiliriz (opsiyonel)
        future.add_done_callback(self._cancel_done_callback)
        
        self._mission_state = "CANCELLING..."

    def _cancel_done_callback(self, future):
        """İptal işlemi tamamlandığında çalışır"""
        self.get_logger().info("Görev başarıyla iptal edildi.")
        self._mission_state = "MISSION_CANCELLED"
        # Handle'ı sıfırla ki tekrar iptal etmeye çalışmayalım
        self._current_goal_handle = None

ros_node: Optional[WebBridgeNode] = None
ros_spin_task: Optional[asyncio.Task] = None


@app.on_event("startup")
async def on_startup():
    global ros_node, ros_spin_task
    rclpy.init(args=None)
    ros_node = WebBridgeNode()

    async def spin():
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.1)
            await asyncio.sleep(0.01)

    ros_spin_task = asyncio.get_event_loop().create_task(spin())


@app.on_event("shutdown")
async def on_shutdown():
    global ros_node, ros_spin_task
    if ros_spin_task:
        ros_spin_task.cancel()
    if ros_node:
        ros_node.destroy_node()
    rclpy.shutdown()


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await asyncio.sleep(0.1)  # 10 Hz
            await ws.send_json(ros_node.telemetry())
    except Exception:
        pass


@app.post("/api/stop")
async def api_stop():
    # Toggle emergency stop: if already stopped -> resume, else engage stop
    if ros_node._stopped:
        ros_node.resume_stop()
        return {"ok": True, "state": "resumed"}
    else:
        ros_node.emergency_stop()
        return {"ok": True, "state": "stopped"}


@app.post("/api/mission")
async def api_mission(payload: Dict[str, Any]):
    wps = payload.get("waypoints", [])
    if not isinstance(wps, list) or len(wps) == 0:
        return {"ok": False, "error": "No waypoints provided."}
    await ros_node.send_mission_latlon(wps)
    return {"ok": True}


@app.post("/api/cancel")
async def api_cancel():
    await ros_node.cancel_mission()
    return {"ok": True}
