#!/usr/bin/env python3
"""Nodo que lleva el efector final del ulite6 sobre el lienzo (canvas) y
traza, a una altura configurable sobre su superficie:

1. un cuadrado de 15x15 cm centrado en el lienzo,
2. regreso a la posicion inicial del brazo,
3. un triangulo equilatero de 15 cm de altura centrado en el lienzo,
4. regreso final a la posicion inicial.

Al terminar la trayectoria la simulacion queda abierta; se cierra
manualmente con Ctrl+C.

Calcula la cinematica inversa con PyKDL a partir del robot_description
(verificando posicion y orientacion de cada solucion, para que la
herramienta se mantenga perpendicular al lienzo) y envia una unica
trayectoria FollowJointTrajectory, con posiciones y velocidades para que
el controlador interpole con splines cubicas, al ulite6_traj_controller.
"""

import math

import PyKDL as kdl
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py import urdf as urdf_parser


ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def _urdf_origin_to_kdl_frame(origin):
    xyz = origin.xyz if origin is not None and origin.xyz is not None else [0.0, 0.0, 0.0]
    rpy = origin.rpy if origin is not None and origin.rpy is not None else [0.0, 0.0, 0.0]
    return kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))


def build_kdl_chain(robot, base_link, tip_link):
    """Construye la cadena KDL base->tip a partir del modelo URDF
    (misma conversion que kdl_parser)."""
    # camino tip -> base usando el mapa hijo->(joint, padre)
    path = []
    link = tip_link
    while link != base_link:
        if link not in robot.parent_map:
            raise ValueError('No hay camino de {} a {}'.format(tip_link, base_link))
        joint_name, parent = robot.parent_map[link]
        path.append((robot.joint_map[joint_name], link))
        link = parent
    path.reverse()

    chain = kdl.Chain()
    for joint, child_link in path:
        frame = _urdf_origin_to_kdl_frame(joint.origin)
        if joint.type in ('revolute', 'continuous'):
            axis = kdl.Vector(*joint.axis)
            kdl_joint = kdl.Joint(joint.name, frame.p, frame.M * axis, kdl.Joint.RotAxis)
        elif joint.type == 'prismatic':
            axis = kdl.Vector(*joint.axis)
            kdl_joint = kdl.Joint(joint.name, frame.p, frame.M * axis, kdl.Joint.TransAxis)
        else:
            kdl_joint = kdl.Joint(joint.name, kdl.Joint.Fixed)
        chain.addSegment(kdl.Segment(child_link, kdl_joint, frame))
    return chain


def joint_limits(robot, joint_names):
    lower, upper = [], []
    for name in joint_names:
        jnt = robot.joint_map[name]
        if jnt.type == 'continuous' or jnt.limit is None:
            lower.append(-2.0 * math.pi)
            upper.append(2.0 * math.pi)
        else:
            lower.append(jnt.limit.lower)
            upper.append(jnt.limit.upper)
    return lower, upper


class SquareDrawer(Node):

    def __init__(self):
        super().__init__('ulite6_draw_square')

        # pose de la base del robot en el mundo (la misma del spawn en gazebo)
        self.declare_parameter('base_xyz', [-0.2, -0.5, 1.021])
        self.declare_parameter('base_yaw', 1.571)
        # centro del lienzo y altura de su superficie en el mundo
        self.declare_parameter('canvas_center_xy', [-0.2, -0.78])
        self.declare_parameter('canvas_surface_z', 1.02)
        # geometria del trazo
        self.declare_parameter('square_side', 0.15)
        self.declare_parameter('triangle_height', 0.15)
        # altura del TCP al trazar: los dedos de la pinza terminan apenas
        # 2.5 mm por encima del TCP y Bullet agrega margen de colision de
        # unos mm, asi que con menos de ~1.5 cm los dedos tocan el lienzo,
        # el brazo se atasca y la trayectoria aborta (verificado en sim)
        self.declare_parameter('draw_offset', 0.02)
        self.declare_parameter('hover_offset', 0.06)   # altura de aproximacion
        self.declare_parameter('cartesian_speed', 0.03)  # m/s
        self.declare_parameter('tip_link', 'link_tcp')

        self.urdf_xml = None
        self.joint_state = None

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self._on_urdf, qos)
        self.create_subscription(JointState, 'joint_states', self._on_joints, 10)

        self._client = ActionClient(
            self, FollowJointTrajectory, '/ulite6_traj_controller/follow_joint_trajectory')

        self._timer = self.create_timer(1.0, self._try_start)
        self._started = False

    def _on_urdf(self, msg):
        self.urdf_xml = msg.data

    def _on_joints(self, msg):
        self.joint_state = msg

    def _try_start(self):
        if self._started:
            return
        if self.urdf_xml is None:
            self.get_logger().info('Esperando robot_description...')
            return
        if self.joint_state is None:
            self.get_logger().info('Esperando joint_states...')
            return
        if not self._client.server_is_ready():
            self.get_logger().info('Esperando al controlador de trayectorias...')
            return
        self._started = True
        self._timer.cancel()
        try:
            self._plan_and_send()
        except Exception as exc:  # errores de IK o de configuracion
            self.get_logger().error('Fallo al planificar: {}'.format(exc))
            raise

    # ------------------------------------------------------------------
    def _canvas_point_in_base(self, dx, dy, z_world):
        """Punto (dx, dy) relativo al centro del lienzo -> frame de la base."""
        bx, by, bz = self.get_parameter('base_xyz').value
        yaw = self.get_parameter('base_yaw').value
        cx, cy = self.get_parameter('canvas_center_xy').value
        wx, wy, wz = cx + dx, cy + dy, z_world
        # trasladar y rotar al frame de la base (Rz(-yaw))
        vx, vy, vz = wx - bx, wy - by, wz - bz
        c, s = math.cos(yaw), math.sin(yaw)
        return kdl.Vector(c * vx + s * vy, -s * vx + c * vy, vz)

    def _plan_and_send(self):
        robot = urdf_parser.Robot.from_xml_string(self.urdf_xml)
        tip = self.get_parameter('tip_link').value
        if tip not in robot.link_map:
            self.get_logger().warn('{} no existe, uso link_eef'.format(tip))
            tip = 'link_eef'
        chain = build_kdl_chain(robot, 'link_base', tip)
        n = chain.getNrOfJoints()
        assert n == 6, 'la cadena deberia tener 6 joints moviles, tiene {}'.format(n)

        lower, upper = joint_limits(robot, ARM_JOINTS)

        fk = kdl.ChainFkSolverPos_recursive(chain)
        # LMA es mas robusto que NR_JL para este brazo; los limites
        # articulares se verifican sobre cada solucion
        ik = kdl.ChainIkSolverPos_LMA(chain, eps=1e-6, maxiter=1000)

        surface_z = self.get_parameter('canvas_surface_z').value
        draw_z = surface_z + self.get_parameter('draw_offset').value
        hover_z = surface_z + self.get_parameter('hover_offset').value

        # esquinas del cuadrado centrado en el lienzo (sentido horario,
        # cerrando en la primera)
        side = self.get_parameter('square_side').value
        half = side / 2.0
        square = [(-half, -half), (-half, half), (half, half), (half, -half), (-half, -half)]

        # triangulo equilatero de altura h, centrado en el lienzo por su
        # caja envolvente: vertice superior en +h/2, base en -h/2
        h = self.get_parameter('triangle_height').value
        tri_half_base = h / math.sqrt(3.0)
        triangle = [(-tri_half_base, -h / 2.0), (0.0, h / 2.0),
                    (tri_half_base, -h / 2.0), (-tri_half_base, -h / 2.0)]

        # orientacion: herramienta apuntando hacia abajo (eje z del TCP = -z base)
        tool_down = kdl.Rotation.RPY(math.pi, 0.0, 0.0)

        def shape_frames(corners, steps_per_side=8):
            """Frames de una figura: aproximacion sobre la primera esquina,
            descenso, lados interpolados (trazo recto) y ascenso final.
            Devuelve tambien los indices donde el TCP debe detenerse
            (velocidad cero): aproximacion, contacto, esquinas y ascenso."""
            frames = [
                kdl.Frame(tool_down, self._canvas_point_in_base(*corners[0], hover_z)),
                kdl.Frame(tool_down, self._canvas_point_in_base(*corners[0], draw_z)),
            ]
            stops = {0, 1}
            for a, b in zip(corners[:-1], corners[1:]):
                for k in range(1, steps_per_side + 1):
                    t = k / steps_per_side
                    dx = a[0] + (b[0] - a[0]) * t
                    dy = a[1] + (b[1] - a[1]) * t
                    frames.append(kdl.Frame(tool_down, self._canvas_point_in_base(dx, dy, draw_z)))
                stops.add(len(frames) - 1)  # esquina: parada breve
            stops.add(len(frames))  # ascenso final
            frames.append(kdl.Frame(tool_down, self._canvas_point_in_base(*corners[-1], hover_z)))
            return frames, stops

        segments = [shape_frames(square), shape_frames(triangle)]

        # IK secuencial: primero la solucion anterior como semilla (para
        # continuidad) y si falla o viola limites, semillas nominales
        name_to_pos = dict(zip(self.joint_state.name, self.joint_state.position))
        current = [name_to_pos.get(jn, 0.0) for jn in ARM_JOINTS]
        nominal_seeds = [
            current,
            [math.pi, 0.6, 1.2, 0.0, 0.6, 0.0],
            [0.0, 0.6, 1.2, 0.0, 0.6, 0.0],
            [-math.pi / 2, 0.6, 1.2, 0.0, 0.6, 0.0],
            [math.pi, 0.3, 0.8, 0.0, 1.0, 0.0],
        ]

        def solve(frame, seeds):
            for s in seeds:
                q_seed = kdl.JntArray(n)
                for i in range(n):
                    q_seed[i] = s[i]
                q_out = kdl.JntArray(n)
                if ik.CartToJnt(q_seed, frame, q_out) < 0:
                    continue
                if not all(lower[i] - 1e-6 <= q_out[i] <= upper[i] + 1e-6 for i in range(n)):
                    continue
                f_chk = kdl.Frame()
                fk.JntToCart(q_out, f_chk)
                if (f_chk.p - frame.p).Norm() > 1e-4:
                    continue
                # verificar tambien la orientacion (herramienta perpendicular
                # al lienzo): rechazar soluciones con mas de ~0.06 grados
                rel = f_chk.M.Inverse() * frame.M
                if abs(kdl.Rotation.GetRotAngle(rel)[0]) > 1e-3:
                    continue
                return [q_out[i] for i in range(n)]
            return None

        # por cada figura: ir al primer waypoint, trazar a velocidad
        # cartesiana constante y regresar a la posicion inicial (home).
        # Las transiciones home<->figura son movimientos articulares grandes
        # (joint1 gira ~180 grados): se les da mas tiempo y velocidad cero en
        # los extremos para que la spline cubica no se desboque, y un punto
        # de asentamiento para que el error de seguimiento decaiga.
        speed = self.get_parameter('cartesian_speed').value
        solutions, times, stop_flags = [], [], []
        t = 0.0
        q_prev = None
        for seg_idx, (frames, stops) in enumerate(segments):
            seg_solutions = []
            for idx, frame in enumerate(frames):
                seeds = ([q_prev] if q_prev is not None else []) + nominal_seeds
                q = solve(frame, seeds)
                if q is None:
                    raise RuntimeError('IK sin solucion en figura {} waypoint {} ({})'.format(
                        seg_idx, idx, [frame.p.x(), frame.p.y(), frame.p.z()]))
                seg_solutions.append(q)
                q_prev = q

            t += 6.0  # desde home hasta la vertical de la primera esquina
            solutions.append(seg_solutions[0])
            times.append(t)
            stop_flags.append(True)
            for k, (f_prev, f_cur, q) in enumerate(
                    zip(frames[:-1], frames[1:], seg_solutions[1:]), start=1):
                dist = (f_cur.p - f_prev.p).Norm()
                t += max(dist / speed, 0.4)
                solutions.append(q)
                times.append(t)
                stop_flags.append(k in stops)

            # regreso a la posicion inicial + asentamiento
            t += 6.0
            solutions.append(list(current))
            times.append(t)
            stop_flags.append(True)
            t += 3.0
            solutions.append(list(current))
            times.append(t)
            stop_flags.append(True)

        # velocidades por diferencias centradas: con posiciones y velocidades
        # el controlador interpola con splines cubicas (sin velocidades usa
        # interpolacion lineal, con saltos de velocidad en cada waypoint que
        # deforman el trazo e inclinan el efector). En los puntos de parada
        # (aproximacion, contacto, esquinas, home) la velocidad es cero para
        # que la spline no oscile en los tramos largos.
        num = len(solutions)
        velocities = []
        for k in range(num):
            if k == 0 or k == num - 1 or stop_flags[k]:
                velocities.append([0.0] * len(ARM_JOINTS))
            else:
                dt = times[k + 1] - times[k - 1]
                velocities.append([
                    (solutions[k + 1][i] - solutions[k - 1][i]) / dt
                    for i in range(len(ARM_JOINTS))])

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        for q, v, t in zip(solutions, velocities, times):
            pt = JointTrajectoryPoint()
            pt.positions = list(q)
            pt.velocities = list(v)
            pt.time_from_start = Duration(seconds=t).to_msg()
            traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.get_logger().info(
            'Enviando trayectoria: {} puntos, duracion {:.1f} s'.format(len(traj.points), times[-1]))
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('El controlador rechazo la trayectoria')
            return
        self.get_logger().info('Trayectoria aceptada, trazando cuadrado y triangulo...')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        code = future.result().result.error_code
        if code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(
                'Cuadrado y triangulo completados; la simulacion queda abierta (cierrala con Ctrl+C)')
        else:
            self.get_logger().error('La trayectoria termino con error {}'.format(code))


def main():
    rclpy.init()
    node = SquareDrawer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
