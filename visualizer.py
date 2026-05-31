# visualizer.py
import vpython as vp
import config as cfg

class SkeletonVisualizer:
    def __init__(self):
        self._init_scene()
        # udp_server에서 quaternion 클래스를 가져오기 위해 임시 인스턴스 생성
        self.bones = {}
        self._create_skeleton()

    def _init_scene(self):
        vp.scene.title = "6-Node Modular Arm Tracker"
        vp.scene.width = 1200
        vp.scene.height = 800
        vp.scene.background = vp.color.gray(0.1)
        vp.scene.camera.pos = vp.vec(0, 0, 7)

    def _create_skeleton(self):
        default_color = vp.color.gray(0.5)
        # 1. Chest
        self.bones['Chest'] = vp.cylinder(pos=vp.vec(0, -cfg.CHEST_H/2, 0), axis=vp.vec(0, cfg.CHEST_H, 0), radius=cfg.BODY_RADIUS, color=vp.color.cyan)
        
        # 2. Left Arm
        self.bones['L_Shoulder'] = vp.sphere(radius=0.2, color=vp.color.red)
        self.bones['L_Upper'] = vp.cylinder(radius=0.15, axis=vp.vec(-cfg.UPPER_LEN,0,0), color=default_color)
        self.bones['L_Elbow'] = vp.sphere(radius=0.18, color=vp.color.red)
        self.bones['L_Lower'] = vp.cylinder(radius=0.12, axis=vp.vec(-cfg.LOWER_LEN,0,0), color=default_color)
        self.bones['L_Wrist'] = vp.sphere(radius=0.15, color=vp.color.red)
        self.bones['L_Hand'] = vp.box(length=0.2, width=0.1, height=0.4, color=default_color)

        # 3. Right Arm
        self.bones['R_Shoulder'] = vp.sphere(radius=0.2, color=vp.color.green)
        self.bones['R_Upper'] = vp.cylinder(radius=0.15, axis=vp.vec(cfg.UPPER_LEN,0,0), color=default_color)
        self.bones['R_Elbow'] = vp.sphere(radius=0.18, color=vp.color.green)
        self.bones['R_Lower'] = vp.cylinder(radius=0.12, axis=vp.vec(cfg.LOWER_LEN,0,0), color=default_color)
        self.bones['R_Wrist'] = vp.sphere(radius=0.15, color=vp.color.green)
        self.bones['R_Hand'] = vp.box(length=0.2, width=0.1, height=0.4, color=default_color)

    def update_pose(self, data_server):
        """서버 데이터를 받아 뼈대 위치 업데이트 (FK)"""
        chest_center = vp.vec(0,0,0)

        # --- Left Arm Process ---
        self._update_arm(
            side='L',
            shoulder_pos=chest_center + vp.vec(-cfg.BODY_RADIUS * 1.1, cfg.CHEST_H * 0.35, 0),
            ref_vec=cfg.V_LEFT,
            ids=(cfg.NODE_MAP['L_UPPER'], cfg.NODE_MAP['L_LOWER'], cfg.NODE_MAP['L_HAND']),
            server=data_server
        )

        # --- Right Arm Process ---
        self._update_arm(
            side='R',
            shoulder_pos=chest_center + vp.vec(cfg.BODY_RADIUS * 1.1, cfg.CHEST_H * 0.35, 0),
            ref_vec=cfg.V_RIGHT,
            ids=(cfg.NODE_MAP['R_UPPER'], cfg.NODE_MAP['R_LOWER'], cfg.NODE_MAP['R_HAND']),
            server=data_server
        )

    def _update_arm(self, side, shoulder_pos, ref_vec, ids, server):
        """팔 하나를 처리하는 내부 함수"""
        id_u, id_l, id_h = ids
        
        # 1. 센서로부터 각 부위의 '절대' 방향 쿼터니언을 가져옴
        q_world_upper = server.get_quat(id_u)
        q_world_lower = server.get_quat(id_l)
        q_world_hand = server.get_quat(id_h)

        # 2. FK(순기구학) 계층에 따라 최종 방향과 위치 계산
        # --- 위팔 (Upper Arm) ---
        self.bones[f'{side}_Shoulder'].pos = shoulder_pos
        self.bones[f'{side}_Upper'].pos = shoulder_pos
        final_q_upper = q_world_upper # 위팔은 월드에 직접 연결되므로 절대 방향이 최종 방향
        self.bones[f'{side}_Upper'].axis = final_q_upper.rotate(ref_vec) * cfg.UPPER_LEN
        self._update_color(id_u, self.bones[f'{side}_Upper'], server)

        # --- 아래팔 (Lower Arm) ---
        elbow_pos = self.bones[f'{side}_Upper'].pos + self.bones[f'{side}_Upper'].axis
        self.bones[f'{side}_Elbow'].pos = elbow_pos
        self.bones[f'{side}_Lower'].pos = elbow_pos
        # 아래팔의 최종 방향은 수학적으로 q_world_lower와 동일하지만, 계층 구조를 명확히 표현
        q_local_lower = q_world_upper.inverse() * q_world_lower # 위팔에 대한 아래팔의 '상대' 방향
        final_q_lower = final_q_upper * q_local_lower # 최종 방향 = 부모 방향 * 자식의 상대 방향
        self.bones[f'{side}_Lower'].axis = final_q_lower.rotate(ref_vec) * cfg.LOWER_LEN
        self._update_color(id_l, self.bones[f'{side}_Lower'], server)

        # --- 손 (Hand) ---
        wrist_pos = self.bones[f'{side}_Lower'].pos + self.bones[f'{side}_Lower'].axis
        self.bones[f'{side}_Wrist'].pos = wrist_pos
        # 손의 최종 방향 계산
        q_local_hand = q_world_lower.inverse() * q_world_hand # 아래팔에 대한 손의 '상대' 방향
        final_q_hand = final_q_lower * q_local_hand # 최종 방향 = 부모 방향 * 자식의 상대 방향
        self.bones[f'{side}_Hand'].pos = wrist_pos + final_q_hand.rotate(ref_vec) * (cfg.HAND_LEN/2)
        self.bones[f'{side}_Hand'].axis = final_q_hand.rotate(ref_vec) * cfg.HAND_LEN
        self.bones[f'{side}_Hand'].up = final_q_hand.rotate(cfg.V_FORWARD)
        self._update_color(id_h, self.bones[f'{side}_Hand'], server)

    def _update_color(self, node_id, bone_obj, server):
        """데이터 수신 시 색상 변경"""
        cnt = server.get_count(node_id)
        if cnt > 0 and cnt % 20 == 0:
            if 'Hand' in str(bone_obj): bone_obj.color = vp.color.red if node_id == cfg.NODE_MAP['L_HAND'] else vp.color.green
            else: bone_obj.color = vp.color.white