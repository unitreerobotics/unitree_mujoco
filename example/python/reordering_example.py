import numpy as np

"""
Unitree Go2 로봇의 모터 인덱스 순서 변환 튜토리얼

Unitree SDK와 IsaacLab에서 사용하는 모터 순서가 다르기 때문에
데이터를 주고받을 때 인덱스 변환이 필요합니다.
"""

print("=" * 60)
print("Unitree Go2 모터 인덱스 순서 변환 튜토리얼")
print("=" * 60)

# 1. 각 시스템의 모터 순서 정의
print("\n1. 각 시스템의 모터 순서:")
print("-" * 30)

# Unitree SDK/Mujoco 순서 (실제 하드웨어/시뮬레이션 순서)
unitree_order = [
    "FR_hip", "FR_thigh", "FR_calf",    # 0, 1, 2
    "FL_hip", "FL_thigh", "FL_calf",    # 3, 4, 5
    "RR_hip", "RR_thigh", "RR_calf",    # 6, 7, 8
    "RL_hip", "RL_thigh", "RL_calf"     # 9, 10, 11
]

# IsaacLab 순서 (강화학습 정책에서 사용하는 순서)
isaaclab_order = [
    "FL_hip", "FR_hip", "RL_hip", "RR_hip",          # 0, 1, 2, 3
    "FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh",  # 4, 5, 6, 7
    "FL_calf", "FR_calf", "RL_calf", "RR_calf"       # 8, 9, 10, 11
]

print(f"Unitree SDK 순서: {unitree_order}")
print(f"IsaacLab 순서:   {isaaclab_order}")

# 2. 변환 인덱스 배열 생성
print("\n2. 변환 인덱스 배열:")
print("-" * 30)

# Unitree -> IsaacLab 변환: IsaacLab[i] = Unitree[UNITREE_TO_ISAACLAB[i]]
UNITREE_TO_ISAACLAB = np.array([
    3, 0, 9, 6,   
    4, 1, 10, 7,  
    5, 2, 11, 8   
], dtype=np.int32)

# IsaacLab -> Unitree 변환: Unitree[i] = IsaacLab[ISAACLAB_TO_UNITREE[i]]
ISAACLAB_TO_UNITREE = np.array([
    1, 5, 9,   
    0, 4, 8,   
    3, 7, 11,  
    2, 6, 10   
], dtype=np.int32)

print(f"Unitree -> IsaacLab 인덱스: {UNITREE_TO_ISAACLAB}")
print(f"IsaacLab -> Unitree 인덱스: {ISAACLAB_TO_UNITREE}")

# 3. 텍스트 변환 예시 (튜플/리스트 사용)
print("\n3. 텍스트 변환 예시:")
print("-" * 30)

# 튜플을 사용한 텍스트 매핑
def convert_unitree_to_isaaclab_names(unitree_names):
    """Unitree 모터 이름을 IsaacLab 순서로 변환"""
    UNITREE_TO_ISAACLAB = np.array([
        3, 0, 9, 6,   
        4, 1, 10, 7,  
        5, 2, 11, 8   
    ], dtype=np.int32)
    return [unitree_names[i] for i in UNITREE_TO_ISAACLAB]

def convert_isaaclab_to_unitree_names(isaaclab_names):
    """IsaacLab 모터 이름을 Unitree 순서로 변환"""
    ISAACLAB_TO_UNITREE = np.array([
        1, 5, 9,   
        0, 4, 8,   
        3, 7, 11,  
        2, 6, 10   
    ], dtype=np.int32)
    return [isaaclab_names[i] for i in ISAACLAB_TO_UNITREE]

# Unitree -> IsaacLab 변환
unitree_to_isaaclab_names = convert_unitree_to_isaaclab_names(unitree_order)
print(f"Unitree 순서를 IsaacLab 순서로 변환:")
print(f"  원본: {unitree_order}")
print(f"  결과: {unitree_to_isaaclab_names}")

# IsaacLab -> Unitree 변환  
isaaclab_to_unitree_names = convert_isaaclab_to_unitree_names(isaaclab_order)
print(f"\nIsaacLab 순서를 Unitree 순서로 변환:")
print(f"  원본: {isaaclab_order}")
print(f"  결과: {isaaclab_to_unitree_names}")

# 4. 숫자 인덱스 매핑 예시 (1~12)
print("\n4. 숫자 인덱스 매핑 예시 (1~12):")
print("-" * 30)

# 1부터 12까지의 숫자로 모터 인덱스 표현
unitree_indices = list(range(1, 13))  # [1, 2, 3, ..., 12]
isaaclab_indices = list(range(1, 13))

print(f"Unitree 인덱스 (1~12): {unitree_indices}")
print(f"IsaacLab 인덱스 (1~12): {isaaclab_indices}")

# Unitree -> IsaacLab 변환
unitree_to_isaaclab_indices = [unitree_indices[i] for i in UNITREE_TO_ISAACLAB]
print(f"\nUnitree 인덱스를 IsaacLab 순서로 재배열:")
print(f"  원본: {unitree_indices}")
print(f"  결과: {unitree_to_isaaclab_indices}")

# IsaacLab -> Unitree 변환
isaaclab_to_unitree_indices = [isaaclab_indices[i] for i in ISAACLAB_TO_UNITREE]
print(f"\nIsaacLab 인덱스를 Unitree 순서로 재배열:")
print(f"  원본: {isaaclab_indices}")
print(f"  결과: {isaaclab_to_unitree_indices}")

# 5. 실제 데이터 변환 예시 (numpy 사용)
print("\n5. 실제 데이터 변환 예시 (numpy 사용):")
print("-" * 30)

# 가상의 모터 위치 데이터 (Unitree SDK에서 받은 데이터라고 가정)
unitree_positions = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2])
print(f"Unitree SDK 모터 위치: {unitree_positions}")

# IsaacLab 순서로 변환
isaaclab_positions = unitree_positions[UNITREE_TO_ISAACLAB]
print(f"IsaacLab 순서로 변환: {isaaclab_positions}")

# 다시 Unitree 순서로 되돌리기
restored_positions = isaaclab_positions[ISAACLAB_TO_UNITREE]
print(f"다시 Unitree 순서로:  {restored_positions}")

# 검증
print(f"원본과 복원 데이터 일치: {np.allclose(unitree_positions, restored_positions)}")

print("\n" + "=" * 60)