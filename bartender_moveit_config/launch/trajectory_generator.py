import pandas as pd
import numpy as np

def generate_bartender_sequence_auto_return(
    file_ready, 
    file_move_drink, 
    file_descend, 
    file_ascend, 
    file_to_user, 
    output_file
):
    print(f"Sequence: Ready -> MoveDrink -> Descend -> [50s Wait] -> Ascend -> ToUser -> [10s Wait] -> [Auto Return to Start]")
    
    # 1. 파일 5개 읽기 (복귀 파일은 읽지 않음)
    df1 = pd.read_csv(file_ready)       # 파일1: 준비 (초기값 포함)
    df2 = pd.read_csv(file_move_drink)  # 파일2: 음료 쪽으로 이동
    df3 = pd.read_csv(file_descend)     # 파일3: 하강
    df4 = pd.read_csv(file_ascend)      # 파일4: 상승
    df5 = pd.read_csv(file_to_user)     # 파일5: 유저에게 이동

    cols = df1.columns.tolist()
    joint_cols = cols[1:] # time 제외한 관절 컬럼들
    
    # ★ 초기 자세 저장 (나중에 돌아오기 위함) ★
    initial_pose = df1.iloc[0][joint_cols].values
    
    all_dfs = []
    dt = 0.1
    current_time_offset = 0.0

    # ---------------------------------------------------------
    # Phase 1: 파일1 (준비)
    # ---------------------------------------------------------
    df1_shifted = df1.copy()
    df1_shifted['time'] = df1_shifted['time'] + current_time_offset
    all_dfs.append(df1_shifted)
    current_time_offset = df1_shifted.iloc[-1]['time']

    # ---------------------------------------------------------
    # Phase 2: 파일2 (음료 쪽으로 이동)
    # ---------------------------------------------------------
    df2_shifted = df2.copy()
    df2_shifted['time'] = df2_shifted['time'] + current_time_offset
    all_dfs.append(df2_shifted)
    current_time_offset = df2_shifted.iloc[-1]['time']

    # ---------------------------------------------------------
    # Phase 3: 파일3 (하강)
    # ---------------------------------------------------------
    df3_shifted = df3.copy()
    df3_shifted['time'] = df3_shifted['time'] + current_time_offset
    all_dfs.append(df3_shifted)
    current_time_offset = df3_shifted.iloc[-1]['time']
    
    last_pose_descend = df3_shifted.iloc[-1][joint_cols].values 

    # ---------------------------------------------------------
    # Phase 4: 19초 대기 (하강 상태 유지)
    # ---------------------------------------------------------
    t_wait50 = np.arange(current_time_offset + dt, current_time_offset + 19.05, dt)
    
    data_wait50 = np.zeros((len(t_wait50), len(cols)))
    data_wait50[:, 0] = t_wait50
    data_wait50[:, 1:] = last_pose_descend
    
    df_wait50 = pd.DataFrame(data_wait50, columns=cols)
    all_dfs.append(df_wait50)
    current_time_offset = df_wait50.iloc[-1]['time']

    # ---------------------------------------------------------
    # Phase 5: 파일4 (상승)
    # ---------------------------------------------------------
    df4_shifted = df4.copy()
    df4_shifted['time'] = df4_shifted['time'] + current_time_offset
    all_dfs.append(df4_shifted)
    current_time_offset = df4_shifted.iloc[-1]['time']

    # ---------------------------------------------------------
    # Phase 6: 파일5 (유저에게 이동)
    # ---------------------------------------------------------
    df5_shifted = df5.copy()
    df5_shifted['time'] = df5_shifted['time'] + current_time_offset
    all_dfs.append(df5_shifted)
    current_time_offset = df5_shifted.iloc[-1]['time']
    
    last_pose_user = df5_shifted.iloc[-1][joint_cols].values

    # ---------------------------------------------------------
    # Phase 7: 10초 대기 (유저 앞 대기)
    # ---------------------------------------------------------
    t_wait10 = np.arange(current_time_offset + dt, current_time_offset + 10.05, dt)
    
    data_wait10 = np.zeros((len(t_wait10), len(cols)))
    data_wait10[:, 0] = t_wait10
    data_wait10[:, 1:] = last_pose_user
    
    df_wait10 = pd.DataFrame(data_wait10, columns=cols)
    all_dfs.append(df_wait10)
    current_time_offset = df_wait10.iloc[-1]['time']

    # ---------------------------------------------------------
    # Phase 8: 자동 복귀 (Auto Return to Initial Pose)
    # ---------------------------------------------------------
    # 파일6을 읽는 대신, 현재 위치에서 초기 위치로 5초간 부드럽게 이동하는 경로 생성
    return_duration = 5.0 # 복귀 소요 시간 (5초)
    t_return = np.arange(current_time_offset + dt, current_time_offset + return_duration + 0.001, dt)
    steps = len(t_return)
    
    data_return = np.zeros((steps, len(cols)))
    data_return[:, 0] = t_return # 시간 컬럼 채우기
    
    # 각 관절별로 [현재위치] -> [초기위치] 로 선형 보간(Linear Interpolation)
    for i in range(len(joint_cols)):
        start_val = last_pose_user[i] # 현재 위치 (유저 앞)
        end_val = initial_pose[i]     # 목표 위치 (맨 처음 자세)
        
        # linspace: 시작값부터 끝값까지 부드럽게 연결
        data_return[:, i+1] = np.linspace(start_val, end_val, steps)
        
    df_return = pd.DataFrame(data_return, columns=cols)
    all_dfs.append(df_return)

    # ---------------------------------------------------------
    # 저장
    # ---------------------------------------------------------
    df_final = pd.concat(all_dfs, ignore_index=True)
    df_final.to_csv(output_file, index=False)
    print(f"Success! Saved to {output_file} (Total points: {len(df_final)})")


# --- 실행 부분 (파일 인자가 하나 줄어듬) ---

# Case 1
generate_bartender_sequence_auto_return(
    './1_ready.csv',        # 파일1 (여기 시작점으로 나중에 돌아옴)
    './2_move_drink1.csv',
    './3_descend1.csv',
    './4_ascend1.csv',
    './5_to_user1.csv',
    './case1.csv'           # 결과
)

# Case 2
generate_bartender_sequence_auto_return(
    './1_ready.csv',
    './2_move_drink2.csv',
    './3_descend2.csv',
    './4_ascend2.csv',
    './5_to_user2.csv',
    './case2.csv'
)